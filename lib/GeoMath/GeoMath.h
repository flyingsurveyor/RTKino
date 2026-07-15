// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#pragma once
#include <math.h>
#include <vector>

// ---------------------------------------------------------------------------
// GeoMath — shared WGS84 great-circle distance / bearing helpers.
// Header-only so any module (Stakeout, main.cpp, ...) can use the exact same
// formulas without duplicating them.
// ---------------------------------------------------------------------------
namespace GeoMath {

  static const double DEG2RAD = M_PI / 180.0;
  static const double WGS84_A = 6378137.0;

  // Haversine great-circle distance (metres)
  inline double haversine2D(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * DEG2RAD;
    double dLon = (lon2 - lon1) * DEG2RAD;
    double a = sin(dLat / 2) * sin(dLat / 2)
             + cos(lat1 * DEG2RAD) * cos(lat2 * DEG2RAD)
               * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return WGS84_A * c;
  }

  // Forward azimuth from (lat1,lon1) to (lat2,lon2), degrees from North (0-360)
  inline double forwardAzimuth(double lat1, double lon1, double lat2, double lon2) {
    double lat1r = lat1 * DEG2RAD;
    double lat2r = lat2 * DEG2RAD;
    double dLon  = (lon2 - lon1) * DEG2RAD;
    double y = sin(dLon) * cos(lat2r);
    double x = cos(lat1r) * sin(lat2r) - sin(lat1r) * cos(lat2r) * cos(dLon);
    double az = atan2(y, x) / DEG2RAD;
    if (az < 0) az += 360.0;
    return az;
  }

  // Slope distance combining 2D great-circle distance with a height difference (metres)
  inline double distance3D(double dist2d, double dAlt) {
    return sqrt(dist2d * dist2d + dAlt * dAlt);
  }

  // Local delta North/East (metres) from (lat1,lon1) to (lat2,lon2), derived from
  // the same distance/bearing used elsewhere so dN^2+dE^2 == haversine2D^2 exactly.
  inline void localDeltaNE(double lat1, double lon1, double lat2, double lon2,
                            double& dN, double& dE) {
    double dist = haversine2D(lat1, lon1, lat2, lon2);
    double brg  = forwardAzimuth(lat1, lon1, lat2, lon2) * DEG2RAD;
    dN = dist * cos(brg);
    dE = dist * sin(brg);
  }

  // Spherical direct problem: point at `dist` metres, bearing `bearingDeg` from North,
  // starting at (lat1,lon1). Inverse of haversine2D+forwardAzimuth (same spherical model).
  inline void destinationPoint(double lat1, double lon1, double bearingDeg, double dist,
                                double& lat2, double& lon2) {
    double ang1 = lat1 * DEG2RAD;
    double lon1r = lon1 * DEG2RAD;
    double brg  = bearingDeg * DEG2RAD;
    double dR   = dist / WGS84_A;

    double ang2 = asin(sin(ang1) * cos(dR) + cos(ang1) * sin(dR) * cos(brg));
    double lon2r = lon1r + atan2(sin(brg) * sin(dR) * cos(ang1),
                                  cos(dR) - sin(ang1) * sin(ang2));
    lat2 = ang2 / DEG2RAD;
    lon2 = lon2r / DEG2RAD;
  }

  // Forward intersection: two known points A, B and the horizontal angles turned at
  // each station from the A-B baseline to the unknown point P (as read off a total
  // station). `rightSide` picks which of the two mirror-image solutions across the
  // baseline to keep (true = clockwise/right of A->B, false = counter-clockwise/left).
  // Returns false if the angles don't form a valid triangle (angA+angB outside 0-180).
  inline bool forwardIntersection(double latA, double lonA, double latB, double lonB,
                                   double angA_deg, double angB_deg, bool rightSide,
                                   double& latP, double& lonP,
                                   double& distAP, double& distBP) {
    if (angA_deg <= 0 || angB_deg <= 0 || (angA_deg + angB_deg) >= 180.0) return false;

    double baseline = haversine2D(latA, lonA, latB, lonB);
    double azAB      = forwardAzimuth(latA, lonA, latB, lonB);
    double angP_deg  = 180.0 - angA_deg - angB_deg;

    double sinP = sin(angP_deg * DEG2RAD);
    if (fabs(sinP) < 1e-9) return false;

    distAP = baseline * sin(angB_deg * DEG2RAD) / sinP;
    distBP = baseline * sin(angA_deg * DEG2RAD) / sinP;

    double azAP = azAB + (rightSide ? angA_deg : -angA_deg);
    destinationPoint(latA, lonA, azAP, distAP, latP, lonP);
    return true;
  }

  // Trilateration: two known points A, B and the measured distances from each to the
  // unknown point P (e.g. a horizontal disto in forced centring). Same `rightSide`
  // convention as forwardIntersection. Returns false if distA/distB/baseline don't
  // satisfy the triangle inequality (no intersection exists).
  inline bool trilaterate(double latA, double lonA, double latB, double lonB,
                           double distA, double distB, bool rightSide,
                           double& latP, double& lonP) {
    double baseline = haversine2D(latA, lonA, latB, lonB);
    if (baseline < 1e-6 || distA <= 0 || distB <= 0) return false;
    if (distA + distB < baseline || fabs(distA - distB) > baseline) return false;

    double cosA = (distA * distA + baseline * baseline - distB * distB) / (2.0 * distA * baseline);
    if (cosA > 1.0) cosA = 1.0;
    if (cosA < -1.0) cosA = -1.0;
    double angA_deg = acos(cosA) / DEG2RAD;

    double azAB = forwardAzimuth(latA, lonA, latB, lonB);
    double azAP = azAB + (rightSide ? angA_deg : -angA_deg);
    destinationPoint(latA, lonA, azAP, distA, latP, lonP);
    return true;
  }

  // Chainage (progressive distance along A->B) and signed perpendicular offset of point P
  // relative to the A->B alignment. offset > 0 means P is to the right of A->B (clockwise),
  // consistent with the `rightSide` convention used by forwardIntersection/trilaterate.
  inline void chainageOffset(double latA, double lonA, double latB, double lonB,
                              double latP, double lonP,
                              double& chainage, double& offset) {
    double azAB   = forwardAzimuth(latA, lonA, latB, lonB);
    double azAP   = forwardAzimuth(latA, lonA, latP, lonP);
    double distAP = haversine2D(latA, lonA, latP, lonP);
    double dAng   = (azAP - azAB) * DEG2RAD;
    chainage = distAP * cos(dAng);
    offset   = distAP * sin(dAng);
  }

  // Planar area (m^2, shoelace) and perimeter (m) of a closed polygon given as ordered
  // lat/lon vertices, projected onto a local tangent plane around the first vertex.
  // Accurate for survey-scale parcels; not meant for large-scale geodetic areas.
  inline void polygonAreaPerimeter(const std::vector<double>& lat, const std::vector<double>& lon,
                                    double& area, double& perimeter) {
    area = 0.0; perimeter = 0.0;
    int n = (int)lat.size();
    if (n < 3) return;

    std::vector<double> N(n), E(n);
    for (int i = 0; i < n; i++) {
      localDeltaNE(lat[0], lon[0], lat[i], lon[i], N[i], E[i]);
    }

    double sum = 0.0;
    for (int i = 0; i < n; i++) {
      int j = (i + 1) % n;
      sum += E[i] * N[j] - E[j] * N[i];
      perimeter += hypot(E[j] - E[i], N[j] - N[i]);
    }
    area = fabs(sum) / 2.0;
  }

} // namespace GeoMath
