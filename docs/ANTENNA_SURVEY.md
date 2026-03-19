# Survey Base Position with Antenna Height Correction

## Overview

This feature implements professional GNSS surveying workflows for RTKino base station setup. The key principle is:

**Ground coordinates are stored, but phase center coordinates are sent to the ZED-F9P.**

## The GNSS Surveying Flow

```
GROUND POINT (benchmark)
Known coordinates: LAT, LON, H_ellipsoidal_ground
         │
         │  + H_antenna_ARP (measured each time, e.g. 1.523m)
         │
         ▼
ARP (Antenna Reference Point)
         │
         │  + Antenna offset (fixed per model, e.g. 0.065m)
         │
         ▼
L1/L2 PHASE CENTER  ← THIS GOES TO ZED-F9P!
```

## Key Formulas

### Starting a Base Station
```
H_to_send = H_ground + H_antenna_ARP + Antenna_offset
```

Example:
- Ground point: H_ground = 118.456 m
- Antenna height: H_ARP = 1.523 m
- Antenna: u-blox ANN-MB (offset = 0.065 m)
- **Result**: H_to_send = 118.456 + 1.523 + 0.065 = **120.044 m** (sent to ZED-F9P)

### Survey Mode (Reverse Calculation)
```
H_ground = H_measured - H_antenna_ARP - Antenna_offset
```

Example:
- Measured position: H_measured = 120.044 m
- Antenna height: H_ARP = 1.523 m
- Antenna: u-blox ANN-MB (offset = 0.065 m)
- **Result**: H_ground = 120.044 - 1.523 - 0.065 = **118.456 m** (stored in bases.txt)

## Files

### Antenna Models (`/gnss/antennas.txt`)

See [examples/antennas.txt](examples/antennas.txt) for sample configuration.

Format: `name;offset`

### Base Stations (`/gnss/bases.txt`)

See [examples/bases_new_format.txt](examples/bases_new_format.txt) for new format.
See [examples/bases_old_format.txt](examples/bases_old_format.txt) for old format.

**New format** (7 fields): `name;lat;lon;altGround;stid;hARP;antennaIdx`
**Old format** (5 fields): `name;lat;lon;alt;stid` (still supported)

## UI Features

### 1. Settings Page → Antenna Models
- Add, edit, and delete antenna models
- Each antenna has a name and phase center offset

### 2. Base Page → Saved Base Stations
Table shows all base station information including:
- H ground (ellipsoidal altitude of ground point)
- H ARP (antenna height to reference point)
- Antenna model name

### 3. Base Page → Add/Edit Base Station
Forms include:
- H ground field (meters, ellipsoidal)
- H antenna ARP field (meters, ground to ARP)
- Antenna model dropdown

### 4. Base Page → Survey Mode
- Select antenna model from dropdown
- Enter H antenna ARP (ground to ARP)
- ARP offset is automatically filled from antenna selection
- Results show both H (ARP) and H (Ground)
- Save function stores ground coordinates

## Important Notes

### Zero Values Are Valid
- `offset = 0` is OK (patch antennas without significant offset)
- `hARP = 0` is OK (antenna mounted directly on pillar)

### Backward Compatibility
Old base station files (5 fields) are automatically detected and loaded:
- `alt` is treated as `altGround`
- `hARP` defaults to 0.0
- `antennaIdx` defaults to -1 (none)
- Files are auto-converted to new format when saved

## API Endpoints

- `GET /api/antennas` - List all antenna models
- `POST /antennas/add` - Add antenna model
- `GET /antennas/edit?idx=X` - Edit antenna
- `GET /antennas/del?idx=X` - Delete antenna
- `POST /antennas/update` - Update antenna

## Migration Guide

### For New Installations
1. Configure antenna models in Settings page
2. Add base stations with antenna information
3. Use survey mode to establish new base points

### For Existing Installations
1. Existing base stations will load with `hARP=0` and no antenna
2. Edit each base station to add proper antenna information
3. Files will auto-convert to new format when saved

## Testing Checklist

Before deploying to production:
- [ ] Test antenna CRUD operations
- [ ] Verify old bases.txt files load correctly
- [ ] Test survey mode with different antennas
- [ ] Verify base start calculations with known benchmarks
- [ ] Test zero values (offset=0, h_arp=0)
- [ ] Verify UI on mobile devices

## References

This implementation follows standard GNSS surveying practices where:
- Ground points (benchmarks) are the fundamental reference
- Antenna heights are measured for each setup
- Phase center corrections are applied per antenna model
- The receiver always gets phase center coordinates
