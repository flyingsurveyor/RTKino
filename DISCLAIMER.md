# Disclaimer

## RTKino Is Not a Commercial Product

RTKino is an open-source project developed and maintained by volunteers. It is not manufactured, sold, certified, or supported as a commercial product. There is no company behind it, no warranty department, no customer support hotline, and no quality assurance process beyond the diligence of its contributors and users.

## No Warranty

RTKino is provided **"as is"**, without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose, and non-infringement.

The entire risk as to the quality and performance of the software is with you. Should the software prove defective, you assume the cost of all necessary servicing, repair, or correction.

## No Liability

In no event shall the authors, contributors, or copyright holders be liable for any claim, damages, or other liability, whether in an action of contract, tort, or otherwise, arising from, out of, or in connection with the software or the use or other dealings in the software.

This includes, without limitation, any liability for:

- **Inaccurate positioning or measurements.** GNSS positioning is subject to errors from satellite geometry, atmospheric conditions, multipath, antenna quality, receiver firmware, obstructions, and many other factors beyond the control of this software. RTKino processes data from third-party hardware and correction services — it does not independently guarantee the accuracy of any measurement.

- **Data loss.** Raw observation logs, survey data, configuration files, and any other data stored by RTKino may be lost due to hardware failure, power loss, SD card corruption, software bugs, or other causes. Always maintain independent backups of critical data.

- **Property damage or personal injury.** RTKino controls no physical actuators, but it may be integrated into systems that do. The user is solely responsible for the safe integration and operation of any system that incorporates RTKino.

- **Financial loss.** This includes lost revenue, lost contracts, penalties, fines, cost overruns, or any other financial consequence arising from the use of or reliance on this software.

- **Professional liability.** If you use RTKino in professional surveying, engineering, construction, agriculture, or any other professional activity, you are solely responsible for the accuracy and validity of your work product. RTKino does not replace professional judgment, instrument calibration, independent verification, or regulatory compliance.

- **Consequential, incidental, special, or exemplary damages** of any kind, regardless of whether the authors have been advised of the possibility of such damages.

## Professional Responsibility

**It is the sole responsibility of the professional surveyor, engineer, or operator to:**

1. **Verify all measurements** independently before relying on them for any purpose.
2. **Understand the limitations** of GNSS positioning, including but not limited to: multipath errors, ionospheric and tropospheric delays, satellite geometry, antenna phase center variations, and the accuracy specifications of the receiver hardware in use.
3. **Comply with all applicable laws, regulations, and professional standards** governing surveying, mapping, and positioning in their jurisdiction.
4. **Validate that RTKino meets the requirements** of their specific application before deploying it in production.
5. **Maintain proper calibration** of all hardware components, including antennas, receivers, tripods, and leveling equipment.
6. **Apply appropriate quality control procedures** to all survey work, regardless of the equipment used.

No software — commercial or open-source — absolves the professional of these responsibilities.

## Third-Party Hardware and Services

RTKino interacts with third-party hardware (u-blox ZED-F9P, ESP32-S3, SD cards, OLED displays) and third-party services (NTRIP casters, VRS networks, NTP servers). The authors of RTKino have no control over and accept no responsibility for the reliability, accuracy, or availability of any third-party hardware or service.

## Regulatory Compliance

RTKino is firmware for a general-purpose microcontroller. It is not certified for any specific regulatory purpose. If your jurisdiction requires certified or type-approved equipment for surveying, mapping, or positioning, it is your responsibility to determine whether your use of RTKino complies with those requirements.

## Changes Without Notice

RTKino is under active development. Features may change, be removed, or behave differently between versions. The authors are under no obligation to maintain backward compatibility, provide migration paths, or notify users of changes.

---

*This disclaimer supplements and does not replace the warranty disclaimer and limitation of liability provisions in the [AGPL-3.0 License](LICENSE) under which RTKino is distributed.*
