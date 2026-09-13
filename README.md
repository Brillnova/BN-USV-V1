# BN-USV V1

BN-USV V1 is an experimental unmanned surface vehicle reference prototype by Brillnova. It was developed to validate the control architecture, navigation software and on-water behavior of a Raspberry Pi and ESP32-based USV.

V1 is **not a commercial product**, is **not for sale**, and is **not a ready-to-print DIY kit**. It is shared as a technical reference. The hull requires substantial post-processing, bonding and waterproofing, and is not optimized for easy reproduction.

For the next platform direction, see Brillnova's public updates on [GitHub](https://github.com/Brillnova) and [YouTube](https://www.youtube.com/@Brillnova).

## Included in this release

- Raspberry Pi navigation and control reference software
- ESP32 propulsion-control firmware, with OTA disabled until local credentials are configured
- Minimal dependency list and local configuration template
- V1 reference hull STL meshes and a FreeCAD assembly composed from those meshes
- Assembly and reference-hull licensing notes

## Not included

- V2 materials, source code or designs
- PCB designs, original CAD, manufacturing files or internal design documents
- Raw field logs, real waypoints, calibration values and network credentials
- BOM and wiring-diagram files; these are not part of this initial release

## Quick start

1. Review [assembly notes](docs/ASSEMBLY_NOTES.md) and the limitations below.
2. Copy `software/config.example.py` to `software/config.py`, then enter local ports, waypoints and calibration values. Do not commit this file.
3. Install the Python dependencies: `python3 -m pip install -r software/requirements.txt`.
4. Review the pin assignments and safety behavior in `firmware/BN_USV_V0.3.1/BN_USV_V0.3.1.ino` before flashing it to an ESP32.
5. Keep OTA disabled or set local credentials only in your working copy.

The software expects sensor hardware and system dependencies not bundled in this repository. This is a reference implementation, not a turnkey installation guide.

## Known limitations

- Hull assembly requires significant bonding and waterproofing.
- The design has limited modularity and difficult access to internal components.
- Wiring is prototype-oriented.
- Assembly support, success guarantees and performance guarantees are not provided.
- The BOM for future releases will include only parts directly tested by Brillnova; alternatives are not validated.

## Licensing

Unless a file states otherwise, the software in this repository is released under the [MIT License](LICENSE).

The V1 reference hull meshes and the FreeCAD mesh assembly are licensed separately under [CC BY-NC-SA 4.0](docs/REFERENCE_HULL_FILES.md). Branding, photographs and videos are not licensed for reuse by the software license.
