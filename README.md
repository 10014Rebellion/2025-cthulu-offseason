# 2025 Cthulu Robot Offseason
This is the offseason code for 2025 cthulu robot

## Motor Layout
* Drivebase
    * Pigeon: (x1)
        * ID: 10
    * Module: (x4)
        * Drive: NEO (x1)
            * FL ID: 11
            * FR ID: 12
            * BL ID: 14
            * BR ID: 13
        * Azimuth: Neo 550 (x1)
            * FL ID: 21
                * Encoder: Hex Bore (x1)
            * FR ID: 22
                * Encoder: Hex Bore (x1)
            * BL ID: 24
                * Encoder: Hex Bore (x1)
            * BR ID: 23
                * Encoder: Hex Bore (x1)

* Intake
    * Pivot: NEO (x1)
        * ID: 41
        * Encoder: Hex Bore (x1)
    * Roller: NEO (x1)
        * ID: 43
        * CAN Range (x1)
            * ID: 42

* Shooter
    * Flywheel:
        * Top Flywheel: Vortex (x1)
            * ID: 32
        * Bottom Flywheel: Vortex (x1)
            * ID: 33
    * Indexer: Neo Vortex (x1)
        * ID: 34
        * Beambreak Sensor:
            * DIO Port: 1

* Arm
    * Pivot: NEO
        * ID: 31
        * Encoder: Hex Bore (x1)


OFFSETS:

Tower Values:
* Encoder Offset (Revolutions): 0.8213854
* Encoder Bottom Hard Limit: -40 Deg
* Encoder Top Limit: 90 deg

Intake Values:
* Encoder Offset (Revolutions): 0.1413399
* Encoder Bottom Hard Limit: -30 Deg
* Encoder Top Limit: 70 deg
