# Third Party Dependency Notice

This board package includes third-party source code. This inventory records the notices found in the supplied files; it is not a legal clearance.

| Component | Location | Notice found in source |
| --- | --- | --- |
| Adafruit MCP3008 | `include/Adafruit_MCP3008.*` | MIT notice; attribution text must be retained |
| Adafruit TCS34725 | `include/Adafruit_TCS34725.*` | Redistribution notice embedded in source |
| Adafruit GFX/ST77xx/BusIO | `include/Adafruit_*` and `include/Fonts` | Copyright and license notices embedded in source files |
| PIO Encoder | `include/pio_encoder.*`, `include/quadrature.pio.h` | GNU Lesser General Public License notice |
| InvenSense ICM-20948 stack | `include/Invn`, `include/ICM20948*`, `include/HAL*` | Restricted-use notices refer to an express InvenSense software license agreement |

Before redistributing this package:

1. Obtain and review the governing InvenSense license agreement.
2. Include the complete license texts required by Adafruit and PIO Encoder components.
3. Preserve copyright notices in source and binary distributions.
4. Generate a source-to-binary dependency manifest for the exact release build.
5. Exclude build caches, object files, ELF, MAP and local machine paths from the public archive.

The current package does not contain enough top-level license material to confirm redistribution clearance.

## Pololu VL53L0X 1.3.1
MIT license. Vendored from https://github.com/pololu/vl53l0x-arduino/tree/1.3.1; see include/vl53l0x/LICENSE.txt. VL53L0X.cpp stored as VL53L0X.inc.
