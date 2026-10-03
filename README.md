# High-PuppyBot6 for Arduino IDE

Boards Manager URL:
https://raw.githubusercontent.com/Amornthep-ra/High-PuppyBot6/main/package_highpuppybot6_index.json

Install **High-PuppyBot6** in Boards Manager. Select Tools > Board > High-PuppyBot6.
Fixed hardware settings: RP2350B, ARM Cortex-M33, 150 MHz, 4 MB flash.
SDA0/SCL0: GPIO4/5. SDA1/SCL1: GPIO6/7. This is a separate package from princebot PuppyBot6.

Bundled library: HighPuppyBot6. Install dependencies using Library Manager:
Adafruit GFX Library, Adafruit ST7735 and ST7789 Library, Adafruit BusIO,
Adafruit MCP3008, Adafruit TCS34725, VL53L0X by Pololu.

Open examples/High-PuppyBot6-New/High-PuppyBot6-New.ino.
The same example is bundled under File > Examples > HighPuppyBot6 > High-PuppyBot6-New.
Edit RobotConfig.h for settings, RobotSensors.h for sensor logic,
RobotMotion.h for movements, RobotMission.h for the route.
The example preserves the latest supplied KBIDE mission.

Platform is based on Arduino-Pico 6.0.0 by Earle F. Philhower.
Upstream: https://github.com/earlephilhower/arduino-pico
License: see the platform release asset LICENSE and source file notices.
Custom drivers retain their original third-party notices.
Platform overrides are in platform/. Editable library source is in libraries/HighPuppyBot6/.

## Rebuild a release

Run `python build_package.py PATH_TO_ARDUINO_PICO_6.0.0`.
This reads the upstream installation, applies the platform/ overrides, bundles
the libraries/ sources, and updates the archive checksum and size in the index.
Upload the ZIP from dist/ to the matching GitHub release, then publish the index.
The upstream installation and other board packages are unchanged.

The compiler tools use the upstream rp2040 package namespace. Their download
metadata is included in this index, so this URL also works on a fresh installation.
Board FQBN: `highpuppybot6:rp2040:highpuppybot6`.
