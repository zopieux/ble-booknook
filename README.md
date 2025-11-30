<!--

Download latest from https://github.com/adafruit/Adafruit_nRF52_Bootloader/releases/.

Update bootloader (not needed it comes pre-installed):

    adafruit-nrfutil --verbose dfu serial --package nice_nano_bootloader-0.9.2_s140_6.1.1.zip -p /dev/ttyACM0 -b 115200 --singlebank --touch 1200

-->

### Reset for entering bootloader upload

Short RST to GND *twice* in a short time.

### Upload (flash)

Mount the nice!nano fake disk drive (e.g. in Thunar), then:

```shell
$ tinygo flash -target=nicenano -serial=none
# or
$ tinygo flash -target=nicenano -monitor
```
