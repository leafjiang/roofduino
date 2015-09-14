# roofduino

## Mega 2560 Pinout for SD card

SD card needs 3.3V power.  Format with FAT32 file system.
```
const int chipSelect = 53;
```

| PIN | Connection      |
| --- | --------------- |
| 50  | SD - MISO       |
| 51  | SD - MOSI       |
| 52  | SD - CLK        |
| 53  | SD - CS         |

SD card test (Cardinfo.ino) ran successfully.