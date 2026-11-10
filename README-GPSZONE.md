# EasySkyMesh with a GPS privacy zone

This is [IoTThinks/MeshCore](https://github.com/IoTThinks/MeshCore) (EasySkyMesh, branch `PowerSaving-v18`)
with three additions to the **companion** firmwares. Repeaters, room servers and sensors are unchanged.

## GPS privacy zone

While the node's GPS position is inside the zone it never transmits its location:
adverts go out without lat/lon and GPS telemetry is left out. Everything else works as normal.

Set it from the MeshCore app's **Command Line** screen (bottom of Settings) or the USB serial console:

| Command | Meaning |
|---|---|
| `zone box <lat1> <lon1> <lat2> <lon2>` | rectangle from two opposite corners (turns the zone on) |
| `zone1` … `zone4 <lat> <lon>` | set one corner of a 4-corner zone |
| `zone1` … `zone4 here` | set a corner to the current GPS fix |
| `zone` | show status, corners and whether you're inside |
| `zone on` / `zone off` / `zone clear` | turn on, turn off, erase |

Coordinates are decimal degrees, latitude first. The settings are saved and survive reboots.
Zone commands are refused when sent over remote CLI (LoRa), so the corners can only be read or changed locally.

## Automatic adverts

| Command | Range |
|---|---|
| `set advert.interval <minutes>` | 60–240, `0` = off (zero-hop) |
| `set flood.advert.interval <hours>` | 3–168, `0` = off (flood) |
| `get advert.interval` / `get flood.advert.interval` | show the setting |

Both are off by default.

## Boot advert

One zero-hop advert is sent 15 seconds after every boot.

All adverts follow the app's location-sharing setting and the GPS privacy zone.

## Building

Use `build.sh`, which turns on the zone (`-DWITH_GPS_PRIVACY_ZONE=1`) for every `*_companion_radio_*` target:

```bash
FIRMWARE_VERSION=gpszone bash build.sh build-firmware RAK_WisMesh_Tag_companion_radio_ble
```

The **Build Companion Firmwares** GitHub workflow builds every companion target and attaches the files to the run.
