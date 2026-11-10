#pragma once

// GPS privacy zone
// ----------------
// A 4-corner area (quadrilateral) inside which this node never transmits its location.
// While the node's GPS position (or the location it is about to send) is inside the zone,
// adverts go out without lat/lon, GPS telemetry is omitted and tracker reports are skipped.
//
// Compiled in only with -D WITH_GPS_PRIVACY_ZONE=1. build.sh sets it for every *_companion_radio_*
// target; RAK_WisMesh_Tag_companion_radio_ble also sets it in its platformio.ini for plain `pio run`.
//
// Configure at build time (platformio.ini build_flags):
//   -D GPS_ZONE_ENABLED=1
//   -D GPS_ZONE_LAT1=52.0001 -D GPS_ZONE_LON1=4.0001   (corner 1)
//   ... up to GPS_ZONE_LAT4 / GPS_ZONE_LON4             (corner 4)
//
// Or at runtime via custom vars (companion app / meshcore-cli, or `sensor set` on the CLI):
//   zone   = 0 | 1
//   zone1 .. zone4 = "<lat>;<lon>"   (a ',' is also accepted as separator)
// Runtime values are saved to /gps_zone and override the build-time defaults.
//
// Corners can be given in any order; they are sorted around their centre so the zone is
// always a simple (non-self-intersecting) shape.

#ifndef WITH_GPS_PRIVACY_ZONE
  #define WITH_GPS_PRIVACY_ZONE 0
#endif

#if WITH_GPS_PRIVACY_ZONE

#include <helpers/IdentityStore.h>   // for FILESYSTEM

#define GPS_ZONE_NUM_POINTS  4

class GpsPrivacyZone {
  uint8_t _enabled;
  uint8_t _set_mask;                       // bit i => corner i has been configured
  double  _lat[GPS_ZONE_NUM_POINTS];
  double  _lon[GPS_ZONE_NUM_POINTS];
  double  _poly_lat[GPS_ZONE_NUM_POINTS];  // corners ordered around the centroid
  double  _poly_lon[GPS_ZONE_NUM_POINTS];
  bool    _is_box;                         // corners form a north-aligned rectangle
  double  _box_lat_lo, _box_lat_hi, _box_lon_lo, _box_lon_hi;
  FILESYSTEM* _fs;
  mutable char _fmt_buf[GPS_ZONE_NUM_POINTS][28];

  void applyDefaults();
  void rebuildPolygon();
  bool load();
  static bool parseLatLon(const char* text, double& lat, double& lon);
  int formatStatus(char* reply, int reply_len, double cur_lat, double cur_lon, bool cur_valid) const;

public:
  GpsPrivacyZone();

  void begin(FILESYSTEM* fs);   // loads saved config (if any)
  bool save();

  bool isEnabled() const { return _enabled != 0; }
  bool isComplete() const { return _set_mask == ((1 << GPS_ZONE_NUM_POINTS) - 1); }
  bool isActive() const { return isEnabled() && isComplete(); }

  void setEnabled(bool on) { _enabled = on ? 1 : 0; }
  bool setPoint(int idx, double lat, double lon);
  bool setBox(double lat1, double lon1, double lat2, double lon2);   // rectangle from 2 opposite corners
  void clear();

  bool contains(double lat, double lon) const;

  // true if location must NOT be transmitted. 'gps_*' is the node's live GPS position,
  // 'tx_*' the coordinates about to be sent (they can differ, e.g. a fixed advert location).
  bool shouldHide(double tx_lat, double tx_lon, double gps_lat, double gps_lon) const;
  bool shouldHide(double lat, double lon) const { return shouldHide(lat, lon, lat, lon); }

  // custom-var style settings: "zone", "zone1".."zone4"
  int getNumSettings() const { return 1 + GPS_ZONE_NUM_POINTS; }
  const char* getSettingName(int i) const;
  const char* getSettingValue(int i) const;
  bool setSettingValue(const char* name, const char* value);   // saves on success

  // Text commands for the companion command line (app "Command Line" screen / USB serial):
  //   zone                      show status and corners
  //   zone on | off | clear
  //   zone box <lat1> <lon1> <lat2> <lon2>   rectangle from two opposite corners (turns zone on)
  //   zone1 <lat> <lon>         set a corner (zone1..zone4; ',' or ';' also accepted)
  //   zone1 here                set a corner to the current GPS fix
  // 'cur_*' is the current GPS position, 'cur_valid' whether it is a real fix.
  // Returns false if 'cmd' is not a zone command.
  bool handleCommand(const char* cmd, char* reply, int reply_len,
                     double cur_lat, double cur_lon, bool cur_valid);
};

extern GpsPrivacyZone gps_privacy_zone;

#define GPS_ZONE_HIDE(lat, lon)  gps_privacy_zone.shouldHide((lat), (lon))
#define GPS_ZONE_BEGIN(fs)       gps_privacy_zone.begin(fs)

#else   // feature compiled out: location is never hidden by a zone

#define GPS_ZONE_HIDE(lat, lon)  false
#define GPS_ZONE_BEGIN(fs)       ((void)0)

#endif
