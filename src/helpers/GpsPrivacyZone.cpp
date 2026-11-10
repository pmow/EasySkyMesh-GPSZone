#include "GpsPrivacyZone.h"

#if WITH_GPS_PRIVACY_ZONE
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#define GPS_ZONE_FILE     "/gps_zone"
#define GPS_ZONE_MAGIC    0x5A4F4E45UL   // 'ZONE'
#define GPS_ZONE_VERSION  1

GpsPrivacyZone gps_privacy_zone;

GpsPrivacyZone::GpsPrivacyZone() : _fs(NULL) {
  applyDefaults();
}

static bool validLatLon(double lat, double lon) {
  return !isnan(lat) && !isnan(lon) && lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
}

void GpsPrivacyZone::applyDefaults() {
  _enabled = 0;
  _set_mask = 0;
  memset(_lat, 0, sizeof(_lat));
  memset(_lon, 0, sizeof(_lon));

#if defined(GPS_ZONE_LAT1) && defined(GPS_ZONE_LON1) && defined(GPS_ZONE_LAT2) && defined(GPS_ZONE_LON2) \
 && defined(GPS_ZONE_LAT3) && defined(GPS_ZONE_LON3) && defined(GPS_ZONE_LAT4) && defined(GPS_ZONE_LON4)
  setPoint(0, GPS_ZONE_LAT1, GPS_ZONE_LON1);
  setPoint(1, GPS_ZONE_LAT2, GPS_ZONE_LON2);
  setPoint(2, GPS_ZONE_LAT3, GPS_ZONE_LON3);
  setPoint(3, GPS_ZONE_LAT4, GPS_ZONE_LON4);
  #if defined(GPS_ZONE_ENABLED)
    _enabled = GPS_ZONE_ENABLED ? 1 : 0;
  #else
    _enabled = 1;   // corners given at build time => on by default
  #endif
#endif
  rebuildPolygon();
}

bool GpsPrivacyZone::setPoint(int idx, double lat, double lon) {
  if (idx < 0 || idx >= GPS_ZONE_NUM_POINTS || !validLatLon(lat, lon)) return false;
  _lat[idx] = lat;
  _lon[idx] = lon;
  _set_mask |= (1 << idx);
  rebuildPolygon();
  return true;
}

void GpsPrivacyZone::clear() {
  _enabled = 0;
  _set_mask = 0;
  memset(_lat, 0, sizeof(_lat));
  memset(_lon, 0, sizeof(_lon));
  rebuildPolygon();
}

// Order the corners by angle around their centroid, so that entering them in any order
// still gives a proper (non bow-tie) quadrilateral.
void GpsPrivacyZone::rebuildPolygon() {
  double c_lat = 0, c_lon = 0;
  for (int i = 0; i < GPS_ZONE_NUM_POINTS; i++) { c_lat += _lat[i]; c_lon += _lon[i]; }
  c_lat /= GPS_ZONE_NUM_POINTS;
  c_lon /= GPS_ZONE_NUM_POINTS;

  int order[GPS_ZONE_NUM_POINTS];
  double ang[GPS_ZONE_NUM_POINTS];
  for (int i = 0; i < GPS_ZONE_NUM_POINTS; i++) {
    order[i] = i;
    ang[i] = atan2(_lat[i] - c_lat, _lon[i] - c_lon);
  }
  for (int i = 1; i < GPS_ZONE_NUM_POINTS; i++) {   // tiny insertion sort
    int o = order[i];
    int j = i - 1;
    while (j >= 0 && ang[order[j]] > ang[o]) { order[j + 1] = order[j]; j--; }
    order[j + 1] = o;
  }
  for (int i = 0; i < GPS_ZONE_NUM_POINTS; i++) {
    _poly_lat[i] = _lat[order[i]];
    _poly_lon[i] = _lon[order[i]];
  }

  // Is it a rectangle aligned with lat/lon lines? Then use a simple inclusive range check.
  _box_lat_lo = _box_lat_hi = _lat[0];
  _box_lon_lo = _box_lon_hi = _lon[0];
  for (int i = 1; i < GPS_ZONE_NUM_POINTS; i++) {
    if (_lat[i] < _box_lat_lo) _box_lat_lo = _lat[i];
    if (_lat[i] > _box_lat_hi) _box_lat_hi = _lat[i];
    if (_lon[i] < _box_lon_lo) _box_lon_lo = _lon[i];
    if (_lon[i] > _box_lon_hi) _box_lon_hi = _lon[i];
  }
  uint8_t seen = 0;   // bit per (lat lo/hi, lon lo/hi) combination
  bool on_grid = _box_lat_lo != _box_lat_hi && _box_lon_lo != _box_lon_hi;
  for (int i = 0; i < GPS_ZONE_NUM_POINTS && on_grid; i++) {
    bool lat_hi = _lat[i] == _box_lat_hi, lon_hi = _lon[i] == _box_lon_hi;
    if (!lat_hi && _lat[i] != _box_lat_lo) on_grid = false;
    if (!lon_hi && _lon[i] != _box_lon_lo) on_grid = false;
    seen |= 1 << ((lat_hi ? 2 : 0) + (lon_hi ? 1 : 0));
  }
  _is_box = on_grid && seen == 0x0F;
}

// Ray casting point-in-polygon test (treats lat/lon as planar, fine for zones up to
// many km across; does not support zones that cross the 180th meridian).
bool GpsPrivacyZone::contains(double lat, double lon) const {
  if (!isComplete() || !validLatLon(lat, lon)) return false;
  if (_is_box) {
    return lat >= _box_lat_lo && lat <= _box_lat_hi && lon >= _box_lon_lo && lon <= _box_lon_hi;
  }
  bool inside = false;
  for (int i = 0, j = GPS_ZONE_NUM_POINTS - 1; i < GPS_ZONE_NUM_POINTS; j = i++) {
    double yi = _poly_lat[i], xi = _poly_lon[i];
    double yj = _poly_lat[j], xj = _poly_lon[j];
    if (((yi > lat) != (yj > lat)) && (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

bool GpsPrivacyZone::shouldHide(double tx_lat, double tx_lon, double gps_lat, double gps_lon) const {
  if (!isActive()) return false;
  return contains(gps_lat, gps_lon) || contains(tx_lat, tx_lon);
}

// ---------------- persistence ----------------

void GpsPrivacyZone::begin(FILESYSTEM* fs) {
  _fs = fs;
  if (!load()) applyDefaults();
}

bool GpsPrivacyZone::load() {
  if (_fs == NULL || !_fs->exists(GPS_ZONE_FILE)) return false;
#if defined(RP2040_PLATFORM)
  File file = _fs->open(GPS_ZONE_FILE, "r");
#else
  File file = _fs->open(GPS_ZONE_FILE);
#endif
  if (!file) return false;

  uint32_t magic = 0;
  uint8_t version = 0, enabled = 0, mask = 0;
  double lat[GPS_ZONE_NUM_POINTS], lon[GPS_ZONE_NUM_POINTS];
  bool ok = file.read((uint8_t *)&magic, sizeof(magic)) == sizeof(magic)
         && file.read(&version, 1) == 1
         && file.read(&enabled, 1) == 1
         && file.read(&mask, 1) == 1
         && file.read((uint8_t *)lat, sizeof(lat)) == sizeof(lat)
         && file.read((uint8_t *)lon, sizeof(lon)) == sizeof(lon);
  file.close();
  if (!ok || magic != GPS_ZONE_MAGIC || version != GPS_ZONE_VERSION) return false;

  _enabled = enabled ? 1 : 0;
  _set_mask = 0;
  for (int i = 0; i < GPS_ZONE_NUM_POINTS; i++) {
    if ((mask & (1 << i)) && validLatLon(lat[i], lon[i])) {
      _lat[i] = lat[i]; _lon[i] = lon[i]; _set_mask |= (1 << i);
    } else {
      _lat[i] = 0; _lon[i] = 0;
    }
  }
  rebuildPolygon();
  return true;
}

bool GpsPrivacyZone::save() {
  if (_fs == NULL) return false;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  _fs->remove(GPS_ZONE_FILE);
  File file = _fs->open(GPS_ZONE_FILE, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  File file = _fs->open(GPS_ZONE_FILE, "w");
#else
  File file = _fs->open(GPS_ZONE_FILE, "w", true);
#endif
  if (!file) return false;
  uint32_t magic = GPS_ZONE_MAGIC;
  uint8_t version = GPS_ZONE_VERSION;
  file.write((uint8_t *)&magic, sizeof(magic));
  file.write(&version, 1);
  file.write(&_enabled, 1);
  file.write(&_set_mask, 1);
  file.write((uint8_t *)_lat, sizeof(_lat));
  file.write((uint8_t *)_lon, sizeof(_lon));
  file.close();
  return true;
}

// ---------------- settings (custom vars) ----------------

static const char* const zone_setting_names[] = { "zone", "zone1", "zone2", "zone3", "zone4" };

const char* GpsPrivacyZone::getSettingName(int i) const {
  if (i < 0 || i >= getNumSettings()) return NULL;
  return zone_setting_names[i];
}

// format degrees with 6 decimals without relying on printf float support
static char* fmtDeg(char* dp, double v) {
  long micro = lround(v * 1000000.0);
  if (micro < 0) { *dp++ = '-'; micro = -micro; }
  snprintf(dp, 24, "%ld.%06ld", micro / 1000000L, micro % 1000000L);
  return strchr(dp, 0);
}

const char* GpsPrivacyZone::getSettingValue(int i) const {
  if (i == 0) return _enabled ? "1" : "0";
  int idx = i - 1;
  if (idx < 0 || idx >= GPS_ZONE_NUM_POINTS) return NULL;
  if ((_set_mask & (1 << idx)) == 0) return "unset";
  char* dp = fmtDeg(_fmt_buf[idx], _lat[idx]);
  *dp++ = ';';
  fmtDeg(dp, _lon[idx]);
  return _fmt_buf[idx];
}

bool GpsPrivacyZone::setSettingValue(const char* name, const char* value) {
  if (name == NULL || value == NULL) return false;

  if (strcmp(name, "zone") == 0) {
    if (strcmp(value, "clear") == 0) {
      clear();
    } else {
      setEnabled(value[0] == '1');
    }
    save();
    return true;
  }

  if (strncmp(name, "zone", 4) == 0 && name[4] >= '1' && name[4] <= '0' + GPS_ZONE_NUM_POINTS && name[5] == 0) {
    int idx = name[4] - '1';
    double lat, lon;
    if (!parseLatLon(value, lat, lon)) return false;
    if (!setPoint(idx, lat, lon)) return false;
    save();
    return true;
  }
  return false;
}

// ---------------- text commands ----------------

// snprintf-append that never writes past the end, even when earlier output was truncated
static int appendf(char* buf, int len, int n, const char* fmt, ...) __attribute__((format(printf, 4, 5)));
static int appendf(char* buf, int len, int n, const char* fmt, ...) {
  if (n < 0 || n >= len - 1) return n;
  va_list ap;
  va_start(ap, fmt);
  int w = vsnprintf(buf + n, len - n, fmt, ap);
  va_end(ap);
  if (w < 0) return n;
  return (n + w >= len) ? len - 1 : n + w;
}

static const char* skipSpaces(const char* p) {
  while (*p == ' ' || *p == '\t') p++;
  return p;
}

// parses exactly 'count' numbers separated by spaces, ',' or ';' (e.g. "52.1 4.3", "52.1, 4.3")
static bool parseNumbers(const char* text, double* out, int count) {
  const char* p = skipSpaces(text);
  for (int k = 0; k < count; k++) {
    if (k > 0) {
      const char* q = skipSpaces(p);
      if (*q == ',' || *q == ';') q = skipSpaces(q + 1);
      else if (q == p) return false;   // need some separator between numbers
      p = q;
    }
    char* end;
    out[k] = strtod(p, &end);
    if (end == p) return false;
    p = end;
  }
  return *skipSpaces(p) == 0;           // no trailing junk
}

bool GpsPrivacyZone::parseLatLon(const char* text, double& lat, double& lon) {
  double v[2];
  if (!parseNumbers(text, v, 2)) return false;
  lat = v[0]; lon = v[1];
  return true;
}

// rectangle from two opposite corners: lat between lat1..lat2 and lon between lon1..lon2
bool GpsPrivacyZone::setBox(double lat1, double lon1, double lat2, double lon2) {
  if (!validLatLon(lat1, lon1) || !validLatLon(lat2, lon2) || lat1 == lat2 || lon1 == lon2) return false;
  double lat_lo = lat1 < lat2 ? lat1 : lat2, lat_hi = lat1 < lat2 ? lat2 : lat1;
  double lon_lo = lon1 < lon2 ? lon1 : lon2, lon_hi = lon1 < lon2 ? lon2 : lon1;
  setPoint(0, lat_hi, lon_lo);
  setPoint(1, lat_hi, lon_hi);
  setPoint(2, lat_lo, lon_hi);
  setPoint(3, lat_lo, lon_lo);
  return true;
}

int GpsPrivacyZone::formatStatus(char* reply, int reply_len, double cur_lat, double cur_lon, bool cur_valid) const {
  if (reply_len <= 0) return 0;
  reply[0] = 0;
  int n = appendf(reply, reply_len, 0, "zone %s", _enabled ? "on" : "off");
  if (!isComplete()) {
    n = appendf(reply, reply_len, n, " (needs 4 corners)");
  } else if (_enabled && cur_valid) {
    n = appendf(reply, reply_len, n, ", you are %s", contains(cur_lat, cur_lon) ? "inside: location hidden" : "outside");
  } else if (_enabled) {
    n = appendf(reply, reply_len, n, ", no GPS fix");
  }
  for (int i = 0; i < GPS_ZONE_NUM_POINTS; i++) {
    char buf[28];
    if (_set_mask & (1 << i)) {
      char* dp = fmtDeg(buf, _lat[i]);
      *dp++ = ' ';
      fmtDeg(dp, _lon[i]);
    } else {
      strcpy(buf, "unset");
    }
    n = appendf(reply, reply_len, n, "\n%d: %s", i + 1, buf);
  }
  return n;
}

bool GpsPrivacyZone::handleCommand(const char* cmd, char* reply, int reply_len,
                                   double cur_lat, double cur_lon, bool cur_valid) {
  cmd = skipSpaces(cmd);
  if (strncmp(cmd, "zone", 4) != 0) return false;
  const char* rest = cmd + 4;

  // zone1..zone4 <lat> <lon> | here
  if (*rest >= '1' && *rest <= '0' + GPS_ZONE_NUM_POINTS && (rest[1] == ' ' || rest[1] == 0)) {
    int idx = *rest - '1';
    const char* arg = skipSpaces(rest + 1);
    double lat, lon;
    if (strcmp(arg, "here") == 0) {
      if (!cur_valid) {
        snprintf(reply, reply_len, "Error: no GPS fix yet");
        return true;
      }
      lat = cur_lat; lon = cur_lon;
    } else if (*arg == 0) {   // show one corner
      snprintf(reply, reply_len, "zone%d: %s", idx + 1, getSettingValue(idx + 1));
      return true;
    } else if (!parseLatLon(arg, lat, lon)) {
      snprintf(reply, reply_len, "Error: use zone%d <lat> <lon>, e.g. zone%d 52.0100 4.0000", idx + 1, idx + 1);
      return true;
    }
    if (!setPoint(idx, lat, lon)) {
      snprintf(reply, reply_len, "Error: lat must be -90..90, lon -180..180");
      return true;
    }
    save();
    char buf[28];
    char* dp = fmtDeg(buf, lat);
    *dp++ = ' ';
    fmtDeg(dp, lon);
    reply[0] = 0;
    int n = appendf(reply, reply_len, 0, "zone%d set to %s", idx + 1, buf);
    if (!isComplete()) appendf(reply, reply_len, n, " (set all 4 corners)");
    else if (!_enabled) appendf(reply, reply_len, n, " (zone is off: send 'zone on')");
    return true;
  }

  if (*rest != 0 && *rest != ' ') return false;   // e.g. "zonefoo": not ours
  const char* sub = skipSpaces(rest);

  if (*sub == 0) {
    formatStatus(reply, reply_len, cur_lat, cur_lon, cur_valid);
  } else if (strcmp(sub, "on") == 0) {
    setEnabled(true);
    save();
    formatStatus(reply, reply_len, cur_lat, cur_lon, cur_valid);
  } else if (strcmp(sub, "off") == 0) {
    setEnabled(false);
    save();
    snprintf(reply, reply_len, "zone off: location is shared normally");
  } else if (strncmp(sub, "box", 3) == 0 && (sub[3] == ' ' || sub[3] == 0)) {
    double v[4];
    if (!parseNumbers(sub + 3, v, 4)) {
      snprintf(reply, reply_len, "Error: use zone box <lat1> <lon1> <lat2> <lon2> (two opposite corners)");
    } else if (!setBox(v[0], v[1], v[2], v[3])) {
      snprintf(reply, reply_len, "Error: corners must differ in both lat and lon (lat -90..90, lon -180..180)");
    } else {
      setEnabled(true);
      save();
      formatStatus(reply, reply_len, cur_lat, cur_lon, cur_valid);
    }
  } else if (strcmp(sub, "clear") == 0) {
    clear();
    save();
    snprintf(reply, reply_len, "zone cleared and off");
  } else {
    snprintf(reply, reply_len, "Usage: zone | zone on|off|clear | zone box <lat1> <lon1> <lat2> <lon2> | zone1..zone4 <lat> <lon>|here");
  }
  return true;
}

#endif  // WITH_GPS_PRIVACY_ZONE
