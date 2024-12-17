#pragma once
#ifndef GEO_MAG_DECLINATION_H
#define GEO_MAG_DECLINATION_H

#include "geo_magnetic_tables.hpp"

#include <math.h>
#include <stdint.h>

namespace mrs_uav_state_estimators
{

class GeoMagDeclination {

public:
  /*//{ constrain() */
  template <typename T>
  static T constrain(T val, T min_val, T max_val) {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
  }
  /*//}*/

  /*//{ degrees() */
  template <typename T>
  static T degrees(T radians) {
    return radians * (static_cast<T>(180) / static_cast<T>(M_PI));
  }
  /*//}*/

  /*//{ getLookupTableIndex() */
  static unsigned getLookupTableIndex(float *val, float min, float max) {
    *val = GeoMagDeclination::constrain(*val, min, max - SAMPLING_RES);

    return static_cast<unsigned>((-(min) + *val) / SAMPLING_RES);
  }
  /*//}*/

  /*//{ getTableData() */
  static float getTableData(float lat, float lon, const int16_t table[LAT_DIM][LON_DIM]) {

    lat = GeoMagDeclination::constrain(lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);

    if (lon > SAMPLING_MAX_LON) {
      lon -= 360;
    }

    if (lon < SAMPLING_MIN_LON) {
      lon += 360;
    }

    /* round down to nearest sampling resolution */
    float min_lat = floorf(lat / SAMPLING_RES) * SAMPLING_RES;
    float min_lon = floorf(lon / SAMPLING_RES) * SAMPLING_RES;

    /* find index of nearest low sampling point */
    unsigned min_lat_index = GeoMagDeclination::getLookupTableIndex(&min_lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
    unsigned min_lon_index = GeoMagDeclination::getLookupTableIndex(&min_lon, SAMPLING_MIN_LON, SAMPLING_MAX_LON);

    const float data_sw = table[min_lat_index][min_lon_index];
    const float data_se = table[min_lat_index][min_lon_index + 1];
    const float data_ne = table[min_lat_index + 1][min_lon_index + 1];
    const float data_nw = table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */
    const float lat_scale = GeoMagDeclination::constrain((lat - min_lat) / SAMPLING_RES, 0.f, 1.f);
    const float lon_scale = GeoMagDeclination::constrain((lon - min_lon) / SAMPLING_RES, 0.f, 1.f);

    const float data_min = lon_scale * (data_se - data_sw) + data_sw;
    const float data_max = lon_scale * (data_ne - data_nw) + data_nw;

    return lat_scale * (data_max - data_min) + data_min;
  }
  /*//}*/

  static float getMagDeclinationRadians(float lat, float lon) {
    return GeoMagDeclination::getTableData(lat, lon, declination_table) * 1e-4f;  // declination table stored as 10^-4 radians
  }

  static float getMagDeclinationDegrees(float lat, float lon) {
    return GeoMagDeclination::degrees(GeoMagDeclination::getMagDeclinationRadians(lat, lon));
  }

private:
  GeoMagDeclination(){};
};

}  // namespace mrs_uav_state_estimators


#endif
