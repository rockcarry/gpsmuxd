#ifndef __NMEA_H__
#define __NMEA_H__

#include <stdint.h>
#include <time.h>

typedef struct {
    int8_t  prn;
    int8_t  snr;
    int16_t elevation;
    int16_t azimuth;
    int8_t  inuse;
} GPS_SATELLITE;

typedef struct {
    #define   GPS_FIXED     1
    #define   GPS_FIXED_2D  2
    #define   GPS_FIXED_3D  3
    uint32_t  fixstatus;
    double    latitude ;
    double    longitude;
    float     altitude ;
    struct tm datetime ;
    float     speed ;
    float     course;
    float     pdop  ;
    float     hdop  ;
    float     vdop  ;
    int8_t    inuse ;
    int8_t    inview;
    GPS_SATELLITE satellites[32];
} GPS_STATUS;

typedef void (*PFN_NMEA_CALLBACK)(void *params, GPS_STATUS *pgs);

void* nmea_init(char *dev, PFN_NMEA_CALLBACK cb, void *params);
void  nmea_exit(void *ctx);
void* nmea_gps_status(void *ctx);
void  nmea_print(GPS_STATUS *pgs);

#endif

