/*
 * gps.h
 *
 *  Created on: 22/07/2014
 *      Author: allan_amorim
 */

#ifndef GPS_H_
#define GPS_H_
#include <stdint.h>
#define GPS 1
#define NMEA 1

#define GPS_SERIAL 2
#define GPS_BAUD   57600
#define MTK_BINARY19
#define INIT_MTK_GPS
#define NAV_MODE_NONE 0

//navigation mode
#define NAV_MODE_NONE          0
#define NAV_MODE_POSHOLD       1
#define NAV_MODE_WP            2
extern uint8_t nav_mode; // Navigation mode
extern int16_t  nav[2];
extern int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

// default POSHOLD control gains
#define POSHOLD_P              .11
#define POSHOLD_I              0.0
#define POSHOLD_IMAX           20        // degrees

#define POSHOLD_RATE_P         2.0
#define POSHOLD_RATE_I         0.08      // Wind control
#define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_IMAX      20        // degrees

// default Navigation PID gains
#define NAV_P                  1.4
#define NAV_I                  0.20      // Wind control
#define NAV_D                  0.08      //
#define NAV_IMAX               20        // degrees

#define LAT  0
#define LON  1

extern int32_t wrap_18000(int32_t ang);

void GPS_set_pids(void);
void GPS_SerialInit(void);
void GPS_NewData(void);
void GPS_reset_home_position(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
void GPS_reset_nav(void);

#endif /* GPS_H_ */

