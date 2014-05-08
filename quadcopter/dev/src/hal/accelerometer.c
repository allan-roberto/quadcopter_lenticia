/*
 * accelerometer.c
 *
 *  Created on: Feb 11, 2014
 *      Author: root
 */

#include <accelerometer.h>
#include <i2c.h>

uint8_t rawADC[6];

uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop

int16_t  magHold,headFreeModeHold; // [-180;+180]

int32_t  AltHold; // in cm
int16_t  sonarAlt;
int16_t  BaroPID = 0;
int16_t  errorAltitudeI = 0;

conf_t conf;
imu_t imu;
uint16_t calibratingA = 1;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint32_t currentTime;
uint16_t previousTime;
global_conf_t global_conf;

#define _1G 		(float)(0.00013 * 9.8)
#define _1_5G		(float)(0.00019 * 9.8)
#define _2G			(float)(0.00025 * 9.8)
#define _3G			(float)(0.00038 * 9.8)
#define _4G			(float)(0.00050 * 9.8)
#define _8G			(float)(0.00099 * 9.8)
#define _16G		(float)(0.00198 * 9.8)

#define GRAV_FACTOR _8G



void init_accelerometer(void) {
  _delay_ms(10);
  //default range 2G: 1G = 4096 unit.
  // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
  i2c_writeReg(BMA180_ADDRESS,0x0D,1<<4);
  _delay_ms(10);
  uint8_t control = i2c_readReg(BMA180_ADDRESS, 0x20);
  control = control & 0x0F;        // save tcs register
  // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 20Hz
  control = control | (0x04 << 4);
  //control = control | (0x00 << 4); // set low pass filter to 10Hz (bits value = 0000xxxx)
  i2c_writeReg(BMA180_ADDRESS, 0x20, control);
  _delay_ms(10);
  control = i2c_readReg(BMA180_ADDRESS, 0x30);
  control = control & 0xFC;        // save tco_z register
  control = control | 0x00;        // set mode_config to 0
  i2c_writeReg(BMA180_ADDRESS, 0x30, control);
  _delay_ms(10);
  control = i2c_readReg(BMA180_ADDRESS, 0x35);
  control = control & 0xF1;        // save offset_x and smp_skip register
  control = control | (0x01 << 1); // set range to 8G
  i2c_writeReg(BMA180_ADDRESS, 0x35, control);
  _delay_ms(10);
}

void accelerometer_get_data(float *X, float *Y, float *Z ) {
  TWBR = ((F_CPU / 400000L) - 16) / 2;  // Optional line.  Sensor is good for it in the spec.
  i2c_getSixRawADC(BMA180_ADDRESS,0x02);
  ACC_ORIENTATION( ((rawADC[1] << 8) | rawADC[0])>>2,
                   ((rawADC[3] << 8) | rawADC[2])>>2,
                   ((rawADC[5] << 8) | rawADC[4])>>2);
  ACC_Common();

  imu.accADC[ROLL] = ~(imu.accADC[ROLL])-1;
  imu.accADC[PITCH] = ~(imu.accADC[PITCH])-1;
  imu.accADC[YAW] = ~(imu.accADC[YAW])-1;

  *X = (imu.accADC[ROLL]) * GRAV_FACTOR;
  *Y = (imu.accADC[PITCH]) * GRAV_FACTOR;
  *Z = (imu.accADC[YAW]) * GRAV_FACTOR;

}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  if (calibratingA>0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // Reset a[axis] at start of calibration
      if (calibratingA == 100) a[axis]=0;
      // Sum up 512 readings
      a[axis] +=imu.accADC[axis];
      // Clear global variables for next reading
      imu.accADC[axis]=0;
      global_conf.accZero[axis]=0;
    }
    // Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) {
      global_conf.accZero[ROLL]  = (a[ROLL]+256)>>9;
      global_conf.accZero[PITCH] = (a[PITCH]+256)>>9;
      global_conf.accZero[YAW]   = ((a[YAW]+256)>>9)-ACC_1G; // for nunchuk 200=1G
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
      //writeGlobalSet(1); // write accZero in EEPROM
    }
    calibratingA--;
  }

  imu.accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  imu.accADC[PITCH] -=  global_conf.accZero[PITCH];
  imu.accADC[YAW]   -=  global_conf.accZero[YAW] ;




}
