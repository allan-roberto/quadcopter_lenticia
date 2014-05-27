/*
 * magnetometer.c
 *
 *  Created on: May 26, 2014
 *      Author: tpv
 */

// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************

#include <magnetometer.h>
#include <uart.h>
#include <i2c.h>
#include <math.h>


static float   magGain[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static uint8_t magInit = 0;
flags_struct_t f;
extern uint8_t rawADC[6];
extern uint32_t currentTime;
extern conf_t conf;
extern imu_t imu;
extern global_conf_t global_conf;


void init_magnetometer() {
  int32_t xyz_total[3]={0,0,0};  // 32 bit totals so they won't overflow.
  bool bret=true;                // Error indicator

  _delay_ms(50);  //Wait before start
  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias

  // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
  // The new gain setting is effective from the second measurement and on.

  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
  i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
  _delay_ms(100);
  getADC();  //Get one sample, and discard it

  for (uint8_t i=0; i<10; i++) { //Collect 10 samples
    i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
    _delay_ms(100);
    getADC();   // Get the raw values in case the scales have already been changed.

    // Since the measurements are noisy, they should be averaged rather than taking the max.
    xyz_total[0]+=imu.magADC[0];
    xyz_total[1]+=imu.magADC[1];
    xyz_total[2]+=imu.magADC[2];

    // Detect saturation.
    if (-(1<<12) >= min(imu.magADC[0],min(imu.magADC[1],imu.magADC[2]))) {
      bret=false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  // Apply the negative bias. (Same gain)
  i2c_writeReg(MAG_ADDRESS,HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
  for (uint8_t i=0; i<10; i++) {
    i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
    _delay_ms(100);
    getADC();  // Get the raw values in case the scales have already been changed.

    // Since the measurements are noisy, they should be averaged.
    xyz_total[0]-=imu.magADC[0];
    xyz_total[1]-=imu.magADC[1];
    xyz_total[2]-=imu.magADC[2];

    // Detect saturation.
    if (-(1<<12) >= min(imu.magADC[0],min(imu.magADC[1],imu.magADC[2]))) {
      bret=false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  magGain[0]=fabs(820.0*HMC58X3_X_SELF_TEST_GAUSS*2.0*10.0/xyz_total[0]);
  magGain[1]=fabs(820.0*HMC58X3_Y_SELF_TEST_GAUSS*2.0*10.0/xyz_total[1]);
  magGain[2]=fabs(820.0*HMC58X3_Z_SELF_TEST_GAUSS*2.0*10.0/xyz_total[2]);

  // leave test mode
  i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_CONFA ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
  i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_CONFB ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
  i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_MODE  ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
  _delay_ms(100);
  magInit = 1;

  if (!bret) { //Something went wrong so get a best guess
    magGain[0] = 1.0;
    magGain[1] = 1.0;
    magGain[2] = 1.0;
  }
  f.CALIBRATE_MAG = 1;
} //  Mag_init().




// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************
//#if MAG

uint8_t magnetometer_get_data(void) { // return 1 when news values are available, 0 otherwise
  static uint32_t t = 100,tCal = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;
  //if ( currentTime < t ) return 0; //each read is spaced by 100ms
  t = currentTime + 100000;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  Device_Mag_getADC();
  imu.magADC[ROLL]  = imu.magADC[ROLL]  * magGain[ROLL];
  imu.magADC[PITCH] = imu.magADC[PITCH] * magGain[PITCH];
  imu.magADC[YAW]   = imu.magADC[YAW]   * magGain[YAW];
  if (f.CALIBRATE_MAG) {
    tCal = t;
    for(axis=0;axis<3;axis++) {
      global_conf.magZero[axis] = 0;
      magZeroTempMin[axis] = imu.magADC[axis];
      magZeroTempMax[axis] = imu.magADC[axis];
    }
    f.CALIBRATE_MAG = 0;
  }
  if (magInit) { // we apply offset only once mag calibration is done
    imu.magADC[ROLL]  -= global_conf.magZero[ROLL];
    imu.magADC[PITCH] -= global_conf.magZero[PITCH];
    imu.magADC[YAW]   -= global_conf.magZero[YAW];
  }

  if (tCal != 0) {
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      LEDPIN_TOGGLE;
      for(axis=0;axis<3;axis++) {
        if (imu.magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = imu.magADC[axis];
        if (imu.magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = imu.magADC[axis];
      }
    } else {
      tCal = 0;
      for(axis=0;axis<3;axis++)
        global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])>>1;
      //writeGlobalSet(1);
    }
  } else {
    #if defined(SENSORS_TILT_45DEG_LEFT)
      int16_t temp = ((imu.magADC[PITCH] - imu.magADC[ROLL] )*7)/10;
      imu.magADC[ROLL] = ((imu.magADC[ROLL]  + imu.magADC[PITCH])*7)/10;
      imu.magADC[PITCH] = temp;
    #endif
    #if defined(SENSORS_TILT_45DEG_RIGHT)
      int16_t temp = ((imu.magADC[PITCH] + imu.magADC[ROLL] )*7)/10;
      imu.magADC[ROLL] = ((imu.magADC[ROLL]  - imu.magADC[PITCH])*7)/10;
      imu.magADC[PITCH] = temp;
    #endif
  }
  return 1;
}

void getADC() {
  i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);


    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[4]<<8) | rawADC[5]) ,
                     ((rawADC[2]<<8) | rawADC[3]) );

}

void Device_Mag_getADC() {
  getADC();
}


void magnetometer_get_raw_data(int16_t *X, int16_t *Y, int16_t *Z ) {
  TWBR = ((F_CPU / 400000L) - 16) / 2;  // Optional line.  Sensor is good for it in the spec.
  i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
  *X = ((rawADC[0]<<8) | rawADC[1]);
  *Y = ((rawADC[4]<<8) | rawADC[5]) ;
  *Z = ((rawADC[4]<<8) | rawADC[5]) ;

}


