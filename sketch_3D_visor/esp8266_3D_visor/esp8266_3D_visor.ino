#include "Arduino.h"

const int sharp = A0;
        
/*******gyro***********/
 // This library allows you to communicate with I2C devices
 #include "Wire.h" 

 // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
 const int MPU_ADDR = 0x69;

 // variables for accelerometer raw data
 int16_t accelerometer_x, accelerometer_y, accelerometer_z;

 int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
 int16_t temperature; // variables for temperature data
 char tmp_str[7]; // temporary variable used in convert function

 /*  converts int16 to string. Moreover, resulting strings
  *   will have the same length in the debug monitor.
 */
 char* convert_int16_to_str(int16_t i) { 
   sprintf(tmp_str, "%6d", i);
   return tmp_str;
 }
/**********************/

void setup (){
    Serial.begin(115200);

    /******wire_conf*******/
     Wire.begin();
     Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
     Wire.write(0x6B); // PWR_MGMT_1 register
     Wire.write(0); // set to zero (wakes up the MPU-6050)
     Wire.endTransmission(true);
    /**********************/
}

void loop (){
    calc_dist ();
    calc_gyro();

    delay(500);
}

void calc_dist (){
    float R = analogRead(sharp);
    float X = R/(1023 - 0)*(3.3 - 0) + 0;
    float DIST = 60.7 * pow(X, -1.12);

    Serial.print("1"); Serial.print(",");
    Serial.print(DIST); Serial.print(";");
}

void calc_gyro (){
    Wire.beginTransmission(MPU_ADDR);

    // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
    Wire.write(0x3B); 

    // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

    //"Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    //reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accelerometer_x = Wire.read()<<8 | Wire.read(); 
    //reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    accelerometer_y = Wire.read()<<8 | Wire.read(); 
    //reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    accelerometer_z = Wire.read()<<8 | Wire.read(); 

    // print out data
    Serial.print("2"); Serial.print(",");
    Serial.print(convert_int16_to_str(accelerometer_x)); Serial.print(";");

    Serial.print("3"); Serial.print(",");
    Serial.print(convert_int16_to_str(accelerometer_y)); Serial.print(";");

    Serial.print("4"); Serial.print(",");
    Serial.print(convert_int16_to_str(accelerometer_z)); Serial.print(";");
}