#include "Arduino.h"

/********************
#include "Wire.h"

const int MPU_addr=0x69; // I2C address of the MPU-6050

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup (){
    Wire.begin();
    Serial.begin(115200);

    check_I2c(MPU_addr); // Check that there is an MPU

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
}

void loop (){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    Serial.print("AcX = "); Serial.print(AcX);
    Serial.print(" | AcY = "); Serial.print(AcY);
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    Serial.print(" | Tmp = "); Serial.print(Tmp);
    Serial.print(" | GyX = "); Serial.print(GyX);
    Serial.print(" | GyY = "); Serial.print(GyY);
    Serial.print(" | GyZ = "); Serial.println(GyZ);

    delay(500); // Wait 0.5 seconds and scan again
}

byte check_I2c(byte addr){
    // We are using the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    byte error;
    Wire.beginTransmission(addr);
    error = Wire.endTransmission();

    if (error == 0){
        Serial.print(" Device Found at 0x");
        Serial.println(addr,HEX);
    }

    else{
        Serial.print(" No Device Found at 0x");
        Serial.println(addr,HEX);
    }

    return error;
}
********************/
/*******************
#include "Wire.h"
#include "Arduino.h"

const uint8_t MPU_addr=0x68; // I2C address of the MPU-6050

const float MPU_GYRO_250_SCALE = 131.0;
const float MPU_GYRO_500_SCALE = 65.5;
const float MPU_GYRO_1000_SCALE = 32.8;
const float MPU_GYRO_2000_SCALE = 16.4;
const float MPU_ACCL_2_SCALE = 16384.0;
const float MPU_ACCL_4_SCALE = 8192.0;
const float MPU_ACCL_8_SCALE = 4096.0;
const float MPU_ACCL_16_SCALE = 2048.0;



struct rawdata {
    int16_t AcX;
    int16_t AcY;
    int16_t AcZ;
    int16_t Tmp;
    int16_t GyX;
    int16_t GyY;
    int16_t GyZ;
};

struct scaleddata{
    float AcX;
    float AcY;
    float AcZ;
    float Tmp;
    float GyX;
    float GyY;
    float GyZ;
};

bool checkI2c(byte addr);
void mpu6050Begin(byte addr);
rawdata mpu6050Read(byte addr, bool Debug);
void setMPU6050scales(byte addr,uint8_t Gyro,uint8_t Accl);
void getMPU6050scales(byte addr,uint8_t &Gyro,uint8_t &Accl);
scaleddata convertRawToScaled(byte addr, rawdata data_in,bool Debug);

void setup() {
    Wire.begin();
    Serial.begin(115200);

    mpu6050Begin(MPU_addr);
}

void loop() {
    Serial.println("______________________________________________");
    rawdata next_sample;
    setMPU6050scales(MPU_addr,0b00000000,0b00010000);
    next_sample = mpu6050Read(MPU_addr, true);
    convertRawToScaled(MPU_addr, next_sample,true);

    delay(500); // Wait 5 seconds and scan again
}



void mpu6050Begin(byte addr){
    // This function initializes the MPU-6050 IMU Sensor
    // It verifys the address is correct and wakes up the
    // MPU.
    if (checkI2c(addr)){
     Wire.beginTransmission(MPU_addr);
        Wire.write(0x6B); // PWR_MGMT_1 register
        Wire.write(0); // set to zero (wakes up the MPU-6050)
        Wire.endTransmission(true);

        delay(30); // Ensure gyro has enough time to power up
    }
}

bool checkI2c(byte addr){
    // We are using the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Serial.println(" ");
    Wire.beginTransmission(addr);

    if (Wire.endTransmission() == 0){
        Serial.print(" Device Found at 0x");
        Serial.println(addr,HEX);
        return true;
    }
    else{
        Serial.print(" No Device Found at 0x");
        Serial.println(addr,HEX);
        return false;
    }
}



rawdata mpu6050Read(byte addr, bool Debug){
    // This function reads the raw 16-bit data values from
    // the MPU-6050

    rawdata values;

    Wire.beginTransmission(addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(addr,14,true); // request a total of 14 registers
    values.AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    values.AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    values.AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    values.Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    values.GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    values.GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    values.GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)


    if(Debug){
        Serial.print(" GyX = "); Serial.print(values.GyX);
        Serial.print(" | GyY = "); Serial.print(values.GyY);
        Serial.print(" | GyZ = "); Serial.print(values.GyZ);
        Serial.print(" | Tmp = "); Serial.print(values.Tmp);
        Serial.print(" | AcX = "); Serial.print(values.AcX);
        Serial.print(" | AcY = "); Serial.print(values.AcY);
        Serial.print(" | AcZ = "); Serial.println(values.AcZ);
    }

    return values;
}

void setMPU6050scales(byte addr,uint8_t Gyro,uint8_t Accl){
    Wire.beginTransmission(addr);
    Wire.write(0x1B); // write to register starting at 0x1B
    Wire.write(Gyro); // Self Tests Off and set Gyro FS to 250
    Wire.write(Accl); // Self Tests Off and set Accl FS to 8g
    Wire.endTransmission(true);
}

void getMPU6050scales(byte addr,uint8_t &Gyro,uint8_t &Accl){
    Wire.beginTransmission(addr);
    Wire.write(0x1B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(addr,2,true); // request a total of 14 registers
    Gyro = (Wire.read()&(bit(3)|bit(4)))>>3;
    Accl = (Wire.read()&(bit(3)|bit(4)))>>3;
}



scaleddata convertRawToScaled(byte addr, rawdata data_in, bool Debug){

scaleddata values;
float scale_value = 0.0;
byte Gyro, Accl;

getMPU6050scales(MPU_addr, Gyro, Accl);

if(Debug){
Serial.print("Gyro Full-Scale = ");
}

    switch (Gyro){
    case 0:
    scale_value = MPU_GYRO_250_SCALE;
    if(Debug){
        Serial.println("±250 °/s");
    }
    break;
    case 1:
    scale_value = MPU_GYRO_500_SCALE;
    if(Debug){
        Serial.println("±500 °/s");
    }
    break;
    case 2:
    scale_value = MPU_GYRO_1000_SCALE;
    if(Debug){
        Serial.println("±1000 °/s");
    }
    break;
    case 3:
    scale_value = MPU_GYRO_2000_SCALE;
    if(Debug){
        Serial.println("±2000 °/s");
    }
    break;
    default:
    break;
    }

    values.GyX = (float) data_in.GyX / scale_value;
    values.GyY = (float) data_in.GyY / scale_value;
    values.GyZ = (float) data_in.GyZ / scale_value;

    scale_value = 0.0;
    if(Debug){
        Serial.print("Accl Full-Scale = ");
    }
    switch (Accl){
    case 0:
    scale_value = MPU_ACCL_2_SCALE;
    if(Debug){
        Serial.println("±2 g");
    }
    break;
    case 1:
    scale_value = MPU_ACCL_4_SCALE;
    if(Debug){
        Serial.println("±4 g");
    }
    break;
    case 2:
    scale_value = MPU_ACCL_8_SCALE;
    if(Debug){
        Serial.println("±8 g");
    }
    break;
    case 3:
    scale_value = MPU_ACCL_16_SCALE;
    if(Debug){
        Serial.println("±16 g");
    }
    break;
    default:
    break;
    }

    values.AcX = (float) data_in.AcX / scale_value;
    values.AcY = (float) data_in.AcY / scale_value;
    values.AcZ = (float) data_in.AcZ / scale_value;


    values.Tmp = (float) data_in.Tmp / 340.0 + 36.53;

    if(Debug){
        Serial.print(" GyX = "); Serial.print(values.GyX);
        Serial.print(" °/s| GyY = "); Serial.print(values.GyY);
        Serial.print(" °/s| GyZ = "); Serial.print(values.GyZ);
        Serial.print(" °/s| Tmp = "); Serial.print(values.Tmp);
        Serial.print(" °C| AcX = "); Serial.print(values.AcX);
        Serial.print(" g| AcY = "); Serial.print(values.AcY);
        Serial.print(" g| AcZ = "); Serial.print(values.AcZ);Serial.println(" g");
    }

    return values;
}
************/

/*************************
#include<Wire.h>
const int MPU=0x69;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);
}
void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();

  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(AcX);
  Serial.print(" | Y = "); Serial.print(AcY);
  Serial.print(" | Z = "); Serial.println(AcZ);

  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);
  Serial.println(" ");
  delay(333);
}

*************************/
/************************
#include<Wire.h>
#include <math.h>
const int MPU=0x69;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
double pitch,roll;

void setup(){
Wire.begin();
Wire.beginTransmission(MPU);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);
}
void loop(){
Wire.beginTransmission(MPU);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU,14,true);

int AcXoff,AcYoff,AcZoff,GyXoff,GyYoff,GyZoff;
int temp,toff;
double t,tx,tf;

//Acceleration data correction
AcXoff = -950;
AcYoff = -300;
AcZoff = 0;

//Temperature correction
toff = -1600;

//Gyro correction
GyXoff = 480;
GyYoff = 170;
GyZoff = 210;

//read accel data
AcX=(Wire.read()<<8|Wire.read()) + AcXoff;
AcY=(Wire.read()<<8|Wire.read()) + AcYoff;
AcZ=(Wire.read()<<8|Wire.read()) + AcYoff;

//read temperature data
temp=(Wire.read()<<8|Wire.read()) + toff;
tx=temp;
t = tx/340 + 36.53;
tf = (t * 9/5) + 32;

//read gyro data
GyX=(Wire.read()<<8|Wire.read()) + GyXoff;
GyY=(Wire.read()<<8|Wire.read()) + GyYoff;
GyZ=(Wire.read()<<8|Wire.read()) + GyZoff;

//get pitch/roll
getAngle(AcX,AcY,AcZ);

//send the data out the serial port
Serial.print("Angle: ");
Serial.print("Pitch = "); Serial.print(pitch);
Serial.print(" | Roll = "); Serial.println(roll);

Serial.print("Temp: ");
Serial.print("Temp(F) = "); Serial.print(tf);
Serial.print(" | Temp(C) = "); Serial.println(t);

Serial.print("Accelerometer: ");
Serial.print("X = "); Serial.print(AcX);
Serial.print(" | Y = "); Serial.print(AcY);
Serial.print(" | Z = "); Serial.println(AcZ);

Serial.print("Gyroscope: ");
Serial.print("X = "); Serial.print(GyX);
Serial.print(" | Y = "); Serial.print(GyY);
Serial.print(" | Z = "); Serial.println(GyZ);
Serial.println(" ");
delay(333);
}

//convert the accel data to pitch/roll
void getAngle(int Vx,int Vy,int Vz) {
double x = Vx;
double y = Vy;
double z = Vz;
}

**********************/

#include "Wire.h" // This library allows you to communicate with I2C devices.
const int MPU_ADDR = 0x69; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data

int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data
char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}
void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

  // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.println(convert_int16_to_str(accelerometer_z));

  // delay
  delay(500);
}
