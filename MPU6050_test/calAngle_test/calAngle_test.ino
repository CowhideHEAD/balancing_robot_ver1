/*
    오일러공식을 이용하여 자이로센서의 acclation 값으로 현재위치의 변화한 각도를 산출해낸다.
*/
#define RADIAN_TO_DEGREE    180/3.14
#include <Wire.h>
#include <stdio.h>
const int MPU_addr=0x68;    //I2C address of the MPU-6050
int16_t AcX,AcY,AcZ;
double AccangleX,AccangleY;

void initial(){     //setting MPU6050 initial state for I2C communication 
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
    
}
void getdata(){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);   //start from 0x3B resgister data read/wirte 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,6,true); //request 14 register form 0x3B

    AcX=Wire.read()<<8|Wire.read(); //read data register 0x3B ,0x3C   AcX=[0x3Bdata 0x3Cdata]
    AcY=Wire.read()<<8|Wire.read(); //read data register 0x3D,0x3E AcY=[0x3Ddata 0x3Edata]
    AcZ=Wire.read()<<8|Wire.read(); //read data register 0x3F,0x40 AcZ=[0x3Fdata 0x40data]
}

void computeangle(){
    AccangleX=atan(-AcX/(sqrt(pow(AcY,2)+pow(AcZ,2))))*RADIAN_TO_DEGREE;//   Xangle return [-90,90]
    AccangleY=atan(AcY/(sqrt(pow(AcX,2)+pow(AcZ,2))))*RADIAN_TO_DEGREE;  //  Yangle return [-90,90]

}

void setup(){
    Serial.begin(115200);
    Serial.println("Serial begin... boud rate 115200");
    initial();
    Serial.println("I2C Initialize has succ...");


}

void loop(){

    getdata();
    computeangle();
    
    Serial.print("  AccAngle X : "); Serial.print(AccangleX);
    Serial.print("  AccAngle Y : "); Serial.print(AccangleY);
    Serial.println("");
}
