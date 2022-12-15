/*
   PD 값을 산출하여 각도변화값에 대한 PD보상기를 구현해본다.
*/
#define RADIAN_TO_DEGREE    180/3.14
#include <Wire.h>
#include <stdio.h>
const int MPU_addr=0x68;    //I2C address of the MPU-6050
int16_t AcX,AcY,AcZ;
double AccangleX,AccangleY;
double Kp=-1,Ki=0,Kd=-10;



class PID{
    public:
        double compute_PID(double errValue,long unit_time);
        double transferFunction(long unit_time,double impulse_signal);


};

double PID::transferFunction(long unit_time,double impulse_signal){
    double unit_time_to_sec=unit_time;

    return 5.1*(-0.87*exp(-0.76*unit_time_to_sec)-(0.873)*exp(-0.76*unit_time_to_sec)+(impulse_signal)
    -0.002*exp(-0.22*unit_time_to_sec));   //invert Laplace transfer , only real component
}

double PID::compute_PID(double errValue,long unit_time){
    double P=Kp*errValue;   //compute P control component [rad]
    double I=Ki*errValue*unit_time;     //compute I control component [rad/microsec]  meaning raditional velocity
    double D=Kd*errValue/(unit_time*1000*1000);     //compute D control component [rad/microsec]  meaning raditional accelation
    double impulse_signal=D*errValue;
    double PID_component=P+I+D;
    double vlaue_of_transferFunction_in_time=transferFunction(unit_time,impulse_signal);
    Serial.print("|transferFunction|: ");Serial.print(transferFunction(unit_time,impulse_signal));Serial.println(" ");
    return PID_component*vlaue_of_transferFunction_in_time;
}


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
    PID PID_Object;
    long previous,now,unit_time;
    double PD_errValue;
    //각도를 계산하는 동안의 시간 구하기[rad/microsec]
    previous=micros();
    getdata();
    computeangle();

    now=micros();   
    unit_time=now-previous;
    PD_errValue=PID_Object.compute_PID(AccangleX,unit_time);
    

    Serial.print("unit time : ");Serial.print(unit_time);Serial.print("us"); Serial.println("");
    Serial.print("AccX angle : ");Serial.print(AccangleX);Serial.print("rad");
    Serial.print("  AccY angle : ");Serial.print(AccangleY);Serial.print("rad"); Serial.println("");
   Serial.print("apply PID Err vlaue : ");Serial.print(PD_errValue,9); Serial.print("voltage");Serial.println(" ");
    delay(1000);


   
}
