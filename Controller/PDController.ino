/*
    PID controller 구현해보기.
*/

#define  pin5    5
double Kp,Ki,Kd=5;
double Ct;

void impuls(){

}

void PID(){
    
}

void setup(){
    Serial.begin(115200);
    pinMode(pin5,OUTPUT);

}

void loop(){    
    long previous,now;
    previous=micros();
    now=micros();
    Serial.println(now-previous);


}