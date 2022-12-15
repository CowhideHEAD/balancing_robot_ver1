#define IN1 2
#define IN2 3
#define ENA 10

/*
    IN1,IN2 (H,L) forword  (L,H) backword
    IN3,IN4 (H,L) forword  (L,H) bakcword
*/
unsigned long previous,now,unit_time;

void setup(){
    Serial.begin(115200);
    Serial.println("Serial begin...");
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(ENA,OUTPUT);
    Serial.println("pinmode setting succees...");
    Serial.println("system all green");
}

void loop(){
    previous=micros();
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(ENA,255);
    now=micros();
    unit_time=now-previous;
    Serial.print("unit time : "); Serial.print(unit_time); Serial.println("us");

}