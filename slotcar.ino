
// Pin definitions
#define IN1 2
#define IN2 3
#define PWM 4

#define LR_LED 9

//TODO: define HALL, LED and SD card pins

void drive(int speed){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM, speed);
}

void brake(){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN1, HIGH);
    analogWrite(PWM,0);
}

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(PWM, OUTPUT);

    pinMode(LR_LED, OUTPUT);

}

void loop() {
    digitalWrite(LR_LED, HIGH);
    drive(100);
    delay(500);

    drive(255);
    delay(500);

    drive(0);
    delay(500);

    brake();
    digitalWrite(LR_LED, LOW);
    delay(500);

}
