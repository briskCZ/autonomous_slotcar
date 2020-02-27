#include <SPI.h>
#include <SD.h>
#include <Arduino_LSM9DS1.h>

// Motor driver
#define STBY 6
#define IN1 7
#define IN2 8
#define PWM 9

// LED
#define LED_1 2
#define LED_2 3
#define LED_3 4
#define LED_4 5

// Hall sensor
#define HALL A0
#define TRESHOLD_HIGH 500
#define TRESHOLD_LOW 180

#define TRACK_LENGTH 425

// SD card
#define CS 10

#define N_AVG_SAMPLES 8
#define N_CAL_SAMPLES 100

#define DRIVEDATA_LENGTH 100

// TODO: class Led ....

struct DriveData {
  int y_acc;
  int rotations;
};

class SDcard : SDClass{
    private:
        bool initialized = false;
        File file;

    public:
        void setup(){
            if(begin(CS)){
                initialized = true;
            }
        }

        void writeOnce(String string){
            if(initialized){
                file = open("log_file.txt", FILE_WRITE);
                file.print(string);
                file.close();                
            }
        }

        bool isInicialized(){
            return initialized;
        }

        void write(String string){
            file.print(string);
        }

        void openFile(){
            file = open("log_file.txt", FILE_WRITE);
        }
        void closeFile(){
            file.close();
        }

        void writeDriveData(DriveData *arr){
            if(isInicialized()){
                openFile();

                for(int i = 0; i < DRIVEDATA_LENGTH; i++){
                    write( String(arr[i].y_acc) + "\t" + String(arr[i].rotations) + "\n");
                }


                closeFile();
            }
        }
};

class Motor {
    private:
        int current_speed;

    public:

        void setup(){
            pinMode(IN1, OUTPUT);
            pinMode(IN2, OUTPUT);
            pinMode(PWM, OUTPUT);
            pinMode(STBY, OUTPUT);
        }

        void drive(int speed){
            if(speed != current_speed){
                digitalWrite(STBY, HIGH);
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, LOW);
                analogWrite(PWM, speed);
            }
            current_speed = speed;
        }

        void brake(){
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, HIGH);
            analogWrite(PWM,0);
            current_speed = 0;
        }

        void turnOff(){
            analogWrite(PWM,0);
            digitalWrite(STBY, LOW);
        }

};

class Hall {
    private:
        int value;
        bool can_count = true;
        int rotations = 0;
        const float tire_circumference = 7.4;   // All lenghts are in cm
        int traveled_distance = 0;
    
    public:
    void loop(){
        value = analogRead(HALL);

        if(value > TRESHOLD_HIGH && can_count){
            rotations ++;
            traveled_distance = countDistance(rotations);
            can_count = false;
        }

        if(value < TRESHOLD_LOW){
            can_count = true;
        }
    }

    int getRotations(){
        return rotations;
    }

    int getTraveledDistance(){
        return traveled_distance;
    }

    int countDistance(int turns){
        return turns * tire_circumference;
    }

};

class Accelerometer {
    private:
    int x,y;

    int x_avg_arr[N_AVG_SAMPLES];
    int y_avg_arr[N_AVG_SAMPLES];
    int avg_arr_p;

    int x_cal;
    int y_cal;
    int cal_counter;


    public:
        bool setup(){
            return IMU.begin();
        }

        void calibrate(){
            long x_sum, y_sum;

            while(cal_counter < N_CAL_SAMPLES){
                if (IMU.accelerationAvailable()) {
                    IMU.readAcceleration(x, y);
                    x_sum += x;
                    y_sum += y;
                    cal_counter++;
                }
          }

        x_cal = x_sum / N_CAL_SAMPLES;
        y_cal = y_sum / N_CAL_SAMPLES;
          
        }

        void loop(){
            if (IMU.accelerationAvailable()) {
                IMU.readAcceleration(x, y);

                x -= x_cal;
                y -= y_cal;

                x_avg_arr[avg_arr_p] = x;
                y_avg_arr[avg_arr_p] = y;
                avg_arr_p ++;

                if(avg_arr_p >= N_AVG_SAMPLES){
                    avg_arr_p = 0;
                }

                long x_avg_sum = 0;
                long y_avg_sum = 0;

                for(int i = 0; i < N_AVG_SAMPLES; i++){
                    x_avg_sum += x_avg_arr[i];
                    y_avg_sum += y_avg_arr[i];
                }

                x = x_avg_sum / N_AVG_SAMPLES;
                y = y_avg_sum / N_AVG_SAMPLES;
            }
        }

        int getX(){
            return x;
        }

        int getY(){
            return y;
        }
};

// Variables
int state = 2;  // 0 - first slow lap, 1 = fast driving, 2 = test state

DriveData dd_arr[DRIVEDATA_LENGTH];

Motor motor;
SDcard sd;
Hall hall;
Accelerometer acc;

void setup() {
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
    pinMode(LED_4, OUTPUT);

    motor.setup();
    sd.setup();
    sd.writeOnce("================================ NEW LOG ================================\n\n");

    acc.setup();
    acc.calibrate();

    Serial.begin(9600);
}

void loop() {

    hall.loop();

    if(hall.getTraveledDistance() > TRACK_LENGTH){
        state = 1;
    }

    if(state == 0){
        // motor.drive(45);

        acc.loop();

        Serial.print(acc.getX());
        Serial.print(',');
        Serial.print(acc.getY());
        Serial.print('\n');

    }

    if(state == 1){
        motor.brake();
    }

    if(state == 2){ // Test state
        dd_arr[0].rotations = 5;
        dd_arr[0].y_acc = 88;
        dd_arr[5].rotations = 42;
        dd_arr[88].y_acc = 69;
        sd.writeDriveData(dd_arr);
        state = 3;
    }
}
