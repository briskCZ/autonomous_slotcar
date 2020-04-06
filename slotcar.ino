#include <SPI.h>
#include <SD.h>
#include <Arduino_LSM9DS1.h>

// Motor driver
#define STBY 6
#define IN1 7
#define IN2 8
#define PWM 9

// Hall sensor
#define HALL A0
#define TRESHOLD_HIGH 500
#define TRESHOLD_LOW 180

// Accelerometer calibration values
#define N_AVG_SAMPLES 16
#define N_CAL_SAMPLES 100

// Algorithm values
#define CONSTANT_SPEED 75

struct Tuple {
    int x;
    int y;
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

        bool new_data = false;

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

                // Subtracting the calibrated value
                x -= x_cal;
                y -= y_cal;

                // Averaging
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

                // Tresholding
                if(abs(x) < 100){
                    x = 0;
                }
                if(abs(y) < 100){
                    y = 0;
                }

                new_data = true;
            }
        }

        int getX(){
            return x;
        }

        int getY(){
            return y;
        }

        Tuple getData(){
            Tuple data;
            data.x = x;
            data.y = y;

            return data;
        }

        
};

class Hall {
    private:
        int value;
        bool can_count = true;
        int rotations = 0;
        
        // All lenghts are in cm
        float base_circumference = 7.2;
        float tire_circumference_error = 0.0;
        float traveled_distance = 0;
        float tire_circumference = base_circumference + tire_circumference_error;
    
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

    void setRotations(int number){
        rotations = number;
    }

    int getTraveledDistance(){
        return traveled_distance;
    }

    int countDistance(int turns){
        return turns * tire_circumference;
    }

    int getValue(){
        return value;
    }

};

// Variables
int state = 0;

Motor motor;
Hall hall;
Accelerometer acc;

void setup() {

    // Dont run on table -_-
    if (analogRead(HALL) > 500){
        state = -1;
    }

    motor.setup();

    acc.setup();
    acc.calibrate();
}

void loop() {

    if(state == 0){

        motor.drive(CONSTANT_SPEED);

    }

}
