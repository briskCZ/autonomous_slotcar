#include <SPI.h>
#include <SD.h>
#include <Arduino_LSM9DS1.h>

// Motor driver
#define STBY 6
#define IN1 7
#define IN2 8
#define PWM 9

// Accelerometer calibration values
#define N_AVG_SAMPLES 16
#define N_CAL_SAMPLES 100

// Algorithm values
#define STRAIGTH_SPEED 100
#define CORNER_SLOW_SPEED 60
#define CORNER_FAST_SPEED 80

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

        AccData getData(){
            AccData data;
            data.x = x;
            data.y = y;

            return data;
        }

        
};

// Variables
int state = 0;

Motor motor;
SDcard sd;
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

        acc.loop();

        if(acc.new_data){
            AccData data = acc.getData();
            acc.new_data = false;

            if(abs(data.x) >= 0 && abs(data.x) <= 200){
                motor.drive(STRAIGTH_SPEED);
            }else if(abs(data.x) > 500 && abs(data.x) <= 1000){
                motor.drive(CORNER_FAST_SPEED);
            }else{
                motor.drive(CORNER_FAST_SPEED);
            }
        }

    }

}
