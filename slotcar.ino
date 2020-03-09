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

#define DRIVEDATA_LENGTH 888

// Algorithm constants
#define BRAKE_ZONE_LENGTH 1

// TODO: class Led ....

struct DriveData {
  int side_acc;
  int rotations;
};

struct Tuple {
  int x;
  int y;
};

struct Corner {
  int start_rotations = -1;
  int end_rotations = -1;
  int severity = -1;
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
                    write( String(arr[i].side_acc) + "\t" + String(arr[i].rotations) + "\n");
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

    void resetRotations(){
        rotations = 0;
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

// Variables
int state = 0;  // 0 - first slow lap, 1 = fast driving, 2 = test state


DriveData dd_arr[DRIVEDATA_LENGTH];
int dd_arr_p = 0;

Corner corners[10];
int corner_p = 0;
long corner_avg = 0;
int corner_samples = 0;
bool last_corner = false;


Motor motor;
SDcard sd;
Hall hall;
Accelerometer acc;

void setup() {

    // Dont run on table -_-
    if (analogRead(HALL) > 500){
        state = -1;
    }

    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
    pinMode(LED_4, OUTPUT);

    motor.setup();
    sd.setup();
    sd.writeOnce("================================ NEW LOG ================================\n\n");

    acc.setup();
    acc.calibrate();

}

void loop() {

    hall.loop();

    if(hall.getTraveledDistance() >= TRACK_LENGTH && state == 0){
        state = 1;
        dd_arr_p = 0;
        corner_p = 0;
        hall.resetRotations();
    }

    if(state == 0){

        // Drive with constant speed
        motor.drive(60);

        acc.loop();

        if(acc.new_data){
            // Get new data and write them to DriveData array
            Tuple data = acc.getData();
            acc.new_data = false;
            int rotations = hall.getRotations();

            dd_arr[dd_arr_p].side_acc = data.x;
            dd_arr[dd_arr_p].rotations = rotations;
            dd_arr_p ++; // TODO: ošetřit šahání mimo pole?

            // If a corner is detected: write number of rotations from start to begining of the corner
            if(abs(data.x) >= 1000){
                if(corners[corner_p].start_rotations == -1){
                    corners[corner_p].start_rotations = rotations;
                }

                corner_avg += abs(data.x);
                corner_samples ++;
            }

            // If acceleration drops and corner ends: write number of rotations from start to the end of the corner
            if(abs(data.x) <= 500){
                if(corners[corner_p].start_rotations != -1 && corners[corner_p].end_rotations == -1){
                    corners[corner_p].end_rotations = rotations;
                    
                    // Vypočítá průměr hodnot v zatáčce a určí podle nich jak je ostrá
                    corner_avg /= corner_samples;
                    if(corner_avg < 4000){
                        corners[corner_p].severity = 2;
                    }else{
                        corners[corner_p].severity = 3;
                    }
                    
                    // Připraví hodnoty na další zatáčku
                    corner_avg = 0;
                    corner_samples = 0;
                    corner_p++;
                }
            }
        }
    }

    if(state == 1){
        hall.loop();
        acc.loop();

        int until_corner_start = corners[corner_p].start_rotations - hall.getRotations();
        int until_corner_end = corners[corner_p].end_rotations - hall.getRotations();

        // Dokud jede na rovince a nedojede do brzdne zony (nebo jede za poslední zatáčkou)
        if(until_corner_start > BRAKE_ZONE_LENGTH || last_corner){
            motor.drive(100);
        }

        // v brzdne zone brzdi
        if(until_corner_start <= BRAKE_ZONE_LENGTH && until_corner_start >= 0){
            motor.brake();
        }

        // zatacku projizdi pomalej
        if(until_corner_start < 0 && until_corner_end > 0){
            motor.drive(60);
        }

        // vyjezd ze zatackyy, pokud existuje dalsi zatacka tak posunout ukazatel, jinak udělat něco aby to jelo
        if(until_corner_end == 0){
            if(corners[corner_p + 1 ].start_rotations != -1){
                corner_p ++;
            }else{
                last_corner = true;
            }
        }

        // Reset hodnot po projeti kola
        if(hall.getTraveledDistance() >= TRACK_LENGTH){
            hall.resetRotations();
            corner_p = 0;
            last_corner = false;
        }

        //TODO: pri vyjezdu ze zatack (podle akcelerometru) nastav spravnou hodnotu

    }

    if(state == 2){
        motor.brake();
        for (int i = 0; i < 10; i++){
            sd.writeOnce(String(corners[i].start_rotations) + "\t" + String(corners[i].end_rotations) + "\t" + String(corners[i].severity) + "\n");
        }
        while(42);
    }
}
