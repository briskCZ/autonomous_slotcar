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

#define TRACK_LENGTH 454

// vnejsi oval - 425
// vnitrni oval - 364

// vnejsi dlouhy oval - 563
// vnitrni dlouhy oval - 502

// vnejsi layout long - 622
// vnitrni layout long - 560

// vnejsi doma layout 1 - 516
// vnitrni doma layout 1  - 454

// Algorithm values
#define TEMP_COEFF 0 //TODO: Pocitat s teplotou??
#define STRAIGHT_SPEED 100
#define SLOW_CORNER_SPEED 60
#define FAST_CORNER_SPEED 70
#define CORNER_EXIT_SPEED 65


// SD card
#define CS 10

#define N_AVG_SAMPLES 16
#define N_CAL_SAMPLES 100

#define DRIVEDATA_LENGTH 888

// TODO: class Led ....

struct DriveData {
int side_acc;
int rotations;
};

struct Tuple {
    int x;
    int y;
};

enum TrackSectionType { STRAIGHT, BRAKING, CORNER, CORNEREXIT, NONE };

struct TrackSection{
        int start_position = -1;
        int end_position = -1;
        int severity = -1;
        TrackSectionType type = NONE;
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
        float tire_circumference = 7.2;   // All lenghts are in cm
        float traveled_distance = 0;
    
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

long corner_avg = 0;
int corner_samples = 0;

TrackSection track[100];
int track_p = 0;

int lap_count = 0;
int last_lap_added = 0;

bool error_coeff_added = false;

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
    
    if(state != -1){
        sd.writeOnce("================================ NEW LOG ================================\n\n");
    }

    acc.setup();
    acc.calibrate();

}

void loop() {

    hall.loop();
    acc.loop();

    if(hall.getTraveledDistance() >= TRACK_LENGTH && state == 0){
        state = 1;
        track_p = 0;
        hall.setRotations(0);
    }

    if(state == 0){

        // Drive with constant speed
        motor.drive(60);

        if(acc.new_data){
            // Get new data and write them to DriveData array
            Tuple data = acc.getData();
            acc.new_data = false;
            int current_position = hall.getRotations();

            // If a corner is detected
            if(abs(data.x) >= 1000){
                if(track[track_p].type == NONE){    // And track array at current position is empty
                    
                    // Create straight if it should be longer than 2
                    if(track_p - 1 >= 0){
                        if(current_position - track[track_p - 1].end_position  > 2){
                            
                            track[track_p].type = STRAIGHT;
                            track[track_p].start_position = track[track_p - 1].end_position + 1 ;
                            track[track_p].end_position = current_position;
                            
                            track_p ++;
                        }
                    }else if(track_p == 0){ // Create begining straight
                            
                        track[track_p].type = STRAIGHT;
                        track[track_p].start_position = 0 ;
                        track[track_p].end_position = current_position;
                        
                        track_p ++;
                    }

                    // Create braking zone if previous straight was long enough on the last rotation of straight
                    if(track_p - 1 >= 0 && track[track_p - 1].type == STRAIGHT && track[track_p - 1].end_position - track[track_p - 1].start_position > 5){
                        track[track_p].type = BRAKING;
                        track[track_p].start_position = current_position - 1;
                        track[track_p - 1].end_position -= 2;
                        track[track_p].end_position = current_position; 
                        track[track_p].severity = track[track_p - 1].end_position - track[track_p - 1].start_position;

                        track_p ++;
                    }


                    // Create corner
                    track[track_p].type = CORNER;
                    track[track_p].start_position = track[track_p - 1].end_position + 1;

                }

                corner_avg += abs(data.x);
                corner_samples ++;
            }

            // If acceleration drops and corner ends
            if(abs(data.x) <= 500){
                if(track[track_p].start_position != -1 && track[track_p].end_position == -1){
                    
                    // Fill remaing corner info
                    track[track_p].end_position = current_position;
                    corner_avg /= corner_samples;
                    track[track_p].severity = corner_avg;

                    

                    // Create corner exit if corner was long enough??
                    if( track[track_p].type == CORNER && track[track_p].end_position - track[track_p].start_position > 5){

                        track_p++;

                        track[track_p].type = CORNEREXIT;
                        track[track_p].start_position = current_position + 1;
                        track[track_p].end_position = current_position + 2;
                    }

                    //  Update start position of the first straight so it covers the whole track between last and first corners
                    if( track[0].type == STRAIGHT){
                        track[0].start_position = track[track_p].end_position + 1;
                    }
                    
                    corner_avg = 0;
                    corner_samples = 0;
                    track_p++;
                }
            }
        }
    }

    if(state == 1){
        hall.loop();
        
        TrackSection current_section = track[track_p];

        switch(current_section.type){
            case STRAIGHT:
                motor.drive(STRAIGHT_SPEED);
                break;
            
            case BRAKING:
                motor.brake();
                break;

            case CORNER:
                if(current_section.severity < 3000){
                    motor.drive(FAST_CORNER_SPEED);
                }else{
                    motor.drive(SLOW_CORNER_SPEED);
                }
                
                break;

            case CORNEREXIT:
                motor.drive(CORNER_EXIT_SPEED);

                // Subtracting rotations because of error when counting rotations while driving
                if(lap_count > 3 && !error_coeff_added){
                    error_coeff_added = true;
                    if(lap_count % 8 == 0){
                        hall.setRotations(hall.getRotations() - 3);
                    }else{
                        hall.setRotations(hall.getRotations() - 2);
                    }
                    
                }
                
                break;

            default:
                motor.drive(40);
                break;

        }

        if(hall.getRotations() == current_section.end_position){
            track_p ++;
            if(track[track_p].type == NONE){
                track_p = 0;
            }
        }

        if(hall.getTraveledDistance() >= TRACK_LENGTH){
            hall.setRotations(0);
            
            
            if(millis() - last_lap_added > 1000){
                error_coeff_added = false;
                lap_count ++;
                last_lap_added = millis();
            }
        }


    }

    if(state == 2){
        motor.brake();
        for (int i = 0; i < 100; i++){

            sd.writeOnce(String(track[i].start_position) + "\t" + String(track[i].end_position) + "\t" + String(track[i].severity) + "\t" + String(track[i].type) + "\n");

        }
        while (42);

    }
}
