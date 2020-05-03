//  Autonomous slotcar - uses Arduino Nano 33 BLE
//  Bachelor thesis
//  Author: Marek Nesvadba (xnesva06)
//  FIT Brno university of technology
//  2020


#include <SPI.h>
#include <SD.h>

// Modified version that returns integers instead of floats
#include <Arduino_LSM9DS1.h>

// Motor driver pins
#define STBY 6
#define IN1 7
#define IN2 8
#define PWM 9

// LED pins
#define LED_0 2
#define LED_1 3
#define LED_2 4
#define LED_3 5

// Hall sensor pin and values
#define HALL A0
#define TRESHOLD_HIGH 500
#define TRESHOLD_LOW 180

// Track length
#define TRACK_LENGTH 592  // in cm

// Algorithm values
#define FIRST_LAP_SPEED 60
#define STRAIGHT_SPEED 110
#define SLOW_CORNER_SPEED 90
#define MEDIUM_CORNER_SPEED 95
#define FAST_CORNER_SPEED 100
#define ULTRA_CORNER_SPEED 105
#define CORNER_EXIT_SPEED 95

#define TRACK_SECTION_ARR_LENGTH 100

#define CORNER_BEGIN 1000
#define CORNER_END 500

#define N_AVG_SAMPLES 16
#define N_CAL_SAMPLES 100

#define ERROR_CONSTANT 300

// SD card pin
#define CS 10

// TODO: class Led ....

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

class LEDs {
    public:

        void setup(){
            pinMode(LED_0, OUTPUT); // LR
            pinMode(LED_1, OUTPUT); // RR
            pinMode(LED_2, OUTPUT); // LF
            pinMode(LED_3, OUTPUT); // RF
        }

        void brake_lights(bool state){
            digitalWrite(LED_0, state);
            digitalWrite(LED_1, state);
        }

        void front_lights(bool state){
            digitalWrite(LED_2, state);
            digitalWrite(LED_3, state);
        }

        void turnOff(){
            digitalWrite(LED_0, LOW);
            digitalWrite(LED_1, LOW);
            digitalWrite(LED_2, LOW);
            digitalWrite(LED_3, LOW);
        }

        void set(int led, bool state){
            digitalWrite(led, state);
        }

};

class Hall {
    private:
        int value;
        bool can_count = true;
        int rotations = 0;
        
        // All lenghts are in cm
        float base_circumference = 7.1;
        float tire_circumference_error = -0.10;
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

class Accelerometer {
    private:
    int x;

    int x_avg_arr[N_AVG_SAMPLES];
    int avg_arr_p;

    int x_cal;
    int cal_counter;

    
    public:

        bool new_data = false;

        bool setup(){
            return IMU.begin();
        }

        void calibrate(){
            long x_sum;

            while(cal_counter < N_CAL_SAMPLES){
                if (IMU.accelerationAvailable()) {
                    IMU.readAcceleration(x);
                    x_sum += x;
                    cal_counter++;
                }
            }

            x_cal = x_sum / N_CAL_SAMPLES;
          
        }

        void loop(){
            if (IMU.accelerationAvailable()) {
                IMU.readAcceleration(x);

                // Subtracting the calibrated value
                x -= x_cal;

                // Averaging
                x_avg_arr[avg_arr_p] = x;
                avg_arr_p ++;

                if(avg_arr_p >= N_AVG_SAMPLES){
                    avg_arr_p = 0;
                }

                long x_avg_sum = 0;

                for(int i = 0; i < N_AVG_SAMPLES; i++){
                    x_avg_sum += x_avg_arr[i];
                }

                x = x_avg_sum / N_AVG_SAMPLES;

                // Tresholding
                if(abs(x) < 100){
                    x = 0;
                }

                new_data = true;
            }
        }

        int getX(){
            return x;
        }

        
};

// Variables
int state = 0;  // 0 - first slow lap, 1 = fast driving, 2 = debug state

long corner_avg = 0;
int corner_samples = 0;

TrackSection track[TRACK_SECTION_ARR_LENGTH];
int track_p = 0;

int lap_count = 0;
int last_lap_added = 0;

bool error_coeff_added = false;

Motor motor;
SDcard sd;
Hall hall;
Accelerometer acc;
LEDs lights;

void setup() {

    // Dont run on table
    if (analogRead(HALL) > 500){
        state = -1;
    }

    motor.setup();
    sd.setup();
    lights.setup();
    
    // Separator in log file
    if(state != -1){
        sd.writeOnce("================================ NEW LOG ================================\n\n");
    }

    acc.setup();
    acc.calibrate();

}

void loop() {

    hall.loop();
    acc.loop();

    // The car travelled through one lap and should switch mode
    if(hall.getTraveledDistance() >= TRACK_LENGTH && state == 0){
        state = 1;
        track_p = 0;
        hall.setRotations(0);
    }

    if(state == 0){ // Autonomous track mapping
        lights.front_lights(true);

        // Drive with constant speed
        motor.drive(FIRST_LAP_SPEED);

        if(acc.new_data){
            // Get new data
            double x = acc.getX();
            acc.new_data = false;
            int current_position = hall.getRotations();

            // If a corner is detected
            if(abs(x) >= CORNER_BEGIN){
                if(track[track_p].type == NONE){    // And track array at current position is empty
                    
                    // Create straight if it should be longer than 2
                    if(track_p - 1 >= 0){
                        if(current_position - track[track_p - 1].end_position  > 1){
                            
                            track[track_p].type = STRAIGHT;
                            track[track_p].start_position = track[track_p - 1].end_position + 1 ;
                            track[track_p].end_position = max(current_position,track[track_p].start_position);
                            
                            track_p ++;
                        }

                    }else if(track_p == 0){ // Create begining straight
                            
                        track[track_p].type = STRAIGHT;
                        track[track_p].start_position = 0 ;
                        track[track_p].end_position = current_position;
                        
                        track_p ++;
                    }

                    // Create braking zone if previous straight was long enough
                    if(track_p - 1 >= 0 && track[track_p - 1].type == STRAIGHT){

                        // Create braking zone after first straight or after long straight
                        if((track_p == 1) || (track[track_p - 1].end_position - track[track_p - 1].start_position >= 10)){
                            track[track_p].type = BRAKING;
                            track[track_p].start_position = current_position - 2;
                            track[track_p - 1].end_position -= 3;
                            track[track_p].end_position = current_position;
                            track_p ++;
                        }
                        else if(track[track_p - 1].end_position - track[track_p - 1].start_position >= 5){ // Create braking zone after short straight
                            track[track_p].type = BRAKING;
                            track[track_p].start_position = current_position - 1;
                            track[track_p - 1].end_position -= 2;
                            track[track_p].end_position = current_position; 
                            track_p ++;
                        }

                    }

                    // Create corner
                    track[track_p].type = CORNER;
                    track[track_p].start_position = track[track_p - 1].end_position + 1;

                }

                corner_avg += abs(x);
                corner_samples ++;
            }

            // If acceleration drops and corner ends
            if(abs(x) <= CORNER_END){
                if(track[track_p].start_position != -1 && track[track_p].end_position == -1){
                    
                    // Fill remaing corner info
                    track[track_p].end_position = current_position;
                    corner_avg /= corner_samples;
                    track[track_p].severity = corner_avg;

                    // Create corner exit if corner was long enough
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

    if(state == 1){ // Fast driving 
        hall.loop();
        lights.front_lights(false);
        lights.brake_lights(false);

        TrackSection current_section = track[track_p];

        switch(current_section.type){   // Change speed based on the actual segment
            case STRAIGHT:
                motor.drive(STRAIGHT_SPEED);
                break;
            
            case BRAKING:
                motor.brake();
                lights.brake_lights(true);
                break;

            case CORNER:
                if(current_section.severity <= 2500){
                    motor.drive(ULTRA_CORNER_SPEED);
                }
                if(current_section.severity > 2500 && current_section.severity <= 3500){
                    motor.drive(FAST_CORNER_SPEED);
                }
                if(current_section.severity > 3500 && current_section.severity <= 4000){
                    motor.drive(MEDIUM_CORNER_SPEED);
                }
                if(current_section.severity > 4000){
                    motor.drive(SLOW_CORNER_SPEED);
                }

                break;

            case CORNEREXIT:
                motor.drive(CORNER_EXIT_SPEED);                
                break;

            default:
                break;

        }

        
        if(hall.getRotations() == current_section.end_position){    // Switch to next segment when the car is at the end of the current one
            track_p ++;
            if(track[track_p].type == NONE){
                track_p = 0;
            }
        }


        if(hall.getTraveledDistance() >= TRACK_LENGTH){ // Reset values when the car drives through the whole lap
            if(millis() - last_lap_added > 1000){
                error_coeff_added = false;
                lap_count ++;
                last_lap_added = millis();
                
                // Setting rotations to less than zero because of error when counting rotations while driving
                int error_coeff = -1 * ((TRACK_LENGTH / ERROR_CONSTANT) + (TRACK_LENGTH % ERROR_CONSTANT != 0));  // Expresion in second pair of paretheses is used to round up the result
                
                hall.setRotations(error_coeff);
                if(lap_count % 3 == 0){
                    hall.setRotations(error_coeff - 2);
                }
                
            }
        }
    }

    if(state == 2){ // State that prints debug info to log file on SD card
        motor.brake();
        String section = "";
        for (int i = 0; i < 100; i++){

            sd.writeOnce(String(track[i].start_position) + "\t" + String(track[i].end_position) + "\t" + String(track[i].severity) + "\t");
            switch (track[i].type)
            {
            case 0:
                section = "STRAIGHT";
                break;
            case 1:
                section = "BRAKING";
                break;
            case 2:
                section = "CORNER";
                break;
            case 3:
                section = "C_EXIT";
                break;
            case 4:
                section = "NONE";
                break;
            
            default:
                break;
            }

            sd.writeOnce(section + "\n");

        }
        while (42);

    }
}
