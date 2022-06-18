#include <Arduino.h>
#include "motorCode.h" // contains motor and encoder functions
#include "lighting.h"
#include "lighting_setup.h"
#include "usb_setup.h"
#include <RF24.h>

#include  <SPI.h>
#include <Streaming.h>
#include <pid.h>

#include <Wire.h> // needed for I2C communication
#include <MPU6050_light.h>

//#define ENCODER_DO_NOT_USE_INTERRUPTS
#include "Encoder.h"

unsigned long pidPeriod = 30;
unsigned long setpoint = 500;
int32_t kp = 4;
int32_t ki = 2;
int32_t kd = 1;
uint8_t qn = 28;    // Set QN to 32 - DAC resolution
Pid::PID pidController = Pid::PID(setpoint, kp, ki, kd, qn);

//====== Peripheral Sensors ========================================================

MPU6050 mpu(Wire); // 

// ===== Radio Declarations ========================================================

//RF24 radio(35, 33, 36, 37, 34);  // (CE, CSN, SCK, MI, MO)

char data_rx[32];  // raw bytes recieved from wherever 

const uint64_t pipes[] = {
    0xF0F0F0F0E1LL, // 
    0xF0F0F0F0E1LL,
};

// Radio Payload
typedef struct {
    float target_angle; // 
    float target_speed; // total mass
    bool reverseFlag;
}RF_data;

RF_data remote; // we will drop the bytes from data_rx into here 


// ===== Motor Control ============================================================

class motorController{
 
 private: // can only be accessed internally
      byte en, dir, dir1, dir2, e1, e2;
      Encoder* encoder;

 public:
        // motor's encoder position

        long pos;
        long last_pos  = -999;
        unsigned long tickTimer;
        int tickNum; // store instantaneous encoder ticks per a specified time period for each motor
        float ticksPerRev; // how many encoder hits per revolution for speed calcultion
        float speed;
        float wheel_d;  // needed for linear speed calculation

        float et;
        float currentV;
        float outputSpeed = 0; // used to control speed to acceleration moves
        float speedError;
        float kP = 4;
       

        motorController(byte set_en, byte set_dir1, byte set_dir2, byte set_e1, byte set_e2) { // pin assignments at construction
            en = set_en;
            dir1 = set_dir1;
            dir2 = set_dir2;
            e1 = set_e1;
            e2 = set_e2;
            encoder = new Encoder(e1, e2); // WHY WHY WHY WHAT IS HAPPENING DON'T TOUCH ANYTHING          
        };
       
        void begin(){ // call once during setup for each motor
          pinMode(en,OUTPUT);
          pinMode(dir1,OUTPUT);
          pinMode(dir2,OUTPUT);
          pinMode(enB,OUTPUT);
          pinMode(e1, INPUT_PULLUP);
          pinMode(e2, INPUT_PULLUP);
        }
       
        void fwd(int speed){
        analogWrite(en, speed);
        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, LOW);
       }

        void bck(int speed){
        analogWrite(en, speed);
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
      }

        void stop(){
       // analogWrite(en, speed);
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, LOW);
      }

        float getSpeed(unsigned long interval = 30){
           unsigned long  currentTime = millis();
           bool tickFlag = false;
      
                pos = encoder->read();
           
                if (currentTime - tickTimer >= interval) {             
                    tickNum = pos - last_pos;
                    last_pos = pos;
                    tickTimer = currentTime;
                    tickFlag = true;             // true if there's new speed data
                  }

              // computes the linear speed each wheel travels in the interval ms
              
                float circum = PI*wheel_d;
                float tickTravel = circum * float(tickNum);
                float distance = tickTravel/ticksPerRev;

                 float currentSpeed = distance / (float(interval) / 1000.0); // linear speed in mm/s
                // speed = currentSpeed;
                 return currentSpeed; 
              }
             
    
         void accel2speed(float targetV, bool dir, float pThreshold){
            
              if(currentV<=10){    // linear ramp to start moving from 0
                  outputSpeed = outputSpeed + 0.1;
               }

           if(currentV != abs(getSpeed())){
             currentV = abs(getSpeed()); // make sure speed ar up
                 float  percentError = ((1.00 - (currentV/targetV)));
              et = percentError;
            //et = targetV-currentV; // instantaneous error term
              if(currentV>10){
                   outputSpeed = (kP*percentError)+outputSpeed; // compute proportional control term
                   outputSpeed = constrain(outputSpeed, 0.0, 255);
               }  
            }
            
            if(dir == 0){
              bck(outputSpeed); 
            }

            if(dir == 1){
              fwd(outputSpeed); 
            }
           
        }


      //pos = encA.read();

};

// Front Wheels
motorController m0(0,2,1,31,32); // driver right, OUT 1 2
//NOTE ENCODER PIN FLIP FOR CW/CCW SYNC
motorController m1(5,3,4,25,26); // driver left,  OUT 3 4

// Back Wheels
motorController m2(8,6,7,27,28);  // driver right, OUT 1 2

motorController m3(11,9, 10,33,34);    // driver left,  OUT 3 4


void go_fwd(float setSpeed){
       m0.accel2speed(setSpeed, 0, 30);
       m1.accel2speed(setSpeed, 0, 40);
       m2.accel2speed(setSpeed, 0, 30);
       m3.accel2speed(setSpeed, 0, 30);
}


void go_bck(float setSpeed){
       m0.accel2speed(setSpeed, 1, 30);
       m1.accel2speed(setSpeed, 1, 40);
       m2.accel2speed(setSpeed, 1, 30);
       m3.accel2speed(setSpeed, 1, 30);
}


void turnDir(bool dir, float setSpeed){ 
  // accelerated turn in various directions
  if(dir == 0){
       m0.accel2speed(setSpeed, 1, 30);
       m1.accel2speed(setSpeed, 0, 40);
       m2.accel2speed(setSpeed, 1, 30);
       m3.accel2speed(setSpeed, 0, 30);
  }
   
  if(dir == 1){
       m0.accel2speed(setSpeed, 0, 30);
       m1.accel2speed(setSpeed, 1, 40);
       m2.accel2speed(setSpeed, 0, 30);
       m3.accel2speed(setSpeed, 1, 30);
  }


}

 float turnSpeed = 0; 

float turn2angle(float targetAngle){
      float kP = 1.3;
      float currentAngle = mpu.getAngleZ();
     
     //if(turnSpeed<10){
   //    turnSpeed = 10;
   //  }

    if(currentAngle != targetAngle){
        mpu.update();
        currentAngle = mpu.getAngleZ(); // make sure speed ar up
        float  percentError = ((1.00 - (currentAngle/targetAngle)));
      
        float et = targetAngle-currentAngle; // instantaneous error term 

            if(et>0){
                turnSpeed = (kP*et)+turnSpeed; // compute proportional control term
                turnSpeed = constrain(turnSpeed, 0.0, 255);  
               Serial.println(turnSpeed);
               turnDir(0, turnSpeed); // turn towards

            }

            if(et<0){
                turnSpeed = (kP*et)+turnSpeed; // compute proportional control term
                turnSpeed = constrain(turnSpeed, 0.0, 255);  
               Serial.println(turnSpeed);
               turnDir(1, turnSpeed); // turn towards
            }
            

           return et;
    }
            
          
}


void stop(){

       m0.stop();
       m1.stop();
       m2.stop();
       m3.stop();

}

bool btn1, btn2;

void spaceMode(float targetSpeed){
  // wireless space mouse rc mode
  bool debug = 0;
    int vX, vY, vZ, wX, wY, wZ, btn;
 if(joysticks[0].available()){
    //uint32_t buttons = joysticks[0].getButtons();
 

     vX = constrain(joysticks[0].getAxis(0), -255, 255); // velocity x  
     vY = constrain(joysticks[0].getAxis(1), -255, 255); // velocity y
        vY = vY * -1; // invert direction of y axis
     vZ = constrain(joysticks[0].getAxis(2), -255, 255); // velocity z
     wX = constrain(joysticks[0].getAxis(3), -255, 255); // angular velocity x
     wY = constrain(joysticks[0].getAxis(4), -255, 255); // angular velocity y
     wZ = constrain(joysticks[0].getAxis(5), -255, 255); // angular velocity z
     btn = joysticks[0].getButtons();
  
    // forward/naclwards controlled by button
 
    joysticks[0].joystickDataClear();
  }
      if(debug){
        Serial << vX << " : " << vY << " : " << vZ << " : " << btn1 << endl;
    }

    if(btn == 1){
      btn1 = true;
    }

    if(btn1){
      
        turnDir(0,targetSpeed);
      // go_fwd(targetSpeed);
       rightBar[8] = forward;
       leftBar[8] = forward;
    }

    if(btn == 2){
      btn2 = true;
    }

    if(btn2){
      turnDir(1,targetSpeed);
      // go_bck(targetSpeed);
       rightBar[8] = back;
       leftBar[8] = back;
    }

    if(btn == 0){
      btn1 = false;
      btn2 = false;
      stop();
          Serial.println("THAT");
    }
 }

void IMU_nav(float angle_to_turn){
    
  // rotate to a prescribed angle through dynamic motor control 

}

void heartbeat(){
  EVERY_N_MILLISECONDS(500){
     digitalWrite(ledPin, blinkState);
     blinkState = !blinkState;
     Serial.println("heartbeat");
  }
}


void setup() {  // Setup runs once per reset

// initialize serial communication @ 9600 baud:
  Serial.begin(9600);

  Serial.println("assigning pins...");

  pinMode(ledPin, OUTPUT); //onboard led

  // L298 Pin Connections
  pinMode(enA,OUTPUT);
  pinMode(dir1_a,OUTPUT);
  pinMode(dir2_a,OUTPUT);
  pinMode(enB,OUTPUT);
  pinMode(dir1_b,OUTPUT);
  pinMode(dir2_b,OUTPUT);
  pinMode(enC,OUTPUT);
  pinMode(dir1_c,OUTPUT);
  pinMode(dir2_c,OUTPUT);
  pinMode(enD,OUTPUT);
  pinMode(dir1_d,OUTPUT);
  pinMode(dir2_d,OUTPUT);

  int motorPWMFreq = 20000;

  analogWriteFrequency(enA, motorPWMFreq); // High frequency for inaudible drivers 
  analogWriteFrequency(enB, motorPWMFreq); 
  analogWriteFrequency(enC, motorPWMFreq); 
  analogWriteFrequency(enD, motorPWMFreq); 

   // analogWriteFrequency(LV1, 4000); 

  // Motor Encoder Pin Connections
  pinMode(e1a, INPUT_PULLUP);
  pinMode(e2a, INPUT_PULLUP);
  pinMode(e1b, INPUT_PULLUP);
  pinMode(e2b, INPUT_PULLUP);
  pinMode(e1c, INPUT_PULLUP);
  pinMode(e2c, INPUT_PULLUP);
  pinMode(e1d, INPUT_PULLUP);
  pinMode(e2d, INPUT_PULLUP);

  myusb.begin();
  Serial.println("Started USB Host");

// Motor Setup
  m1.begin();
  m0.begin();
  m2.begin();
  m3.begin();

  m0.wheel_d = 100.0;
  m1.wheel_d = 100.0; 
  m2.wheel_d = 93.0;
  m3.wheel_d = 93.0;

  m0.ticksPerRev = 2200;
  m1.ticksPerRev = 2200;
  m2.ticksPerRev = 760;
  m3.ticksPerRev = 760;
  // MPU6050 Setup

  Wire.begin();                      // Initialize comunication
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {} // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  // Lighting Setup
  octo.begin();
    pcontroller = new CTeensy4Controller<GRB, WS2811_800kHz>(&octo);

    FastLED.setBrightness(255);
    FastLED.addLeds(pcontroller, leds, numPins * ledsPerStrip);
        
  // PID Setup

    pidController.setOutputMin(0);      // This is the default
    pidController.setOutputMax(255);   // The Arduino has a 10-bit'analog' PWM output,
                                      // but the maximum output can be adjusted down.
    pidController.init(0);  // Initialize the pid controller to make sure there
                                      // are no output spikes
  
  
    // give a little rainbow flare to startup
    for(int i = 0; i<=255; i++){
	  	fill_solid(rightBar,  NUM_LEDS, CHSV(i, 255, 64));
      fill_solid(leftBar,  NUM_LEDS, CHSV(i, 255, 64));
	  	FastLED.delay(5);
	    FastLED.show();
	}

  Serial.println("Setup Complete!");

}

void getSpeeds(unsigned long interval){
  //m0.speed = m0.getSpeed(interval);
  m1.speed = m1.getSpeed(interval);
  m0.speed = m0.getSpeed(interval);
  m2.speed = m2.getSpeed(interval);
  m3.speed = m3.getSpeed(interval);
}
 
byte mode = 1;
bool turnStart = 0;
bool turnComplete = 0;
  float Pz_mark; 
  float Pz_now;
  int outputSpeed = 0; // increments for acceleration control

void loop() {
  FastLED.clear();
  mpu.update();
  myusb.Task(); // check for new USBhost device information
  heartbeat();
    getSpeeds(30); // get current speeds for each motor
          
  
  
  mode = 0;
  rightBar[mode] = modeLED;
  leftBar[mode] = modeLED;

  switch(mode){
    
    case 0:
         // getSpeeds(30); // get current speeds for each motor
          float test_speed = 230;
          float test_angle = 30;

        float pError = turn2angle(test_angle);

          //spaceMode(test_speed);
    //  go_bck(200);
     // go_fwd(600);
        //m0.fwd(30);
    //  m3.fwd(100);

       //go_fwd(20);
    
     // m0.bck(80);
       // float test = m2.getSpeed();
      //Serial << test << endl;  
        Serial << m0.speed << " : " << m1.speed << " : " << m2.speed << " : " << pError << " : " << mpu.getAngleZ() <<  endl;  
      // Serial << m0.speed << " : " << m1.speed << " : " << m2.speed << " : " << m3.speed << " : " << mpu.getAngleZ() <<  endl;      
 //Serial << m0.pos << " : " << m1.pos << " : " << m2.pos << " : " << m3.pos << endl;    
   //    Serial << m3.speed<< "  :  " << m3.et << "  :  " << m3.outputSpeed  << endl;  
    break;
    
    case 1:
 
     // spaceMode();

    break;
    default:

    break;
  }


  FastLED.show();
}

