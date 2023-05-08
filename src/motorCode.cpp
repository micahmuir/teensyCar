#include "motorCode.h"


moCont::moCont(byte set_en, byte set_dir1, byte set_dir2, byte set_e1, byte set_e2){
        _en = set_en;
        _dir1 = set_dir1;
        _dir2 = set_dir2;
        _e1 = set_e1;
        _e2 = set_e2;

        pinMode(set_en,OUTPUT);
        pinMode(set_dir1,OUTPUT);
        pinMode(set_dir2,OUTPUT);
        pinMode(set_e1, INPUT_PULLUP);
        pinMode(set_e2, INPUT_PULLUP);

        encoder = new Encoder(_e1, _e2); // WHY WHY WHY WHAT IS HAPPENING DON'T TOUCH ANYTHING          
} 
        

long pos;
long last_pos  = 0;
unsigned long tickTimer;
int tickNum; // store instantaneous encoder ticks per a specified time period for each motor

long prevT = micros(); // start the timer for speed measurements
float tickSpeed; // ticks per second instantaneous
float groundSpeed;

float cutoff_freq = 60.0; // encoder
float sampling_time = 0.005; // filter sampling time in seconds
float filtered_tickSpeed;
float RPM;

float ticksPerRev; // how many encoder hits per revolution for speed calcultion
float speed;
float wheel_d;  // needed for linear speed calculation


float lastV; // most previous velocity value, used to compute currentA
float currentV;  // instantaneous velocity value
float currentA;  // instantaneous acceleration, calcaulated alongside current v
float outputSpeed = 0; // used to control speed to acceleration moves
float speedError;

float kP = 0.0;
float kI = 0.0;
float kD = 0.0;
float u = 0; // raw speed signal
byte pwr = 0; // power currently being written to motor
float et;
float eintegral = 0;
float deltaT; // time difference between speed measurements

// the possible states of the state-machine
enum motorStates{ STATE_IDLE,  
                    STATE_ACC, 
                    STATE_STEADY,
                    STATE_DECC};

// current state-machine state
enum motorStates mState;



void moCont::bck(byte speed){
    analogWrite(_en, speed);
    digitalWrite(_dir1, HIGH);
    digitalWrite(_dir2, LOW);
}

void moCont::fwd(byte speed){
    analogWrite(_en, speed);
    digitalWrite(_dir1, LOW);
    digitalWrite(_dir2, HIGH);
}

void moCont::setMotor(int velocity){ // set motor speed with direction
    velocity = constrain(velocity, -255, 255);
    
    if(velocity >0){
        moCont::fwd(abs(velocity));
    }
    else if(velocity<0){
        moCont::bck(abs(velocity));
    }

    else if(velocity == 0){
        moCont::stop();
    }
}


void moCont::stop(){
    // analogWrite(en, speed);
    digitalWrite(_dir1, LOW);
    digitalWrite(_dir2, LOW);
    }

long moCont::getPos(){
// compute velocity with time between ticks
    pos = encoder->read();
    return pos;
}

void moCont::getTicks(){
    long currT = micros();
    deltaT = float( currT-prevT);
    if(deltaT>=20000){
        pos = encoder->read();
        tickSpeed = (pos-last_pos)/(deltaT/1000000); // number of ticks in 5 ms, 200 hz
        last_pos = pos;
        prevT = currT;
        //Serial.println(tickSpeed);
    }
}

void moCont::computeRPM(){
RPM = (tickSpeed/ticksPerRev)*60;
}

void moCont::computeGroundSpeed(){
// will vary with each wheel
    float circum = PI*wheel_d;
    groundSpeed = (circum*RPM)/60000; // RPM to m/s
}

void moCont::getSpeed(){
    getTicks();
    computeRPM();
    computeGroundSpeed();
}

void moCont::set_RPM(float targetRPM, int dir){
        moCont::getSpeed();
        // PI controller to get to this speed

        et = targetRPM - abs(RPM); // error term in m/s
        eintegral = eintegral + et*deltaT; // starts off as 0
        u = kP*et + kI*eintegral; // Process variable. signal to be added to the current speed value


        pwr = pwr+(int) (u);
          if(pwr>255){
            pwr=255;
          }

        if(dir>0){
          moCont::fwd(pwr);
        }
        else if(dir<0){
          moCont::bck(pwr);
        }
          
      }

  
