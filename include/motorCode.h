#ifndef MOCONT_H
#define MOCONT_H


#include "Encoder.h"
#include <Arduino.h>


class moCont{

  // Combined motor/encoder controller
 
 private: // can only be accessed internally
      byte _en,  _dir1, _dir2, _e1, _e2;
      Encoder* encoder;
  

 public:
        // motor's encoder position
     
        long pos;
        long last_pos;
        unsigned long tickTimer;
        int tickNum; // store instantaneous encoder ticks per a specified time period for each motor
    
        long prevT; // start the timer for speed measurements
        float tickSpeed; // ticks per second instantaneous
        float groundSpeed;

        float cutoff_freq; // encoder
        float sampling_time; // filter sampling time in seconds
        float filtered_tickSpeed;
        float RPM;

        float ticksPerRev; // how many encoder hits per revolution for speed calcultion
        float speed;
        float wheel_d;  // needed for linear speed calculation


        float lastV; // most previous velocity value, used to compute currentA
        float currentV;  // instantaneous velocity value
        float currentA;  // instantaneous acceleration, calcaulated alongside current v
        float outputSpeed; // used to control speed to acceleration moves
        float speedError;

        float kP;
        float kI;
        float kD;
        float u; // raw speed signal
        byte pwr; // power currently being written to motor
        float et;
        float eintegral;
        float deltaT; // time difference between speed measurements
       
        // the possible states of the state-machine
        enum motorStates{ STATE_IDLE,  
                          STATE_ACC, 
                          STATE_STEADY,
                          STATE_DECC};

        // current state-machine state
        enum motorStates mState;


        // motor constructor
        moCont(byte _en, byte _dir1, byte _dir2, byte _e1, byte _e2); 
        
      
        void bck(byte speed);
        void fwd(byte speed);
        void setMotor(int velocity);

        void stop();

        long getPos();

        // Polling based speed measurements
        void getTicks(); // get number of ticks counted in a given time frame
        void computeRPM(); // compute rpm from number of ticks
        void computeGroundSpeed(); // linear ground speed at wheel contact

        void getSpeed(); // execute getting a total speed state for the wheel

        void set_RPM(float targetRPM, int dir);

}; // end of class definition




#endif