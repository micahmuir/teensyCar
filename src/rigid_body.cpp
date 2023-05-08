#include "rigid_body.h"
#include "motorPins.h"


RIGID_BODY::RIGID_BODY(){
  // MOTOR SETUP FOR THE RIGID BODY
  // Class for a rigid body controller. 
  // Motors, encoders, and IMU should join together here
  // Also set motor parameters

    m0 =  new moCont(enA,dir1_a,dir2_a, e1a, e2a); // driver right, OUT 1 2
    m1 =  new moCont(enB,dir1_b,dir2_b, e1b, e2b); 
    
    int motorPWMFreq = 20000;  // High frequency for inaudible drivers 

    analogWriteFrequency(enA, motorPWMFreq);
    analogWriteFrequency(enB, motorPWMFreq); 

    m0->wheel_d = 100.0;
    m0->ticksPerRev = 2200;

    m1->wheel_d = 100.0;
    m1->ticksPerRev = 2200;
} 


int m0_v = 0;
int m1_v = 0;

void RIGID_BODY::getPositions(){
    m0_pos = m0->getPos();
    m1_pos = m1->getPos();
}


void RIGID_BODY::translate(int speed){ // First establish translational speed vector
    m0_v = speed;
    m1_v = speed;
}

void RIGID_BODY::turn(int moment){ // then determine joystick turning contribution 

    if(moment > 0){
        m0_v =  m0_v + abs(moment);
        m1_v =  m1_v - abs(moment);
        m0_v = constrain(m0_v, -255, 255);
        m1_v = constrain(m1_v, -255, 255);
    }

     if(moment < 0){
        m0_v =  m0_v - abs(moment);
        m1_v =  m1_v + abs(moment);
        m0_v = constrain(m0_v, -255, 255);
        m1_v = constrain(m1_v, -255, 255);
    }
}



void RIGID_BODY::go(){
    
    if(abs(m0_v) < 20){
        m0_v = 0;
    }

     if(abs(m1_v) < 20){
        m1_v = 0;
    }



    m0->setMotor(m0_v);
    m1->setMotor(m1_v);
    

}