#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include "motorCode.h"
#include <Arduino.h>


class RIGID_BODY{
  // Class for a rigid body controller. Motors, encoders, and IMU should join together here
  private: 
   moCont* m0;
   moCont* m1;

 public:

    long m0_pos, m1_pos;
    
    int m0_v, m1_v; // individual wheel velocities 

    // body vectors
    int thrust; // translation thrust MAGNITUDE, 0:255. direction handled by torque 

    RIGID_BODY(); // constructor 

    void getPositions(); //get instant wheel positions

    void fwd(byte speed); // move body forward at prescribed speed
    void bck(byte speed);

    void translate(int velocity); // determine forward/backward movement speed   
    void turn(int moment); // set the wheels to apply a turning force, if any. <0 = ccw, >0 = cw
    
    void go(); // compute speeds for each wheel, apply forces to body

}; // end of class definition




#endif