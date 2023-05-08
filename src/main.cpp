#include <Arduino.h>
#include "definitions.h"
#include "rigid_body.h" // controller for the car as a rigid body






MPU6050 mpu(Wire); // 

RIGID_BODY body; // This is the base piece of the robot. 


void turnDir(bool dir, float setSpeed){ 
  // accelerated turn in various directions
  if(dir == 0){

  }
   
  if(dir == 1){
     
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



void heartbeat(){
 

  EVERY_N_MILLISECONDS(3000){
     digitalWrite(ledPin, blinkState);
     blinkState = !blinkState;
     Serial.println("heartbeat");
    // teensyMAC(mac);
  }
}



void setup() {  // Setup runs once per reset

// initialize serial communication @ 9600 baud:
  Serial.begin(9600);
  stdPrint = &Serial; // enables printf statements
  Ethernet.begin(mac, ip);   // MAC address already known by library
  Ethernet.setHostname("teensy");
  Ethernet.waitForLocalIP(10000); // Wait for local IP

  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
 // start UDP
  Udp.begin(localPort);

  Serial.println("assigning pins...");
  pinMode(ledPin, OUTPUT); //onboard led

  // L298 Pin Connections


  clearControl(); //set all initial control signals to 0

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



  Serial.println("Setup Complete!");

}



void loop() {
  
  //heartbeat();

  mpu.update();
  bool rslt = check_controlRX();

  int turnForce = 0;
  int lineForce = 0;

  if(rslt){ 
        
  }

  //getSpeeds(5); // needed to initialize the speed control loop
  // Serial << m0.pos << ":" << m1.pos << ":" << mpu.getAccAngleX() << endl;

  body.getPositions();
  
  // First set translational forces

       if(control.rt > 60){
        lineForce = map(control.rt, 0, 1023, 0, 255);
      }
      

      if(control.lt > 60){
        lineForce = map(control.lt, 0, 1023, 0, -255);
       
      }

        // Then set turning forces
      if(control.lx != 32768){
        turnForce = map(control.lx, 0, 65536, -255, 255);
      }

      lineForce = constrain(lineForce, -255, 255);

      body.translate(lineForce); 
      body.turn(turnForce);
      

      body.go(); // move the body in accordance to the applied controller forces

      

      Serial << body.m0_v << ":" << body.m1_v << ":" << lineForce << ":" << turnForce << endl;

 //   m0.fwd(controlRX[6]/4);
//  m1.fwd(controlRX[7]/4);



}





