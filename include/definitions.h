#include "lighting.h"
#include "lighting_setup.h"
#include "usb_setup.h"
#include <RF24.h>


#include <QNEthernet.h> 
using namespace qindesign::network;

#include  <SPI.h>
#include <Streaming.h>
#include <pid.h>

#include <Wire.h> // needed for I2C communication
#include <MPU6050_light.h>

//#define ENCODER_DO_NOT_USE_INTERRUPTS
#include "Encoder.h"


int ledPin = 13;


// Networking ======================================================================

byte mac[] = {0x4, 0xE9, 0xE5, 0xE, 0x37, 0xC2}; // burned into hardware 


void teensyMAC(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
    Serial.println(mac[0], HEX);
    Serial.println(mac[1], HEX);
    Serial.println(mac[2], HEX);
    Serial.println(mac[3], HEX);
    Serial.println(mac[4], HEX);
    Serial.println(mac[5], HEX);

    Serial.println(Ethernet.localIP());

}
// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

IPAddress ip(169, 254, 195, 56);

#define UDP_TX_PACKET_MAX_SIZE 128

unsigned int localPort = 8888;      // local port to listen on
// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";        // a string to send back


uint16_t controlRX[25]; // Each command comes through as an element binary element in here

// XBOX CONTROLLER VARIABLE ASSIGNMENTS

struct gamepad{
    bool x, y, a, b, lb, rb; // buttons, either 0 or 1
    int rt, lt, rx, ry, lx, ly;
    byte dx, dy;
};

gamepad control;

void clearControl(){
          control.x = 0;
          control.y = 0;
          control.b = 0;
          control.dy = 0;
          control.dx = 0;
          control.rt = 0;
          control.lt = 0; 
          control.rx = 32768; // neutral midrange valuegnf
          control.ry = 32768;
          control.lx = 32768;
          control.ly = 32768;
          control.lb = 0;
          control.rb = 0;
}







bool check_controlRX(){
    int packetSize = Udp.parsePacket();
        if (packetSize>-1) {
         
          /*
         
          Serial.print("Received packet of size ");
          Serial.println(packetSize);
          Serial.print("From ");
          IPAddress remote = Udp.remoteIP();
          for (int i=0; i < 4; i++) {
            Serial.print(remote[i], DEC);
            if (i < 3) {
              Serial.print(".");
            }
          }
          Serial.print(", port ");
          Serial.println(Udp.remotePort());

          */

          // read the packet into packetBufffer
          Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
          //Serial.println("Contents:");
          memcpy(&controlRX, packetBuffer, sizeof(packetBuffer));

          int len = sizeof(controlRX)/2; // because they're shorts

          control.x = controlRX[0];
          control.y = controlRX[1];
          control.b = controlRX[2];
          control.a = controlRX[3];
          control.dy = controlRX[4];
          control.dx = controlRX[5];
          control.rt = controlRX[6];
          control.lt = controlRX[7];
          control.lx = controlRX[8];
          control.ly = controlRX[9];
          control.rx = controlRX[10];
          control.ry = controlRX[11];
          control.lb = controlRX[12];
          control.rb = controlRX[13];
       /*
          for(int i = 0; i<len; i++){
            Serial.print(controlRX[i]);
            Serial.print(":");
          }
          Serial.println();
       
          //for(int i = 0; i<packetSize; i++){
          //  Serial.print(packetBuffer[i]);
         // }
        // Serial.println();
          

          Serial.println(Ethernet.linkSpeed());

          // send a reply to the IP address and port that sent us the packet we received
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write(ReplyBuffer);
          Udp.endPacket();
          */
         return 1;
        }
      else 
        return 0;

}




