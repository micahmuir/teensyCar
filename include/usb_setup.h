#ifndef usb_setup
#define usb_setup

#include <Arduino.h>
#include <USBHost_t36.h>


USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);

#define COUNT_JOYSTICKS 4
JoystickController joysticks[COUNT_JOYSTICKS](myusb);
int user_axis[64];
uint32_t buttons_prev = 0;

USBDriver *drivers[] = {&hub1, &joysticks[0], &joysticks[1], &joysticks[2], &joysticks[3], &hid1};
#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1", "joystick[0D]", "joystick[1D]", "joystick[2D]", "joystick[3D]",  "HID1"};
bool driver_active[CNT_DEVICES] = {false, false, false, false};

// Lets also look at HID Input devices
USBHIDInput *hiddrivers[] = {&joysticks[0], &joysticks[1], &joysticks[2], &joysticks[3]};
#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * hid_driver_names[CNT_DEVICES] = {"joystick[0H]", "joystick[1H]", "joystick[2H]", "joystick[3H]"};
bool hid_driver_active[CNT_DEVICES] = {false};
bool show_changed_only = false;

uint8_t joystick_left_trigger_value[COUNT_JOYSTICKS] = {0};
uint8_t joystick_right_trigger_value[COUNT_JOYSTICKS] = {0};
uint64_t joystick_full_notify_mask = (uint64_t) - 1;

int psAxis[64];

float gx, gy, gz;
float ax, ay, az;
float pitch, roll;
uint16_t xc, yc; 
uint8_t isTouch;
int16_t xc_old, yc_old;


void getCoords(uint16_t &xc, uint16_t &yc, uint8_t &isTouch){

  //uint8_t finger = 0;  //only getting finger 1
  uint8_t Id = 0;


  // Trackpad touch 1: id, active, x, y
  xc = ((psAxis[37] & 0x0f) << 8) | psAxis[36];
  yc = psAxis[38] << 4 | ((psAxis[37] & 0xf0) >> 4),

  isTouch = psAxis[35] >> 7;
  if(xc != xc_old || yc != yc_old){
    Serial.printf("Touch: %d, %d, %d, %d\n", psAxis[33], isTouch, xc, yc);
    xc_old = xc;
    yc_old = yc;
  }
}

void getAccel( float &ax,  float &ay,  float &az){
      int accelx = (int16_t)(psAxis[20]<<8) | psAxis[19];
      int accelz = (int16_t)(psAxis[22]<<8) | psAxis[21];
      int accely = (int16_t)(psAxis[24]<<8) | psAxis[23];
  
      ax = (float) accelx/8192;
      ay = (float) accely/8192;
      az = (float) accelz/8192;
}

void getAngles(float &p, float &r){
  getAccel( ax,  ay,  az);
  p = (atan2f(ay, az) + PI) * RAD_TO_DEG;
  r = (atan2f(ax, az) + PI) * RAD_TO_DEG;
}

void getGyro(float &gx, float &gy, float &gz){
  int gyroy = (int16_t)(psAxis[14]<<8) | psAxis[13];
  int gyroz = (int16_t)(psAxis[16]<<8) | psAxis[15];
  int gyrox = (int16_t)(psAxis[18]<<8) | psAxis[17];

  gx = (float) gyrox * RAD_TO_DEG/1024;
  gy = (float) gyroy * RAD_TO_DEG/1024;
  gz = (float) gyroz * RAD_TO_DEG/1024;
}
  
void printAngles(){
  //test function calls
  float gx, gy, gz;
  getAccel(ax, ay, az);
  Serial.printf("Accel-g's: %f, %f, %f\n", ax, ay, az);
  getGyro(gx, gy, gz);
  Serial.printf("Gyro-deg/sec: %f, %f, %f\n", gx, gy, gz);

  getAngles(pitch, roll);
  Serial.printf("Pitch/Roll: %f, %f\n", pitch, roll);

  getCoords(xc, yc, isTouch);
}



//=============================================================================
// Show when devices are added or removed
//=============================================================================
void PrintDeviceListChanges() {
  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
        driver_active[i] = true;

        const uint8_t *psz = drivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = drivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = drivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }

  for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
    if (*hiddrivers[i] != hid_driver_active[i]) {
      if (hid_driver_active[i]) {
        Serial.printf("*** HID Device %s - disconnected ***\n", hid_driver_names[i]);
        hid_driver_active[i] = false;
      } else {
        Serial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
        hid_driver_active[i] = true;

        const uint8_t *psz = hiddrivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = hiddrivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = hiddrivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }
}




#endif