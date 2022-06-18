#ifndef lighting_setup
#define lighting_setup

#include "lighting.h"


// Color Definitions

      CRGB forward(0,255,0);
      CRGB back(255,0,0);
      CRGB turn(0,0,255);
      CRGB idle(222,130,9);
      CRGB modeLED(15,30,200);

bool blinkState = 0; // flag for the onboard LED


//Class goes here, full declaration AND implementation

// ==================== LED SETUP ===============================================

template <EOrder RGB_ORDER = RGB,
          uint8_t CHIP = WS2811_800kHz>
class CTeensy4Controller : public CPixelLEDController<RGB_ORDER, 8, 0xFF>
{
    OctoWS2811 *pocto;

public:
    CTeensy4Controller(OctoWS2811 *_pocto)
        : pocto(_pocto){};

    virtual void init() {}
    virtual void showPixels(PixelController<RGB_ORDER, 8, 0xFF> &pixels)
    {

        uint32_t i = 0;
        while (pixels.has(1))
        {
            uint8_t g = pixels.loadAndScale0();
            uint8_t r = pixels.loadAndScale1();
            uint8_t b = pixels.loadAndScale2();
            pocto->setPixel(i++, r, g, b);
            pixels.stepDithering();
            pixels.advanceData();
        }

        pocto->show();
    }
};

  const int numPins = 2;
  byte pinList[numPins] = {LV1, LV2};
  const int ledsPerStrip = 9;

  CRGB leds[numPins * ledsPerStrip]; // need to split the matster strip into sections with this method
  CRGBSet rightBar(leds, 0, 8);
  CRGBSet leftBar(leds,  9, 18);
  // These buffers need to be large enough for all the pixels.
  // The total number of pixels is "ledsPerStrip * numPins".
  // Each pixel needs 3 bytes, so multiply by 3.  An "int" is
  // 4 bytes, so divide by 4.  The array is created using "int"
  // so the compiler will align it to 32 bit memory.
  DMAMEM int displayMemory[ledsPerStrip * numPins * 3 / 4];
  int drawingMemory[ledsPerStrip * numPins * 3 / 4];
  OctoWS2811 octo(ledsPerStrip, displayMemory, drawingMemory, WS2811_GRB | WS2811_800kHz, numPins, pinList);

CTeensy4Controller<GRB, WS2811_800kHz> *pcontroller;

// Animation Helper Functions

float breathe(){
    // trig wave
	 float breath = (exp(sin(millis()/3000.0*PI)) - 0.36787944)*108.0;
     return breath;
}

#endif


