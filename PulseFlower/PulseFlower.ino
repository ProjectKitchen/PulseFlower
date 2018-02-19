/* 

  Heart Pulse Flower
  Version 0.1

  electromechanical flower opens when heart beat synchronizes
  for signal processing traces see serial plotter window
  
  uses Arduino Uno
  connect neopixel strip to pin 2
  servo for opening the flower to pin 9
  and Pulse sensor to anlog input A0

  thanks to the Puzzlebox Bloom project for the great OpenSCAD flower 3d-model 
  for 3d-print files see: https://www.thingiverse.com/thing:618490
  
*/

#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

#define PIN 2
#define PIXELS_STRIP 10
#define PIXELS_FLOWER 7
#define SERVO_CLOSED 22
#define SERVO_OPENED 90
#define MILLIS_SAMPLE 20  // about 50 Hz sampling rate
#define AVG_LEN 8         // 8 samples for averager 
#define PEAK_LEN 200      // 200 samples = 2 seconds sample buffer
#define MIN_FLOWERBRIGHTNESS 10
#define ADD_BRIGHTNESS 25

#define RISING 0
#define FALLING 1

#define STATE_CLOSED  0
#define STATE_OPENING 1
#define STATE_OPENED  2
#define STATE_CLOSING 3


Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXELS_STRIP + PIXELS_FLOWER, PIN, NEO_GRB + NEO_KHZ800);
uint16_t avgbuf[AVG_LEN]={0};
uint16_t peakbuf[PEAK_LEN]={0};


Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

void setup() {
  Serial.begin(115200);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(SERVO_CLOSED);  // attaches the servo on pin 9 to the servo object
  delay(1000);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  for (int i=0;i<AVG_LEN;i++) 
    avgbuf[i]=0;
  for (int i=0;i<PEAK_LEN;i++) 
    peakbuf[i]=0;
}

uint32_t counter =0;
int pos = SERVO_CLOSED;        // variable to store the servo position
uint8_t state = STATE_CLOSED;  // variable to store the servo position
uint8_t bypass_servo=0,old_servo=0;
int rainbowspeed=3;
int waitcnt=0;
uint16_t actsample=0, averaged_sample=0, last_sample;
uint16_t actpeak=0;
uint32_t last_timestamp=0;
uint8_t avgpos=0;
uint16_t peakpos=0;
uint32_t avgsum=0;
uint16_t peak_max=0,peak_min=1024,pulse_threshold=0;
uint8_t signalstate=0,last_signalstate=0;
uint8_t peak_detected=0;
uint8_t flower_brightness=128;


void loop() {
  
  counter=(counter+1) % 5000;

  switch (state) {
    
    case STATE_CLOSED:  // flower closed, wait for reopen
            bypass_servo=1;
            rainbowspeed=1;
            break;

    case STATE_OPENING: // flower opening
            bypass_servo=0;
            if (!(counter % 25)) {
                if (pos<SERVO_OPENED) 
                  pos++; 
                else state=STATE_OPENED;
            }
            break;

    case STATE_OPENED: // flower opened, wait for closing
            bypass_servo=1;
            rainbowspeed=2;
            if (flower_brightness < 150) 
                  state=STATE_CLOSING;
            break;
            
    case STATE_CLOSING:  // flower closing
            bypass_servo=0;
            if (!(counter % 25)) { 
              if (pos>SERVO_CLOSED) { 
                pos--;
              } else state=STATE_CLOSED;
            }
            break;
  }

  if ( bypass_servo ) {
    if ( !old_servo ) myservo.detach();
    if (!(counter % 20)) { 
      rainbowCycle();
      strip.show(); 
    }
  }
  else {
    if ( old_servo ) myservo.attach(9);
    if (!(counter % 5)) myservo.write(pos);
    if (!(counter % 20)) { 
      rainbowCycle();
      strip.show(); 
    }
  }

  if (!(counter % 20)) {
    if (flower_brightness>MIN_FLOWERBRIGHTNESS) 
      flower_brightness--; 
  }

  
  old_servo=bypass_servo;

  if (millis()-last_timestamp > MILLIS_SAMPLE) {
    last_timestamp=millis();
    actsample=analogRead(0);

    avgbuf[avgpos]=actsample;
    avgpos=(avgpos+1) % AVG_LEN;

    avgsum+=actsample;
    avgsum-=avgbuf[avgpos];

    averaged_sample=avgsum/AVG_LEN;

    peakbuf[peakpos]=averaged_sample;
    peakpos=(peakpos+1) % PEAK_LEN;

    uint8_t st=RISING,ost=RISING,peakcount;
    peak_max=0;peak_min=1024;peakcount=0;
    for (int i=0;i<PEAK_LEN-1;i++) {
      if (peak_max<peakbuf[i]) peak_max=peakbuf[i];
      if (peak_min>peakbuf[i]) peak_min=peakbuf[i];   
      if (peakbuf[i]>peakbuf[i+1]) st=RISING; else st=FALLING;
      if ((ost != FALLING) && (st=FALLING) && (peakbuf[i]>pulse_threshold))
         peakcount++;  
      ost=st;
    }
    pulse_threshold=peak_max-(peak_max-peak_min)/4;

    if (last_sample<averaged_sample) signalstate=RISING;
    else {
       signalstate=FALLING;
       if ((last_signalstate != FALLING) && (averaged_sample > pulse_threshold)
           && (peakcount>=2) && (peakcount<12)  && (peak_max - peak_min>20))
          peak_detected=20;  
    }

    if (peak_detected) {
       peak_detected--;
       
       if (!peak_detected) {
         if (flower_brightness<215) {
           flower_brightness+=ADD_BRIGHTNESS; 
         } else { 
           flower_brightness = 255; 
           if (state!=STATE_OPENING) state=STATE_OPENING; 
         }
       }
    }
    
    last_sample=averaged_sample;
    last_signalstate=signalstate;

    Serial.print(actsample);
    Serial.print(",");
    Serial.print(averaged_sample);
    Serial.print(",");
    Serial.print(pulse_threshold);
    Serial.print(",");
    Serial.print(peak_detected);
    Serial.print(",");
    Serial.println(peakcount);

  }
  delay (2);
}


void rainbowCycle() {
  uint16_t i;
  uint32_t actcol;
  static uint16_t j=0;
  j=j+rainbowspeed; if (j>1280) j-=1280; 
  for(i=0; i< strip.numPixels(); i++) {
    if (i<PIXELS_STRIP) {
      if (peak_detected) {
        int br=peak_detected-i;
        if (br>10) br=20-br; 
        if (br<0) br=0;
        br*=4;
        if (br==0) actcol= (((uint32_t) 0)<< 16 )+ (((uint32_t) 10) << 0) + ((uint32_t) 5);
        else actcol= (((uint32_t) br*2)<< 16 )+ (((uint32_t) br) << 8);
        
      } else { /*
        actcol=Wheel(((i * 256 / strip.numPixels()) + j) & 255);
        uint8_t r=(actcol>>16) & 0xff;
        uint8_t g=(actcol>>8) & 0xff;
        uint8_t b=actcol&0xff;
        r>>=3; g>>=3; b>>=3;
        actcol= (((uint32_t) r)<< 16 )+ (((uint32_t) g) << 8) + ((uint32_t) b); */
        actcol= (((uint32_t) 0)<< 16 )+ (((uint32_t) 10) << 0) + ((uint32_t) 5);

      }
    } else {  
      actcol=Wheel(((i * 256 / strip.numPixels()) + j) & 255);
      uint32_t r=(actcol>>16) & 0xff;
      uint32_t g=(actcol>>8) & 0xff;
      uint32_t b=actcol&0xff;
      r*=flower_brightness;r>>=8; 
      g*=flower_brightness;g>>=8; 
      b*=flower_brightness;b>>=8; 
      actcol= (((uint32_t) r)<< 16 )+ (((uint32_t) g) << 8) + ((uint32_t) b);

    }
    strip.setPixelColor(i, actcol);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
