#include <SoftwareSerial.h> //Used to allow serial communications via all pins
#include <MedianFilterLib2.h> //Used for filtering sensor values
#include <FastLED.h> //Used for LED strips

#define NUM_LEDS 60 //LED strip
#define DATA_PIN 5
#define CLOCK_PIN 6
CRGB leds[NUM_LEDS];

//defines pin numbers
const int s0 = 13; //multiplexing select bit 0 - the others should be grounded if only two sensors are used
const int signalpin[2] = {2,3}; //pins used for sending sensor signal
const int TX = 9; //Processed value / real time value output selection
const int RX = 8; //Reception from sensors

SoftwareSerial mySerial(RX, TX);

uint8_t sum[2] = {0,0};
uint8_t data[2][4] = {{0, 0, 0, 0},{0,0,0,0}};
unsigned int distance[2] = {0, 0};

//Sensor switching variable - two sensors used means that it will assume values 0 and 1
//values chosen to use arrays more efficiently
int sensor = 0;

//Variables for running average
int NextRunningMedian = 0;
const int RunningMedianCount = 121; //The history of measurement over which the median is taken. Should be tweaked.
unsigned int RunningMedianBuffer[2][RunningMedianCount];
unsigned int RunningMedianDistance[2] = {0, 0};

//Variables used for time tracking during occupation and certain events
unsigned long count[2] = {0, 0};
unsigned long lowCount[2] = {0, 0};
unsigned long lowCountInterrupt[2] = {0,0};
bool flag[2] = {false, false};
unsigned long offset[2] = {0, 0};
unsigned long lowOffset[2] = {0, 0};
unsigned long lowOffsetInterrupt[2] = {0,0};

//Variables defining the limits of the detection logic
int timeThreshold = 30;      // 30 seconds parking or more -> occupied
int lowTimeThreshold = 30;   // 30 seconds after an object leaving the interval -> vacant
int lowTimeInterruptThreshold = 10; //Timeframe in which an object is allowed to reenter and leave the interval before it is registered as still being occupied
int distanceUpperThreshold = 1500;  // car parked in distance interval between 0.2 and 1.5 metres?
int distanceLowerThreshold = 200;

int signalTest[2] = {0, 0}; //Used to test the detection logic

/*
   https://wiki.dfrobot.com/A02YYUW%20Waterproof%20Ultrasonic%20Sensor%20SKU:%20SEN0311#target_6
   The data is sent with a header, 16 bits for the distance measurement in mm and a checksum. The maximum range of 450 cm requires
   a minimum of 13 bits.
   Unsigned char is 8 bit.
*/

//Constructing a median filter using the MedianFilterLib2 library
MedianFilter2 < unsigned int > medianFilter1( RunningMedianCount );
MedianFilter2 < unsigned int > medianFilter2( RunningMedianCount );

void setup()
{
  // LED setup
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  
  Serial.begin(57600);
  mySerial.begin(9600);
  Serial.println("BEGIN SESSION");
  pinMode(s0,OUTPUT); //select bit used for multiplexing
  pinMode(signalpin[0],OUTPUT);
  pinMode(signalpin[1],OUTPUT);
  digitalWrite(signalpin[0],LOW);
  digitalWrite(signalpin[1],LOW);
}

void loop()
{
  
  //Load the 4 bytes of information from serial using do/while when the header reads 8 high bits from serial
  do {
    for (int i = 0; i < 4; i++)
    {
      data[sensor][i] = mySerial.read();
    }
  } while (mySerial.read() == 0xff); //Loading only happens when a header signal is detected

  //flush serial after reading
  mySerial.flush();
  
  if (data[sensor][0] == 0xff){
    
    sum[sensor] = (data[sensor][0] + data[sensor][1] + data[sensor][2]) & 0x00ff; //value used for the checksum - the header is removed by 'and'-ing with 0 in the largest values
    
    if (sum[sensor] == data[sensor][3]) //compares the received measurement data with the sent checksum for error detection
    {
      distance[sensor] = (data[sensor][1] << 8) + data[sensor][2]; //Final distance value in mm obtained when shifting and adding the high and low bits
    }
  }

  //Adding the distance value to the median filter, which also returns the median of the buffer
  if (sensor == 0){
    RunningMedianDistance[sensor] = medianFilter1.AddValue(distance[sensor]);
  }
  else{
    RunningMedianDistance[sensor] = medianFilter2.AddValue(distance[sensor]);
  }
  
  //Car in distance interval?
  if ((RunningMedianDistance[sensor] >= distanceLowerThreshold) && RunningMedianDistance[sensor] <= distanceUpperThreshold) {
      if (flag[sensor] == false) {
        flag[sensor] = true;
        offset[sensor] = millis(); //time offset for the in-interval time tracking (count)
      }
    
      else if (flag[sensor] == true && lowCount[sensor] != 0) { //If an object is re-detected after a short, low period of duration below X seconds

      //The redetection in interval should not cause lowCount to reset instantly
        if (lowOffsetInterrupt[sensor] == 0){
          lowOffsetInterrupt[sensor] = millis();
        }
        else{
          lowCountInterrupt[sensor] = (millis() - lowOffsetInterrupt[sensor]) / 1000;
          lowCount[sensor] = (millis() - lowOffset[sensor]) / 1000;
          count[sensor] = (millis() - offset[sensor]) / 1000; // All counts continue if in a low period being interrupted
        }
        if((lowCountInterrupt[sensor] > lowTimeInterruptThreshold) || lowCount[sensor] > lowTimeThreshold){
          lowCount[sensor] = 0;
          lowCountInterrupt[sensor] = 0;
          lowOffset[sensor] = 0;
          lowOffsetInterrupt[sensor] = 0;
        }
      }
      else {
          count[sensor] = (millis() - offset[sensor]) / 1000; //Counting the time an object is within the interval
      }
 }
   
    //If an object was in the interval and moves out, the count should not immediately reset.
    //This would potentially cause problems with detection resetting if there was a short period of low values.
    //The following logic combats that issue.
    if (((RunningMedianDistance[sensor] < distanceLowerThreshold) || (RunningMedianDistance[sensor] > distanceUpperThreshold)) && signalTest[sensor] == true) {
      if (lowOffset[sensor] == 0) {
        lowOffset[sensor] = millis();
      }
      else {
        lowCount[sensor] = (millis() - lowOffset[sensor]) / 1000;
        count[sensor] = (millis() - offset[sensor]) / 1000; // Both counts continue if in a low period
      }
      //If the detector is currently detecting an object, and the object disappears from the sensors for longer than the specified threshold, the count is reset.
      if (lowCount[sensor] > lowTimeThreshold) {     
        flag[sensor] = false;
        signalTest[sensor] = 0;
        digitalWrite(signalpin[sensor], LOW);
        count[sensor] = 0;
        offset[sensor] = 0;
        lowCount[sensor] = 0;
        lowOffset[sensor] = 0;
      }
    } //If the occupation signal is not high, and an object disappears from the interval, the count is instantly reset without delay.
    else if ((RunningMedianDistance[sensor] < distanceLowerThreshold) || (RunningMedianDistance[sensor] > distanceUpperThreshold)){
      count[sensor] = 0;
      flag[sensor] = false;
    }

    if ((count[sensor] >= timeThreshold) && lowCount[sensor] == 0) {
      signalTest[sensor] = 1;
      digitalWrite(signalpin[sensor], HIGH);

      //LED strip logic
      if (sensor == 0){
        leds[40] = CRGB::Red;
        /*
        for (int i = 40; i < 45; i++){
          leds[i] = CRGB::Red;
        }
        */
      }
      else{
        leds[55] = CRGB::Red;
        /*
        for (int i = 51; i < 56; i++){
          leds[i] = CRGB::Red;
        }
        */
      }      
    } 
    else if ((count[sensor] > 0) || lowCount[sensor] != 0){
      if (sensor == 0){
        leds[40] = CRGB::Blue;  
      }
      else{
        leds[55] = CRGB::Blue;
      }
    }
    else{
      signalTest[sensor] = 0;
      digitalWrite(signalpin[sensor], LOW);

      //LED strip logic
      if (sensor == 0){
        leds[40] = CRGB::Green;
        /*
        for (int i = 40; i < 45; i++){
          leds[i] = CRGB::Green;
        }
        */
      }
      else {
        leds[55] = CRGB::Green;
        /*
        for (int i = 51; i < 56; i++){
          leds[i] = CRGB::Green;
        }
        */
      }      
    }
    FastLED.show();
  
  switch (sensor) { //Switching between the two sensors every loop
  case 0:
    sensor = 1;
    digitalWrite(s0, HIGH);
    break;
  case 1:
    sensor = 0;
    digitalWrite(s0, LOW);
    break;
  default:
    sensor = 0;
    digitalWrite(s0, LOW);
    break;
  }
  delay(100); //Delay to match the 100 ms response time of the sensor
}
