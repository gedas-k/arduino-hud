#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//Variables:
int FOV = 20; // field of view
// Pressent position:
double presXtest = 23.318922;
double presYtest = 55.925182;
int presElev = 100; // altitude in meters
// Target position:
double tgtX = 23.312558;
double tgtY = 55.928551;
int tgtElev = 120; // target elevation in meters
bool smoot = true; // is smooting on

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// The TinyGPS++ object
TinyGPSPlus gps;

// If using software SPI (the default case):
#define OLED_MOSI   6
#define OLED_CLK   3
#define OLED_DC    8
#define OLED_CS    11
#define OLED_RESET 7
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
//TVout TV;
//unsigned char x,y;

unsigned char hor;
unsigned char ver;
unsigned char mid;
unsigned char pxInDegree;
unsigned char lineAmount;
unsigned char spaces;
unsigned char origin;
double presX;
double presY;
int delta;
float alpha;
int alphaS;

// getting ready for smoothing function:
const int numReadings = 10;     // number of readings
float readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
float sum = 0;                    // the running total
float average = 0;                // the average

void setup() {
  Serial.begin(9600);

  // Initialise the sensor
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done
  
  // setup text properties
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  hor = display.width() - 1;
  ver = display.height() - 1;
  mid = hor / 2;
  pxInDegree = hor / FOV; // how many pixels in one degree
  lineAmount = (FOV / 5) + 1; // how many side lines on screen
  spaces = 5 * pxInDegree; // spaces betwean lines
  origin = mid - ((FOV/10) * spaces); // where starts lines

  //Serial.print("Spaces: "); Serial.println(spaces);
  
  // initialize all the readings to 0 for smoothing:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

}

void loop() {

  display.clearDisplay(); // clear the buffer
  
  int heading = getHeading();
  calcPosition();
  calcOffsetToTGT(heading);
  drawHeading(heading);
  drawCircle();

  display.display(); // drawing everything

}

float getHeading()
{

  /* Get a new sensor event */
  sensors_event_t event; 
  mag.getEvent(&event);
  
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = 0;
  if (smoot == true)
  {
    heading = filterHeading(heading);
  }
  else
  {
    heading = atan2(event.magnetic.y, event.magnetic.x);
  }
  
  //heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  heading = heading * 180/M_PI; // UNCOMENT/////////////////////////
  return heading;

}

float filterHeading(int heading)
{
  // subtract the last reading:
  sum = sum - readings[readIndex];
  // read from the sensor:
//  readings[readIndex] = atan2(event.magnetic.y, event.magnetic.x);;
  // add the reading to the total:
  sum = sum + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  
  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
  
  // calculate the average:
  average = sum / numReadings;
  // return it:
  heading = average;
  //delay(1);        // delay in between reads for stability
  return heading;
}

void calcPosition()
{
  //double presX = gps.location.lng(); UNCOMENT FOR GPS!!!!!!!!!!!!
  //double presY = gps.location.lat();
  double presX = presXtest;
  double presY = presYtest;

  // calculate altitude
  //Change altitude with pot for testing:
  presElev = analogRead(A0);
  presElev = map(presElev, 0, 1023, 0, 2000);
}

void calcOffsetToTGT(int heading)
{

  // calculate heading to target
  double tgtHdg =
    TinyGPSPlus::courseTo(
      presY,
      presX,
      tgtY, 
      tgtX);
  
  // calculate ground distance to target
  double groundDistance =
    TinyGPSPlus::distanceBetween(
      presY,
      presX,
      tgtY, 
      tgtX);

  // correct distance for altitude
  float alt = presElev - tgtElev;
  double distance = sqrt(sq(groundDistance/1000)+sq(alt/1000))*1000L;

  // calculating delta:
  delta = tgtHdg - heading;
  if (delta >= 180)
  {
    delta -= 360;
  }
  else if (delta <= -180)
  {
    delta += 360;
  }

  // calculating alpha:
  //int alpha = ver/2;
  alpha = acos(alt/distance);
  alpha = alpha * 180/M_PI - 90;
  alphaS = map(alpha, -90, 90, 0, ver); // make that it fits on screen for now
  
}

void drawHeading(int heading)
{

  // calculate offset lines from center:
  int remainder = heading % 5;
  int offset = 5 - remainder;
  offset = map(remainder, 0, 5, 0, spaces); // offset changed to pixels
  
  // midle heading:
  display.setCursor(mid-9, ver-6);
  if (heading < 10)
  {
    display.print("00"); display.print(heading);
  }
  else if (heading < 100)
  {
    display.print('0'); display.print(heading);
  }
  else
  {
    display.print(heading);
  }
  
  // midle line:
  display.drawLine(mid, (ver-12), mid, (ver-15), WHITE);
  
  // side lines:
  for (int i = 0; i < lineAmount; i++)
  {
    int x = (origin+(spaces*i)) - offset;
    int y = ver - 8;
    if (x >= hor)
    {
      break;
    }
    display.drawLine(x, y, x, y-2, WHITE);

    //Write side headings
    x = x-6;
    y = ver-6;
    if ((x>0 && x < mid-39) || (x > mid+12 && x < mid+46)) // write only if not overlaps
    {
      int output = (heading-remainder)-(15)+(5*i); //Doesn't work with chaged FOV
      if (output < 0)
      {
        output = 360 + output;
      }
      else if (output > 360)
      {
        output = output - 360;
      }
      
      display.setCursor(x, y);
      if (output < 10)
      {
        display.print("00"); display.print(output);
        //TV.print(x+8, y, output);
      }
      else if (output < 100)
      {
        display.print('0'); display.print(output);
        //TV.print(x+4, y, output);
      }
      else
      {
        display.print(output);
      }
      //Serial.print("Output: "); Serial.println(output);
    }
    
  }
}

void drawCircle()
{

  if (delta < 15 && delta > -15)
  {
    display.drawCircle(mid + (pxInDegree*delta), alphaS, 8, WHITE);
  }
  if (delta > 15 && alpha > 0)
  {
    int x = 360 - alpha;
    x = x * M_PI / 180;
    display.drawLine(mid, ver/2, (5*cos(x)+mid), 5*sin(x)+(ver/2), WHITE);
  }
  else if (delta > 15 && alpha <= 0)
  {
    int x = 0 - alpha;
    x = x * M_PI / 180;
    display.drawLine(mid, ver/2, (5*cos(x)+mid), 5*sin(x)+(ver/2), WHITE);
  }
  else if (delta < -15)
  {
    int x = 180 + alpha;
    x = x * M_PI / 180;
    display.drawLine(mid, ver/2, (5*cos(x)+mid), 5*sin(x)+(ver/2), WHITE);
  }
  
}
