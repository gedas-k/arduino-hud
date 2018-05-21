#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <TVout.h>
#include <TVoutfonts/fontALL.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//Variables:
int FOV = 30; // field of view
// Pressent position:
double presX = 23.318922;
double presY = 55.925182;
int presElev = 200; // altitude in meters
// Target position:
double tgtX = 23.317962;
double tgtY = 55.925614;
int tgtElev = 100; // target elevation in meters
bool smoot = true; // is smooting on

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// The TinyGPS++ object
TinyGPSPlus gps;

TVout TV;
//unsigned char x,y;

unsigned char hor;
unsigned char ver;
unsigned char mid;
unsigned char pxInDegree;
unsigned char lineAmount;
unsigned char spaces;
unsigned char origin;

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
  
  TV.begin(_NTSC,128,64);
  TV.select_font(font4x6);
  hor = TV.hres() - 1;
  ver = TV.vres() - 1;
  mid = hor / 2;
  pxInDegree = hor / FOV; // how many pixels in one degree
  lineAmount = (FOV / 5) + 1; // how many lines on screen
  spaces = 5 * pxInDegree; // spaces betwean lines
  origin = mid - ((FOV/10) * spaces); // where starts lines

  //Serial.print("Spaces: "); Serial.println(spaces);
  
  // initialize all the readings to 0 for smoothing:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

}

void loop() {

  /* Get a new sensor event */
  sensors_event_t event; 
  mag.getEvent(&event);  

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = 0;
  if (smoot == true)
  {
    // subtract the last reading:
    sum = sum - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = atan2(event.magnetic.y, event.magnetic.x);;
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
  int compass = heading * 180/M_PI; // UNCOMENT//////////////////////////
  
  //COMENT:
/*  int sensorHdg = analogRead(A0);
  int compass = map(sensorHdg, 0, 1023, 0, 359);
  heading = compass*M_PI/180;*/
 /* if (compass >= 360)
  {
    compass = 360 - compass;
  }*/

  Serial.print("Compass: "); Serial.println(compass);
  
  // Calculate offset from center:
  int remainder = compass % 5;
  int offset = 5 - remainder;
  offset = map(remainder, 0, 5, 0, spaces); // offset changed to pixels
  //Serial.print("Offset: "); Serial.println(offset);

  //Drawing stuff:
  TV.clear_screen();
  
  //Midle heading:
  if (compass < 10)
  {
    TV.print(mid-5, ver-5, "00");
    TV.print(mid+3, ver-5, compass);
  }
  else if (compass < 100)
  {
    TV.print(mid-5, ver-5, '0');
    TV.print(mid-1, ver-5, compass);
  }
  else
  {
    TV.print(mid-5, ver-5, compass);
  }
  
  //TV.print(mid-5, ver-5, compass);  
  TV.draw_line(mid, (ver-12), mid, (ver-15), WHITE);

  //Side lines:
  for (int i = 0; i < lineAmount; i++)
  {
    int x = (origin+(spaces*i)) - offset;
    int y = ver - 6;
    if (x >= hor)
    {
      break;
    }
    TV.draw_line(x, y, x, y-5, WHITE);

    //Write side headings
    x = x-6;
    y = ver-5;
    if ((x>0 && x < mid-21) || (x > mid+9 && x < mid+51)) // write only if not overlaps
    {
      int output = (compass-remainder)-(15)+(5*i); //Don't work with chaged FOV
      if (output < 0)
      {
        output = 360 + output;
      }
      else if (output > 360)
      {
        output = output - 360;
      }
      
      if (output < 10)
      {
        TV.print(x, y, "00");
        TV.print(x+8, y, output);
      }
      else if (output < 100)
      {
        TV.print(x, y, '0');
        TV.print(x+4, y, output);
      }
      else
      {
        TV.print(x, y, output);
      }
      //Serial.print("Output: "); Serial.println(output);
    }
    
  }

  //Calculate and draw target circle:
  //Calculate target heading and distance:
  
  //Change altitude with pot for testing:
  presElev = analogRead(A0);
  presElev = map(presElev, 0, 1023, 0, 2000);
  
  //double presX = gps.location.lng(); UNCOMENT FOR GPS!!!!!!!!!!!!
  //double presY = gps.location.lat();
  double groundDistance =
    TinyGPSPlus::distanceBetween(
      presY,
      presX,
      tgtY, 
      tgtX);
  
  float alt = presElev - tgtElev;
  double distance = sqrt(sq(groundDistance/1000)+sq(alt/1000))*1000L; // corrected for altitude
  
  double tgtHdg =
    TinyGPSPlus::courseTo(
      presY,
      presX,
      tgtY, 
      tgtX);
      
  //Serial.print("Altitude                               : "); Serial.println(alt);
  //Serial.print("Air distance: "); Serial.println(distance);
  //Serial.print("Target heading: "); Serial.println(tgtHdg);
  //Serial.print("Ground distance: "); Serial.println(groundDistance);
  

  //Calculating delta:
  int delta = tgtHdg - compass;
  if (delta >= 180)
  {
    delta -= 360;
  }
  else if (delta <= -180)
  {
    delta += 360;
  }

  //Calculating alfa:
  //int alfa = ver/2;
  float alfa = acos(alt/distance);
  alfa = alfa * 180/M_PI - 90;
  int alfaS = map(alfa, -90, 90, 0, ver); //make that it fits on screen for now
  //Serial.print("Alfa: "); Serial.println(alfa);
  
  //Drawing circle:
  if (delta < 15 && delta > -15)
  {
    TV.draw_circle(mid + (pxInDegree*delta), alfaS, 8, WHITE);
  }
  if (delta > 15 && alfa > 0)
  {
    draw_arrow(360 - alfa);
  }
  else if (delta > 15 && alfa <= 0)
  {
    draw_arrow(0 - alfa);
  }
  else if (delta < -15)
  {
    draw_arrow(180 + alfa);
  }
  
  
  TV.delay_frame(1);
}

void draw_arrow(float x) //Change to degreese
{
  x = x * M_PI / 180;
  TV.draw_line(mid, ver/2, (5*cos(x)+mid), 5*sin(x)+(ver/2), WHITE);
}
