#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <TVout.h>
#include <TVoutfonts/fontALL.h>

//Variables:
int FOV = 30; // field of view

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

TVout TV;
//unsigned char x,y;

unsigned char hor;
unsigned char ver;
unsigned char mid;
unsigned char pxInDegree;
unsigned char lineAmount;
unsigned char spaces;
unsigned char origin;
//char hdg[3] = {0, 1, 5};

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

}

void loop() {

  /* Get a new sensor event */
  sensors_event_t event; 
  mag.getEvent(&event);  

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
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
  // int compass = heading * 180/M_PI; UNCOMENT//////////////////////////
  
  //COMENT:
  int sensorHdg = analogRead(A0);
  int compass = map(sensorHdg, 0, 1023, 0, 359);
 /* if (compass >= 360)
  {
    compass = 360 - compass;
  }*/

  //Serial.print("Compass: "); Serial.println(compass);
  /*
  if (compass < 10)
  {
    Serial.print("Compass: "); Serial.print("00"); Serial.print(compass);
  }
  else if (compass < 100)
  {
   Serial.print("Compass: "); Serial.print('0'); Serial.print(compass);
  }
  else
  {
    Serial.print("Compass: "); Serial.println(compass);
  }
*/
  //Serial.print("Compass: "); Serial.println(compass);
  
  //For later use
  /*
  //Heading selection with Dial
  int hdg=analogRead(A0);
  hdg = map(sensorHdg, 0, 1023, 0, 360);

  //Nuokrypio skaiciavimas:
  int delta = hdg - compass;
  if (delta > 180)
  {
    delta = delta - 360;
  }
  else if (delta < -180)
  {
    delta = delta + 360;
  }
  */
  
  // Calculate offset from center:
  int remainder = compass % 5;
  int offset = 5 - remainder;
   offset = map(remainder, 1, 5, 0, spaces); // offset changed to pixels
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
      //TV.print(x, y, output);
      //Serial.print("Output: "); Serial.println(output);
    }
    
  }
  
  /*
  int sensorHdg=analogRead(A0);
  int x = map(sensorHdg, 0, 1023, 0, 127);
  TV.clear_screen();
  int y=0;
  TV.draw_line(64,63,x,y,1);
  //TV.delay_frame(1);
  */
  TV.delay_frame(1);
}

