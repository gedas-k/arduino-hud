#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <ResponsiveAnalogRead.h> // filter
#include <TinyGPS++.h>

// If using software SPI (the default case):
#define OLED_MOSI   6
#define OLED_CLK   3
#define OLED_DC    8
#define OLED_CS    11
#define OLED_RESET 7
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

ResponsiveAnalogRead analog(A0, false); // filter initiation
float snap=0.01;
void setSnapMultiplier(float snap); // filter responsivity 0-1 default 0.01

//Variables:
int fov = 20; // field of view

unsigned char hor;
unsigned char ver;
unsigned char mid;
unsigned char pxInDegree;
unsigned char lineAmount;
unsigned char spaces;
unsigned char origin;
char charLine[] = "000000000"; // HHHAAABBB
float info[] = {000, 000, 000, 000}; // heading, alpha, beta, yaw
double targetPosition[] = {23.312558, 55.928551, 100}; // longitude (X), latitude (Y), elevation
double toTarget[] = {0, 0, 0}; // delta to target, alpha to target, distance to target
bool addA = true; // for testing alpha
bool addB = true; // for testing beta
bool addY = true; // for testing yaw
bool filter = true; // is filter ON

void setup() {
  Serial.begin(19200);
  
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done
  
  // setup text properties
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  hor = display.width() - 1;
  ver = display.height() - 1;
  mid = hor / 2;
  pxInDegree = hor / fov; // how many pixels in one degree
  lineAmount = (fov / 5) + 1; // how many side lines on screen
  spaces = 5 * pxInDegree; // spaces betwean lines
  origin = mid - ((fov/10) * spaces); // where starts lines

}

void loop() {

  //Serial.println("Loop");
  display.clearDisplay(); // clear the buffer

  handleSerial();
  getHeading();
  getOrentation();
  getToTarget();
  int heading = info[0];
  int alpha = info[1];
  int beta = info[2];
  int yaw = info[3];
  display.setCursor(0, 0);
  display.print("Heading: "); display.println(heading);
  display.print("Alpha: "); display.println(alpha);
  display.print("Beta: "); display.println(beta);
  //display.print("Yaw: "); display.println(yaw);
 /* Serial.print("Heading: "); Serial.println(heading);
  Serial.print("Alpha: "); Serial.println(alpha);
  Serial.print("Beta: "); Serial.println(beta);
    */

  drawCross();
  drawHorizon();
  drawHeading();
  drawTarget();

  display.display(); // drawing everything
  //delay(1000);

}

void handleSerial() {
  //Serial.println("Inicijuojama funkcija");
  if (Serial.available() > 0) {
    //Serial.println("Pradedama while");
    char incomingCharacter = Serial.read();
    display.print("Nu simbolis: "); display.println(incomingCharacter);
    delay(500);
    switch (incomingCharacter) {
      case 'H':
      {
          //Serial.println("Pradedama case H");
          int i = 0;
          while (Serial.available() > 0) {
            incomingCharacter = Serial.read();
            charLine[i] = incomingCharacter;
            //Serial.print("Nuskaitytas naujas simbolis H: "); Serial.println(incomingCharacter);
            i++;
          }
      }
        break;

      case 'A':
      {
          //Serial.println("Pradedama case A");
          int i = 3;
          while (Serial.available() > 0) {
            incomingCharacter = Serial.read();
            charLine[i] = incomingCharacter;
            //Serial.print("Nuskaitytas naujas simbolis A: "); Serial.println(incomingCharacter);
            i++;
          }
      }
        break;

      case 'B':
      {
          //Serial.println("Pradedama case B");
          int i = 6;
          while (Serial.available() > 0) {
            incomingCharacter = Serial.read();
            charLine[i] = incomingCharacter;
            //Serial.print("Nuskaitytas naujas simbolis B: "); Serial.println(incomingCharacter);
            i++;
          }
      }
        break;
   /*  case ‘-’:
      pwmValue = pwmValue - 5;
      If (pwmValue <= 0)
         pwmValue = 0;
      break;*/
    }
 }
}

void drawCross()
{
  int outer = 7; // outer circle
  int inner = 3; // inner circle
  int center = ver/2;
  display.drawLine(mid-inner, center, mid-outer, center, WHITE); // horizontal 1
  display.drawLine(mid+inner, center, mid+outer, center, WHITE); // horizontal 2
  display.drawLine(mid, center-inner, mid, center-outer, WHITE); // vertical 1
  display.drawLine(mid, center+inner, mid, center+outer, WHITE); // vertical 1
}

void drawHeading()
{

  int heading = info[0];
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
    if ((x>0 && x < mid-30) || (x > mid+12 && x < mid+46)) // write only if not overlaps heading
    {
      int output = (heading-remainder)-(fov/2)+(5*i);
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

void drawHorizon()
{

  int alpha = info[1];
  float beta = info[2];
  int yaw = info[3];
  double distCrossToTarget = alpha * pxInDegree;
  
  if (beta == 90 || beta == -90)
  {
    int x1 = mid - alpha*distCrossToTarget;
    int y1 = 0;
    int x2 = mid - alpha*distCrossToTarget;
    int y2 = ver;
    display.drawLine(x1, y1, x2, y2, WHITE);
  }
  else
  {
    beta = beta * M_PI / 180;  // convert beta to rad
    int x1 = 0;
    int y1 = tan(beta)*(x1-mid + sin(beta)*distCrossToTarget) + (ver/2 + cos(beta)*distCrossToTarget);
    int x2 = hor;
    int y2 = tan(beta)*(x2-mid + sin(beta)*distCrossToTarget) + (ver/2 + cos(beta)*distCrossToTarget);
    display.drawLine(x1, y1, x2, y2, WHITE);
  }
  
}

void drawTarget()
{

  float beta = info[2];
  beta = beta * M_PI/180; // convert to radians
  
  int deltaTgt = toTarget[0];
  int alfaTgt = toTarget[1] - info[1]; // change alfa for head possition alpha
  //alfaTgt = alfaTgt / cos(beta); // change alfa for head possition beta

  double distCrossToTarget = sqrt(sq(alfaTgt) + sq(deltaTgt)) * pxInDegree;
  double teta = atan2(deltaTgt, alfaTgt); // angle from vertical IRL to target
  float offAngle = teta + beta; // angle from Y axis on display to target

  int x = mid + (distCrossToTarget * sin(offAngle));
  //float x = mid + deltaTgt*pxInDegree*cos(beta);
  int y = ver/2 - (distCrossToTarget * cos(offAngle));
  //float y = ver/2 + alfaTgt + pxInDegree*deltaTgt*sin(beta);

  display.drawCircle(x, y, 8, WHITE);

  /*
  if (delta < 15 && delta > -15)
  {
    display.drawCircle(mid + (pxInDegree*delta), alfa, 8, WHITE);
  }
  if (delta > 15 && alfa > 0)
  {
    drawArrow(360 - alfa);
  }
  else if (delta > 15 && alfa <= 0)
  {
    drawArrow(0 - alfa);
  }
  else if (delta < -15)
  {
    drawArrow(180 + alfa);
  }*/
}

void drawArrow(float x)
{
  x = x * M_PI / 180;
  display.drawLine(mid, ver/2, (5*cos(x)+mid), 5*sin(x)+(ver/2), WHITE);
}

void getHeading()
{  

  int sensorHdg = analogRead(A0);
  int heading = map(sensorHdg, 20, 1000, 0, 359);
  if (filter == false)
  {
    info[0] = heading;
  }
  else if (filter == true)
  {
    info[0] = filterHdg(heading);
  }
  
  
  /*
  char hdg[] = "000";
  for (int i=0; i<3; i++)
  {
    hdg[i] = charLine[i];
  }
  int heading = atoi(hdg);
  info[0] = heading;*/
}

int filterHdg(int heading)
{
  analog.update(heading);
  return analog.getValue();
}

void getOrentation()
{

  float sensorAlpha = analogRead(A1);
  info[1] = map(sensorAlpha, 20, 1000, -90, 90);

  float sensorBeta = analogRead(A5);
  info[2] = map(sensorBeta, 20, 1000, -90, 90);
  
  /*
  // Alpha
  if (addA == true)
  {
    int alpha = info[1] + 1;
    info[1] = alpha;
  }
  else if (addA == false)
  {
    int alpha = info[1] - 1;
    info[1] = alpha;
  } 
  if (info[1] == 10)
  {
    addA = false;
  }
  if (info[1] == -10)
  {
    addA = true;
  }

  // Beta
  if (addB == true)
  {
    int beta = info[2] + 1;
    info[2] = beta;
  }
  else if (addB == false)
  {
    int beta = info[2] - 1;
    info[2] = beta;
  }  
  if (info[2] == 90)
  {
    addB = false;
  }
  if (info[2] == -90)
  {
    addB = true;
  }

  // Yaw
  if (addY == true)
  {
    int yaw = info[3] + 1;
    info[3] = yaw;
  }
  else if (addY == false)
  {
    int yaw = info[3] - 1;
    info[3] = yaw;
  }  
  if (info[3] == 20)
  {
    addY = false;
  }
  if (info[3] == -20)
  {
    addY = true;
  }
  */
  
/*  char al[] = "000";
  for (int i=0; i<3; i++)
  {
    al[i] = charLine[i+3];
  }
  int alpha = atoi(al);
  info[1] = alpha;

  char be[] = "000";
  for (int i=0; i<3; i++)
  {
    be[i] = charLine[i+6];
  }
  int beta = atoi(be);
  info[2] = beta;*/
}

void getToTarget()
{
  // Pressent position:
  //double presX = gps.location.lng(); UNCOMENT FOR GPS!!!!!!!!!!!!
  //double presY = gps.location.lat();
  double presX = 23.318922;
  double presY = 55.925182;
  float presElev = 10; // present elevation

  double groundDistance =           // ground distance to target
    TinyGPSPlus::distanceBetween(
      presY,
      presX,
      targetPosition[1], 
      targetPosition[0]);

  float elevation = presElev - targetPosition[2]; // elevation difference
  double distance = sqrt(sq(groundDistance/1000)+sq(elevation/1000))*1000L; // corrected distance for altitude

  double tgtHdg =                   // heading to target
    TinyGPSPlus::courseTo(
      presY,
      presX,
      targetPosition[1], 
      targetPosition[0]);

  // calculating delta from heading to target heading:
  int delta = tgtHdg - info[0];
  if (delta >= 180)
  {
    delta -= 360;
  }
  else if (delta <= -180)
  {
    delta += 360;
  }

  // calculating alfa for altitude to target:
  float alfa = acos(elevation/distance);
  alfa = alfa * 180/M_PI - 90;
  //int alfa = map(alfa, -90, 90, 0, ver); //make that it fits on screen for now

  // add everything to array:
  toTarget[0] = delta;
  toTarget[1] = alfa;
  toTarget[2] = distance;

}

