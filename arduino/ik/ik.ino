//------------------------------------------------------------------------------
// R2B2 - Reconfigurable Robotic Button Basher
// Modifications - justin@isecpartners.com, Paul Vines
// Original - dan@marginallycelver.com 2011-06-21
//------------------------------------------------------------------------------
// Copyright at end of file.



#include "Vector3.h"
#include "Joint.h"
#include "Arm.h"
#include "DeltaRobot.h"
#include <Servo.h>
#include "point.h"

//------------------------------------------------------------------------------
// constants
//------------------------------------------------------------------------------

// Serial communication bitrate
const long BAUD        = 57600;
// Maximum length of serial input message.
const int MAX_BUF      = 64;

#define TWOPI            (PI*2.0f)
#define DEG2RAD          (PI/180.0f)
#define RAD2DEG          (180.0f/PI)
#define DELAY            (5)

#define ACHAR            (97)

// how far should we subdivide arcs into line segments?
#define CM_PER_SEGMENT   (0.50)
#define MIN_FEED_RATE    (0.01)  // cm/s

const int MAX_ANGLE    = 90+35;
const int MIN_ANGLE    = 90-80;
static const float shoulder_to_elbow  = 5;  // cm
//static const float elbow_to_wrist     = 18.5f;  // cm
static const float elbow_to_wrist     = 17.6f;  // cm


static const float effector_to_wrist  = 1.59258f;  //cm
static const float center_to_shoulder = 5.753f;  // cm
//static const float effector_to_wrist = 2.0f;


static const int time_on_button=300; //ms

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// globals
//------------------------------------------------------------------------------
long start;  // clock time since start
long last_frame, this_frame;  // for finding dt
double dt;

char buffer[MAX_BUF];
int sofar;

DeltaRobot robot;
float feed_rate=10;  // how fast the tool moves in cm/s

float liftvalue=0;
float dropvalue=-2;

int reverse=0;

#define LISTSIZE 10
Point pointlist[LISTSIZE];
Point zeropoint;


//------------------------------------------------------------------------------
// methods
//------------------------------------------------------------------------------

/**
  * finds angle of dy/dx as a value from 0...2PI
  * @return the angle
  */
float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


/**
 * setup the geometry of the robot for faster inverse kinematics later
 */
void setup_robot() {
  Vector3 temp,n;
  float aa,bb,cc;
  int i;

  for(i=0;i<NUM_ARMS;++i) {
    Arm &a=robot.arms[i];
    // shoulder
    a.shoulder.pos=Vector3(cos(TWOPI*i/NUM_ARMS)*center_to_shoulder,  
                           sin(TWOPI*i/NUM_ARMS)*center_to_shoulder,
                           0);
    // elbow
    a.elbow.pos=Vector3(cos(TWOPI*i/NUM_ARMS)*(center_to_shoulder+shoulder_to_elbow),
                        sin(TWOPI*i/NUM_ARMS)*(center_to_shoulder+shoulder_to_elbow),
                        0);
    // Find wrist position.  This is a special case of forward kinematics.
    n=a.shoulder.pos;
    n.Normalize();
    temp=a.shoulder.pos-n*(center_to_shoulder-effector_to_wrist);
    aa=(a.elbow.pos-temp).Length();
    cc=elbow_to_wrist;
    bb=sqrt((cc*cc)-(aa*aa));
    a.wrist.pos=temp+Vector3(0,0,bb);

    a.elbow.relative=a.elbow.pos;
    a.wrist.relative=a.wrist.pos;
    a.wrist.relative.z=0;
    
    // connect to the servos
    robot.arms[i].s.attach(5-i);
    // center the arm
    robot.arms[i].s.writeMicroseconds(1500);
  }
  robot.default_height=bb;
  robot.ee.pos.z=bb;
}


/**
 * can the tool reach a given point?
 * @param test the point to test
 * @return 1=out of bounds (fail), 0=in bounds (pass)
 */
char outOfBounds(float x,float y,float z) {
  // test if the move is impossible
  int error=0,i;
  Vector3 w, test(x,y,z);
  float len;
  for(i=0;i<NUM_ARMS;++i) {
    Arm &arm=robot.arms[i];
    // get wrist position
    w = test + arm.wrist.relative - arm.shoulder.pos;
    len=w.Length() - elbow_to_wrist;

    if(fabs(len) > shoulder_to_elbow) return 1;
  }
  return 0;
}


/**
 * prints ee position
 */
void printEEPosition() {
  Serial.print(robot.ee.pos.x);
  Serial.print(F("\t"));
  Serial.print(robot.ee.pos.y);
  Serial.print(F("\t"));
  Serial.println(robot.ee.pos.z);
}


/**
 * inverse kinematics for each leg.  if you know the wrist, it finds the shoulder angle(s).
 */
void ik() {
  // find wrist positions
  int i;
  for(i=0;i<NUM_ARMS;++i) {
    Arm &arm=robot.arms[i];

    // get wrist position
    arm.wrist.pos = robot.ee.pos + arm.wrist.relative;
    Vector3 w = arm.wrist.pos - arm.shoulder.pos;
    Vector3 n=arm.shoulder.pos;
    n.z=0;
    n.Normalize();
    Vector3 ortho(-n.y,n.x,0);
    ortho.Normalize();

    // get wrist position on plane of bicep
    float a=w | ortho;  // ee' distance
    Vector3 wop = w - ortho * a;
    arm.wop.pos=wop + arm.shoulder.pos;
    
    // use pythagorean theorem to get e'j
    float b=sqrt(elbow_to_wrist*elbow_to_wrist-a*a);
    
    // use intersection of circles to find elbow point (j).
    //a = (r0r0 - r1r1 + d*d ) / (2 d) 
    float r1=b;  // circle 1 centers on e'
    float r0=shoulder_to_elbow;  // circle 0 centers on shoulder
    float d=wop.Length();
    // distance from shoulder to the midpoint between the two possible intersections
    a = ( r0 * r0 - r1 * r1 + d*d ) / ( 2*d );
    // find the midpoint
    n=wop;
    n.Normalize();
    Vector3 temp=arm.shoulder.pos+(n*a);
    // with a and r0 we can find h, the distance from midpoint to intersections.
    float h=sqrt(r0*r0-a*a);
    // get a normal to the line wop in the plane orthogonal to ortho
    Vector3 r = ortho ^ n;
    Vector3 p1 = temp + r * h;
    
    arm.elbow.pos=p1;

    // use atan2 to find theta
    temp=arm.elbow.pos-arm.shoulder.pos;
    float y=temp.z;
    temp.z=0;
    float x=temp.Length();
    
    if( ( arm.elbow.relative | temp ) < 0 ) x=-x;

    float new_angle=atan2(-y,x) * RAD2DEG;
    if (reverse){
      new_angle=-new_angle;
    }
    
    // cap the angle
    if(new_angle>90) new_angle=90;
    //if (new_angle>MAX_ANGLE) new_angle=MAX_ANGLE;
    if(new_angle<-90) new_angle=-90;
    //if(new_angle<MIN_ANGLE) new_angle=MIN_ANGLE;

    // we don't care about elbow angle, but we could find it here if we needed it.
  
    // update servo to match the new IK data
    int nx=((new_angle)*(500.0f/90.0f)) + 1500;

    Serial.print(new_angle);
    Serial.print("\t");
    Serial.print(nx);
    Serial.print("\n");
    
    if(nx>2000) {
      Serial.println("over max");
      nx=2000;
    }
    if(nx<1000) {
      Serial.println("under min");
      nx=1000;
    }
    if(arm.angle!=nx) {
      //arm.s.writeMicroseconds(nx);
      arm.s.write(new_angle+90);
      arm.angle=nx;
    }
  }
}


/**
 * moving the tool in a straight line
 * @param destination x coordinate
 * @param destination y coordinate
 * @param destination z coordinate
 */
void line(float x, float y, float z) {
  if( outOfBounds(x, y, z) ) {
    Serial.println("Out of bounds.");
    return;
  }

  // how long does it take to reach destination at speed feed_rate?
  Vector3 destination(x,y,z);
  Vector3 start = robot.ee.pos;  // keep a copy of start for later in this method
  Vector3 dp = destination - start;  // far do we have to go? 
  float travel_time = dp.Length() / feed_rate;
  travel_time *= 1000; // convert to ms
  
  // save the start time of this move so we can interpolate linearly over time.
  long start_time = millis();
  long time_now = start_time;
/*
  Serial.print(F("length="));
  Serial.println(dp.Length());
  Serial.print(F("time="));
  Serial.println(travel_time);
  Serial.print(F("feed="));
  Serial.println(feed_rate);
  printEEPosition();
  */
  
  // we need some variables in the loop.  Declaring them outside the loop can be more efficient.
  float f;
  // until the interpolation finishes...
  while(time_now - start_time < travel_time) {
    // update the clock
    time_now = millis();
  
    // find the point between destination and start that we've reached.
    // this is linear interpolation
    f = (float)(time_now - start_time) / travel_time;
    robot.ee.pos = dp * f + start;
  
    //printEEPosition();

    // update the inverse kinematics
    ik();
    
    delay(DELAY);
  }
  
  // one last time to make sure we hit right on the money
  robot.ee.pos = destination;
  // update the inverse kinematics
  ik();
}


/**
 * subdivides a line into shorter segments for straighter motion
 * @param destination x coordinate
 * @param destination y coordinate
 * @param destination z coordinate
 */
static void line_safe(float x,float y,float z) {
  // split up long lines to make them straighter?
  float dx=x-robot.ee.pos.x;
  float dy=y-robot.ee.pos.y;

  float len=sqrt(dx*dx+dy*dy);
  
  if(len<=CM_PER_SEGMENT) {
    line(x,y,z);
    return;
  }
  
  // too long!
  long pieces=ceil(len/CM_PER_SEGMENT);
  float x0=robot.ee.pos.x;
  float y0=robot.ee.pos.y;
  float z0=robot.ee.pos.z;
  float a;
  for(int j=0;j<=pieces;++j) {
    a=(float)j/(float)pieces;

    line((x-x0)*a+x0,
         (y-y0)*a+y0,
         (z-z0)*a+z0);
  }
}


/**
 * arcs, in a plane, starting at the current tool position.
 * @param cw 1=clockwise, 0=counterclockwise
 * @param cx center of arc
 * @param cy center of arc
 * @param cz center of arc
 * @param dx end of arc
 * @param dy end of arc
 * @param dz end of arc
 */
void arc(char cw,float cx,float cy,float cz,float x,float y,float z) {
  // get radius
  float dx = robot.ee.pos.x - cx;
  float dy = robot.ee.pos.y - cy;
  float radius = sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float angle1 = atan3(dy,dx);
  float angle2 = atan3(y-cy,x-cx);
  float theta = angle2 - angle1;
  
  if(cw>0 && theta<0) angle2 += TWOPI;
  else if(cw<0 && theta>0) angle1 += TWOPI;
  
  theta = angle2 - angle1;
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = floor( len / CM_PER_SEGMENT );
 
  float nx, ny, nz, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    nz = ( z - robot.ee.pos.z ) * scale + robot.ee.pos.z;
    // send it to the planner
    line(nx,ny,nz);
  }
  
  line(x,y,z);
}


/**
 * displays help message
 */
void help() {
  //Serial.println(F("== DELTA ROBOT - http://github.com/i-make-robots/Delta-Robot/ =="));
  //Serial.println(F("All commands end with a semi-colon."));
  //Serial.println(F("I understand the following Gcode (http://en.wikipedia.org/wiki/G-code):"));
  //Serial.println(F("G00,G01,G02,G03,M114"));
}

/*
*  From text, gets x, y, and z coordinates in gcode format:
*  ex:  " X55.2 Y22.55 Z123"
*   - ptr is a pointer to buffer (global) somewhere before the first space before
*  the coordinates.
*   - targetpoint is an output param.  Pass the point you want data set into here.
*
*/
void getCoordinatesFromText(char* ptr, Point* targetpoint){
  float xx=robot.ee.pos.x;
  float yy=robot.ee.pos.y;
  float zz=robot.ee.pos.z-robot.default_height; //Why subtract this?

  while(ptr && ptr<buffer+sofar) {
    ptr=strchr(ptr,' ')+1;
    switch(*ptr) {
    case 'X': xx=atof(ptr+1);  Serial.print('x'); Serial.println(xx); break;
    case 'Y': yy=atof(ptr+1);  Serial.print('y'); Serial.println(yy); break;
    case 'Z': zz=atof(ptr+1);  Serial.print('z'); Serial.println(zz); break;
    case '&': break;
    default: ptr=0; break;
    }
  }
  //DIRTY HACK:  X axis is reversed somehow, so let's just flip it here
  targetpoint->x=-xx;
  targetpoint->y=yy;
  targetpoint->z=zz;
}

void rawPrintPoint(float xx, float yy, float zz) {
  Serial.print("x: "); Serial.print(xx);
  Serial.print("y: "); Serial.print(yy);
  Serial.print("z: "); Serial.print(zz);
}

void printPoint(Point inpoint) {
  rawPrintPoint(inpoint.x, inpoint.y, inpoint.z);
}

//reads a point into a storage array
//ex:  WRITEPOINT h X123.4 Y567.8 Z0
//the coordinate values would be stored under the character 'h'
void setPoint() {

  //Find the first space
  char* ptr=buffer;
  ptr=strchr(ptr,' ')+1;
  
  //ptr is now the index we're setting. 
  //Grab the coordinates and stick them in the point array
  int pointslot=(*ptr)-ACHAR;
  Serial.print(pointslot);
  if (pointslot >=0 && pointslot < LISTSIZE)
  {
     getCoordinatesFromText(ptr, &(pointlist[pointslot]));
  }
  else Serial.println(-1);
  
  
}

Point retrievePoint() {
  //Find the first space
  char* ptr=buffer;
  ptr=strchr(ptr,' ')+1;
  byte pointslot=(*ptr)-ACHAR;

  if (pointslot >=0 && pointslot < LISTSIZE)
  {
    return pointlist[pointslot];
  }
  else Serial.println(-1);
  return zeropoint;
  
}

void setDrop() {
 //Find the first space
  char* ptr=buffer;
  ptr=strchr(ptr,' ')+1;
  
  dropvalue=atof(ptr);
}

float retrieveDrop() {
  return dropvalue;
}

void setLift() {
 //Find the first space
  char* ptr=buffer;
  ptr=strchr(ptr,' ')+1;
  liftvalue=atof(ptr);
}

float retrieveLift() {
  return liftvalue;
}

void rawMove(float xx, float yy, float zz) {
   if( outOfBounds(xx, yy, zz+robot.default_height) ) {
    Serial.println("Out of bounds.");
    rawPrintPoint(xx,yy,zz);
    return;
  }
  
  robot.ee.pos.x=xx;
  robot.ee.pos.y=yy;
  robot.ee.pos.z=zz+robot.default_height;
  // update the inverse kinematics

  ik();
}

void dropEffector() {
  rawMove(robot.ee.pos.x,
            robot.ee.pos.y,
            dropvalue); //what's the def h for?
}

void liftEffector() {
  rawMove(robot.ee.pos.x,
            robot.ee.pos.y,
            liftvalue); //what's the def h for?
}

void directMove(Point targetpoint){
  rawMove(targetpoint.x,targetpoint.y,targetpoint.z);
  delay(50);
}

void bounceToPoint(Point targetpoint){
  rawMove(targetpoint.x, targetpoint.y, targetpoint.z + 1);
  delay(150);
  rawMove(targetpoint.x, targetpoint.y, targetpoint.z);
  delay(150);
  rawMove(targetpoint.x, targetpoint.y, targetpoint.z + 1);
}

void moveToPoint(Point targetpoint) {
  rawMove(targetpoint.x, targetpoint.y, targetpoint.z + 1);
  delay(200);
  rawMove(targetpoint.x, targetpoint.y, targetpoint.z);  
}

void enterPin(){
  Point temppoint;
  char* ptr=buffer;
  ptr=strchr(ptr,'&');
  
  while (ptr != NULL){
  
    getCoordinatesFromText(ptr + 1, &temppoint);
    
    moveToPoint(temppoint);
    
    delay(500);
    
    ptr=strchr(ptr + 1,'&');
  }
}


void directMoveCommand(){
  //Find the first space
  char* ptr=buffer;
  ptr=strchr(ptr,' ');
  
  Point temppoint;
  getCoordinatesFromText(ptr, &temppoint);
  
  directMove(temppoint);

}

void bounceCommand(){
  //Find the first space
  char* ptr=buffer;
  ptr=strchr(ptr,' ');
  
  Point temppoint;
  getCoordinatesFromText(ptr, &temppoint);
  
  bounceToPoint(temppoint);
}

void moveCommand()
{
  //Find the first space
  char* ptr=buffer;
  ptr=strchr(ptr,' ');
  
  Point temppoint;
  getCoordinatesFromText(ptr, &temppoint);
  
  moveToPoint(temppoint);
  
}

void reverser()
{
  reverse=!reverse;
}


/**
 * process instructions waiting in the serial buffer
 */
void processCommand() {
  if(!strncmp(buffer,"M114",4)) {
    // get position
    printEEPosition();

  
  } else if (! strncmp(buffer, "WP",2)) {
      setPoint();
  } else if (! strncmp(buffer, "RP",2)) {
      printPoint(retrievePoint());
      Serial.println("");
  
  } else if (! strncmp(buffer, "PO",2)) {
      moveToPoint(retrievePoint()); 
    
  } else if (! strncmp(buffer, "WD",2)) {
    setDrop();
  } else if (! strncmp(buffer, "RD",2)) {
    Serial.println(retrieveDrop());
  } else if (! strncmp(buffer, "WL",2)) {
    setLift();
  } else if (! strncmp(buffer, "RL",2)) {
    Serial.println(retrieveLift());
  } else if (! strncmp(buffer, "DR",2)) {
    dropEffector();
  } else if (! strncmp(buffer, "LI",2)) {
    liftEffector();
  } else if (! strncmp(buffer, "PU",2)) {
      //Find the first space
      moveToPoint(retrievePoint());
      delay (time_on_button);
     // dropEffector();
     // delay(time_on_button);
     // liftEffector();
  } else if (! strncmp(buffer, "EP", 2)){
    enterPin();
  
  }else if(!strncmp(buffer,"BM",2)){
    bounceCommand();
  }else if (! strncmp(buffer, "MV",2)) {
    moveCommand();
  } else if(! strncmp(buffer, "DM", 2)){
    directMoveCommand();
  } else if(! strncmp(buffer, "RV",2)){
   reverser(); 
  } else if( !strncmp(buffer,"G00",3) || !strncmp(buffer,"G01",3) ) {
    // line
    float xx=robot.ee.pos.x;
    float yy=robot.ee.pos.y;
    float zz=robot.ee.pos.z-robot.default_height;
    float ff=feed_rate;

    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'X': xx=atof(ptr+1);  Serial.print('x'); Serial.println(xx); break;
      case 'Y': yy=atof(ptr+1);  Serial.print('y'); Serial.println(yy); break;
      case 'Z': zz=atof(ptr+1);  Serial.print('z'); Serial.println(zz); break;
      case 'F': ff=atof(ptr+1);  Serial.print('f'); Serial.println(ff); break;
      default: ptr=0; break;
      }
    }
    
    if(feed_rate < MIN_FEED_RATE) feed_rate=MIN_FEED_RATE;
    feed_rate=ff;    
    line_safe(xx,yy,zz+robot.default_height);
  } else if( !strncmp(buffer,"G02",3) || !strncmp(buffer,"G03",3) ) {
    // arc
    float xx=robot.ee.pos.x,
          yy=robot.ee.pos.y,
          zz=robot.ee.pos.z-robot.default_height,
          aa=robot.ee.pos.x,
          bb=robot.ee.pos.y,
          cc=robot.ee.pos.z-robot.default_height;
    char ww= !strncmp(buffer,"G02",3);
    float ff=feed_rate;

    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'X': xx=atof(ptr+1);  Serial.print('x'); Serial.println(xx); break;
      case 'Y': yy=atof(ptr+1);  Serial.print('y'); Serial.println(yy); break;
      case 'Z': zz=atof(ptr+1);  Serial.print('z'); Serial.println(zz); break;
      case 'A': aa=atof(ptr+1);  Serial.print('a'); Serial.println(aa); break;
      case 'B': bb=atof(ptr+1);  Serial.print('b'); Serial.println(bb); break;
      case 'C': bb=atof(ptr+1);  Serial.print('c'); Serial.println(cc); break;
      case 'F': ff=atof(ptr+1);  Serial.print('f'); Serial.println(ff); break;
      default: ptr=0; break;
      }
    }
    
    if(feed_rate < MIN_FEED_RATE) feed_rate=MIN_FEED_RATE;
    feed_rate=ff;    
    arc(ww,aa,bb,cc+robot.default_height,
           xx,yy,zz+robot.default_height);
  }
}


/**
 * listen for instructions coming from the serial connection.
 * See: http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-serial-monitor/
 */
void listenToSerial() {
  while(Serial.available() > 0) {
    buffer[sofar++]=Serial.read();
    if(buffer[sofar-1]==';') break;  // in case there are multiple instructions
  }
 
  // if we hit a semi-colon, assume end of instruction.
  if(sofar>0 && buffer[sofar-1]==';') {
    buffer[sofar]=0;
    
    // echo confirmation
    Serial.println(buffer);
 
    // do something with the command
    processCommand();
 
    // reset the buffer
    sofar=0;
 
    // echo completion
    Serial.print(F("> "));
  }
}


/**
 * runs once when board turns on/resets.  Initializes variables.
 */
void setup() {
  //set the zero point
  zeropoint.x=0;
  zeropoint.y=0;
  zeropoint.z=0;
  //zero out point array
  for (int i=0;i<LISTSIZE;i++) {
    pointlist[i]=zeropoint;
  }
  // initialize the read buffer
  sofar=0;
  // start serial communications
  Serial.begin(BAUD);
  //Serial.print(F("\n\nHELLO WORLD! I AM A DELTA ROBOT."));
  Serial.println(sizeof(pointlist[0]));

  setup_robot();
  // @TODO: Is this necessary?
  //line(0,0,0);
  
  help();
  
  Serial.print(F("> "));
  start = millis();
}


/**
 * runs over and over.  if loop() finishes then the board calls it again. globals do not get reset if loop() ends.
 */
void loop() {
  listenToSerial();
}


//------------------------------------------------------------------------------
// Copyright (C) 2011 Dan Royer (dan@marginallyclever.com)
// Permission is hereby granted, free of charge, to any person obtaining a 
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation 
// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
// and/or sell copies of the Software, and to permit persons to whom the 
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//------------------------------------------------------------------------------

