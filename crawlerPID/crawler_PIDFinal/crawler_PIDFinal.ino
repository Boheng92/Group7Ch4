#include <Servo.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>

//create an xBee object
//SoftwareSerial xbee(2,3); // Rx, Tx


double Setpoint, Input, Output;
double Kp=3.0, Ki=0.00001, Kd=1.2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
 
Servo wheels; // servo for turning the wheels
Servo esc; // not actually a servo, but controlled like one!
bool startup = true; // used to ensure startup only happens once
int startupDelay = 1000; // time to pause at each calibration step
double maxSpeedOffset = 45; // maximum speed magnitude, in servo 'degrees'
//double maxWheelOffset = 85; // maximum wheel turn magnitude, in servo 'degrees'
double maxWheelOffset = 40; // maximum wheel turn magnitude, in servo 'degrees'

double wheelOffset = 0.0; // For Adjusting the wheel
double threshHoldDistance = 27.5 * 27.5;
//double threshHoldDistance = 69.85 * 69.85;

int pin_head = 0;
int pin_tail = 3;

double steerAngle = 0.0;
double car_length = 13;         //inches
//double car_length = 15;       //cm
//double wheelStartUpOffset = 0.0; // For Adjusting the steering

void setup()
{
//  xbee.begin(9600);
  Serial.begin(9600);
  wheels.attach(8); // initialize wheel servo to Digital IO Pin #8
  esc.attach(9); // initialize ESC to Digital IO Pin #9
  /*  If you're re-uploading code via USB while leaving the ESC powered on, 
   *  you don't need to re-calibrate each time, and you can comment this part out.
   */
  calibrateESC();
    //initialize the variables we're linked to
  Input = calcDistance(getHeadDis(), getTailDis());
  Setpoint = threshHoldDistance;

  //turn the PID on
  myPID.SetOutputLimits(-0.5,0.5);
  myPID.SetMode(AUTOMATIC);

  
  setVelocity(0.3);
}


double getHeadDis() {
  double A1 = (double)analogRead(pin_head);
  double distance_head = exp(8.5841-log(A1));
//  Serial.println("head: "+ (String)distance_head);
  return distance_head;
}

double getTailDis() {
  double A2 = (double)analogRead(pin_tail);
  double distance_tail = exp(8.5841-log(A2));
//  Serial.println("tail: "+ (String)distance_tail);
  return distance_tail;
}

boolean compareHeadTail(double head, double tail) {
    if (abs(head-tail) < 2) {
      if (head < 0.7 * threshHoldDistance || head > 1.3 * threshHoldDistance) {
        return false;
      } 
      else {
        return true;
      }
    } 
    else {
      return false;
    }
}

boolean detectWall() {
    double distance = (double)analogRead(5) / 2;
    Serial.println(distance);
     if (distance < 15) {
      return true;  
    } else {
      return false;
    }
}


double calcDistance(double head, double tail) {
  double distance_head = head;
  double distance_tail = tail;
//  Serial.println("head: " + (String)distance_head);
//  Serial.println("tail: " + (String)distance_tail);
  double distance = (car_length * car_length * distance_head * distance_head) / (car_length * car_length + (distance_head - distance_tail) * (distance_head - distance_tail));
//  Serial.println("distance:  " + (String)distance);
  return distance;
}

/* Calibrate the ESC by sending a high signal, then a low, then middle.*/
void calibrateESC(){
    esc.write(180); // full backwards
    delay(startupDelay);
    esc.write(0); // full forwards
    delay(startupDelay);
    esc.write(90); // neutral
    delay(startupDelay);
    esc.write(90); // reset the ESC to neutral (non-moving) value
}

void steerLeft(double d)
{ 
//  Serial.write("Steer Left:");
//  Serial.write("\n");
  
  if( (d >= 0.0 ) && (d <= 1.0))
  {
    double temp = min( (d * maxWheelOffset + wheelOffset), maxWheelOffset);
    
    wheels.write(70 + temp);
  }
}

void steerRight(double d)
{
//  Serial.write("Steer Right:");
//  Serial.write("\n");
  
  if( (d >= 0.0 ) && (d <= 1.0))
  {
    double temp = min( (d * maxWheelOffset + wheelOffset), maxWheelOffset);
//    Serial.println("temp :  "+ (String)temp);
    
    wheels.write(70 - temp);
  }
}
/*
  Set the velocity of the car. Control the back and forward directions.
  Input s > 0 will go forward, and s < 0 will go backward.
  The input should between (-1 , 1).
  
*/
void setVelocity(double s)
{
  if( (s >= -1.0 ) && (s <= 1.0))
  {
    esc.write(90 - (s * maxSpeedOffset));
  }
}


void loop()
{
//  if (xbee.available() > 0) {
//    String msg  = "";
//    while(xbee.available() > 0) {
//      msg += char(xbee.read());
//    }
//    if (msg.equals("START\n")) {
//      Serial.println(msg);
//      setVelocity(0.35);                          
//    
//    } else if (msg.equals("STOP\n")) {
//      Serial.println(msg);
//      setVelocity(0.0);
//    }
//  } else {
//      double head_dis = getHeadDis();
//      double tail_dis = getTailDis();
//      Serial.println("head_dis: " + (String)head_dis + "   tail_dis:  "+ (String)tail_dis);
//      Input = calcDistance(getHeadDis(), getTailDis());
//      myPID.Compute();
//      Serial.println(Output);
//      xbee.println(Output);
//      steer(Output); 
//      if (Output < 0) {
//        steerRight(-Output);
//      } else {
//        steerLeft(Output);
//      }
//  }
//delay(100);
  int count = 0;
  double head_dis = getHeadDis();
  double tail_dis = getTailDis();
  if (head_dis < 80 && tail_dis < 80) {
    Serial.println("head_dis: " + (String)head_dis + "   tail_dis:  "+ (String)tail_dis);
    Input = calcDistance(getHeadDis(), getTailDis());
    myPID.Compute();
    if (Output < 0) {
       steerRight(-Output);
    } else {
       steerLeft(Output);
    }
  } else {
    steerRight(0.1);  
  }
  if (detectWall()) {
    count++;
    if (count == 3) {
      setVelocity(0.0);
    }
  } else{
    count = 0;  
  }
  delay(50);
}



