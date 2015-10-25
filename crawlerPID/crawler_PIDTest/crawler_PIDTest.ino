#include <Servo.h>
#include <PID_v1.h>

double Setpoint, Input, Output;
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


 
Servo wheels; // servo for turning the wheels
Servo esc; // not actually a servo, but controlled like one!
bool startup = true; // used to ensure startup only happens once
int startupDelay = 1000; // time to pause at each calibration step
double maxSpeedOffset = 45; // maximum speed magnitude, in servo 'degrees'
double maxWheelOffset = 85; // maximum wheel turn magnitude, in servo 'degrees'

double wheelOffset = 0.0; // For Adjusting the wheel
double threshHoldDistance = 20.0;

int pin_head = 0;
int pin_tail = 3;

double steerAngle = 0.0;
double car_length = 13;
bool goForward = true;
//double wheelStartUpOffset = 0.0; // For Adjusting the steering


int count = 3;
double distance_sum = 0.0;
double head_sum = 0.0;
double tail_sum = 0.0;
 
void setup()
{
  wheels.attach(8); // initialize wheel servo to Digital IO Pin #8
  esc.attach(9); // initialize ESC to Digital IO Pin #9
  /*  If you're re-uploading code via USB while leaving the ESC powered on, 
   *  you don't need to re-calibrate each time, and you can comment this part out.
   */
  calibrateESC();

    //initialize the variables we're linked to
  Input = getHeadDis() - getTailDis();
  Setpoint = 0.0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);


  

  Serial.begin(9600);
}

double getHeadDis() {
  double A1 = (double)analogRead(pin_head);
  double distance_head = exp(8.5841-log(A1));
  return distance_head;
}

double getTailDis() {
  double A2 = (double)analogRead(pin_tail);
  double distance_tail = exp(8.5841-log(A2));
  return distance_tail;
}

boolean compareHeadTail(double head, double tail) {
    if (abs(head-tail) < 2) {
      if (head < 0.8 * threshHoldDistance || head > 1.2 * threshHoldDistance) {
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


void steerTheCar(double dis) {
  double temp = abs(threshHoldDistance * threshHoldDistance - dis);
  if (temp == 0.0) {
    return;
  }
  if (dis < threshHoldDistance * threshHoldDistance) {
    if (temp > 0.4 * threshHoldDistance * threshHoldDistance) {
      steerLeft(0.6);
    }
    else {
      steerLeft(0.6 * temp / (threshHoldDistance * threshHoldDistance) / 0.4);
    }
  } else {
    if (temp > 0.4 * threshHoldDistance * threshHoldDistance) {
      steerRight(0.6);
    } 
    else {
      steerRight(0.6 * temp / (threshHoldDistance * threshHoldDistance) / 0.4);
    }
  } 
}

void steerLeft(double d)
{ 
  Serial.write("Steer Left:");
  Serial.write("\n");
  
  if( (d >= 0.0 ) && (d <= 1.0))
  {
    double temp = min( (d * maxWheelOffset + wheelOffset), maxWheelOffset);
    
    wheels.write(90 + temp);
  }
}

void steerRight(double d)
{
  Serial.write("Steer Right:");
  Serial.write("\n");
  
  if( (d >= 0.0 ) && (d <= 1.0))
  {
    double temp = min( (d * maxWheelOffset + wheelOffset), maxWheelOffset);
    Serial.println("temp :  "+ (String)temp);
    
    wheels.write(90 - temp);
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
   if (count > 0) {
//      Serial.println((String)count+"  "+(String)distance_sum);
//      distance_sum += calcDistance();
      head_sum += getHeadDis();
      tail_sum += getTailDis();
      if (compareHeadTail(head_sum, tail_sum)) {


          Input = getHeadDis() - getTailDis();
          myPID.Compute(); 
          steerLeft(output);
      }
      count--; 
   } else {
      head_sum /= 3;
      tail_sum /= 3;
      Serial.println("head:" + (String)head_sum + "   tail:   "+ (String)tail_sum);
      if (!compareHeadTail(head_sum, tail_sum)) {
        Serial.println("Steer the car");
        steerTheCar(calcDistance(head_sum, tail_sum));  
      }
      head_sum = 0;
      tail_sum = 0;
      count = 3;
   }
   setVelocity(0.3);
//   steerTheCar(calcDistance());
   delay(100);
}



