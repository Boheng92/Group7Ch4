#include <Servo.h>
 
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
double car_length = 12.5;
bool goForward = true;
//double wheelStartUpOffset = 0.0; // For Adjusting the steering


int count = 10;
double distance_sum = 0.0;
 
void setup()
{
  wheels.attach(8); // initialize wheel servo to Digital IO Pin #8
  esc.attach(9); // initialize ESC to Digital IO Pin #9
  /*  If you're re-uploading code via USB while leaving the ESC powered on, 
   *  you don't need to re-calibrate each time, and you can comment this part out.
   */
  calibrateESC();

  Serial.begin(9600);
}

double calcDistance() {
  double A1 = (double)analogRead(pin_head);
  double A2 = (double)analogRead(pin_tail);
  double distance_head = exp(8.5841-log(A1));
  double distance_tail = exp(8.5841-log(A2));
  Serial.println("head: " + (String)distance_head);
  Serial.println("tail: " + (String)distance_tail);
  double distance = (car_length * car_length * distance_head * distance_head) / (car_length * car_length + (distance_head - distance_tail) * (distance_head - distance_tail));
  Serial.println("distance:  " + (String)distance);
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

double calc(double d, double lim)
{
  double result = ((d - threshHoldDistance)/threshHoldDistance) * lim;

  if (result > lim)
  {
    result = lim;
  }
  else if(result < -lim)
  {
    result = -lim;
  }

  return result;
}

void backAndForwardControl() {
  double distance = (double)analogRead(pin_head) / 2;

  double temp = calc(distance, 0.3);

  if(temp > 0)
  {
    steerRight(temp);
  }
  else
  {
    steerLeft(-temp);
  }
   
}

void steerTheCar(double dis) {
  if (dis < 0.8*threshHoldDistance * threshHoldDistance) {
    Serial.println("dis: " + (String)dis + "turn left    " + (String)(0.8*threshHoldDistance * threshHoldDistance));
    steerLeft(0.2);
  } else if (dis > 1.2*threshHoldDistance * threshHoldDistance){
    Serial.println("dis: " + (String)dis + "turn right   " + (String)(1.2*threshHoldDistance * threshHoldDistance));
    steerRight(0.2);
  } else {
    steerLeft(0.0);
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
    
    wheels.write(90 - temp);
  }
}

/*
  Set the velocity of the car. Control the back and forward directions.
  Input s > 0 will go forward, and s < 0 will go backward.
  The input should between -1 - 1.
  
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
      distance_sum += calcDistance();
      count--; 
   } else {
      steerTheCar(distance_sum / 10);
      distance_sum = 0;
      count = 10;
   }
   setVelocity(0.3);
//   steerTheCar(calcDistance());
//   Serial.println("distance: " + (String)calcDistance());
   delay(500);
}



