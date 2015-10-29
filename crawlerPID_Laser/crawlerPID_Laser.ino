#include <Servo.h>
#include <PID_v1.h>

double Setpoint, Input, Output;
double Kp=3.0, Ki=0.00001, Kd=1.2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
 
Servo wheels; // servo for turning the wheels
Servo esc; // not actually a servo, but controlled like one!
bool startup = true; // used to ensure startup only happens once
int startupDelay = 1000; // time to pause at each calibration step
double maxSpeedOffset = 45; // maximum speed magnitude, in servo 'degrees'
double maxWheelOffset = 85; // maximum wheel turn magnitude, in servo 'degrees'

int pin_head = 0;
int pin_tail = 3;

//double threshHoldDistance = 27.5 * 27.5;
double threshHoldDistance = 25;
double car_length = 13;         //inches

void setup()
{
  wheels.attach(8); // initialize wheel servo to Digital IO Pin #8
  esc.attach(9); // initialize ESC to Digital IO Pin #9
  /*  If you're re-uploading code via USB while leaving the ESC powered on, 
   *  you don't need to re-calibrate each time, and you can comment this part out.
   */
  calibrateESC();
  Input = calcDistance(getDistance(pin_head), getDistance(pin_tail));
  Setpoint = threshHoldDistance;

  //turn the PID on
  myPID.SetOutputLimits(-0.6,0.5);
  myPID.SetMode(AUTOMATIC);

  
  setVelocity(0.3);
}

double getDistance(int pin) {
  double A1 = (double)analogRead(pin);
  double distance = exp(8.5841-log(A1));
  return distance;  
}

double calcDistance(double head, double tail) {
  double distance_head = head;
  double distance_tail = tail;
  double distance = (car_length * car_length * distance_head * distance_head) / (car_length * car_length + (distance_head - distance_tail) * (distance_head - distance_tail));
  return sqrt(distance);
}

void setVelocity(double s)
{
  if( (s >= -1.0 ) && (s <= 1.0))
  {
    esc.write(90 - (s * maxSpeedOffset));
  }
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


void steer(double d)
{
  if( (d >= 0.0 ) && (d <= 1.0))
  {
    double temp = d * maxWheelOffset;
    
    wheels.write(90 + temp);
  }
}
 
void loop()
{
  myPID.Compute();
  steer(Output);
  delay(100);
}


