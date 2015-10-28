#include <Servo.h>
#include <SoftwareSerial.h>
//LIDAR SENSOR
/**********************************************/
#include <Wire.h>
#include <I2C.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

int pos = 0;         // Position of the servo (degress, [0, 180])
int distance = 0;    // Distance measured
int sensorPins[] = {2,3}; // Array of pins connected to the sensor Power Enable lines
int sensorPinsArraySize = 2; // The length of the array

/********************************************/
//LIDAR SENSOR END
//create an xBee object
SoftwareSerial xbee(2,3); // Rx, Tx


Servo wheels; // servo for turning the wheels
Servo esc; // not actually a servo, but controlled like one!
bool startup = true; // used to ensure startup only happens once
int startupDelay = 1000; // time to pause at each calibration step
double maxSpeedOffset = 45; // maximum speed magnitude, in servo 'degrees'
double maxWheelOffset = 85; // maximum wheel turn magnitude, in servo 'degrees'

double wheelOffset = 0.0; // For Adjusting the wheel
double threshHoldDistance = 69.85;

double steerAngle = 0.0;
double car_length = 13;
bool goForward = true;
//double wheelStartUpOffset = 0.0; // For Adjusting the steering

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

  setVelocity(0.0);
  xbee.begin(9600);
  Serial.begin(9600);

  Wire.begin(); // join i2c bus
  for (int i = 0; i < sensorPinsArraySize; i++){
    pinMode(sensorPins[i], OUTPUT); // Pin to first LIDAR-Lite Power Enable line
    Serial.print(sensorPins[i]);
  }
}
int lidarGetRange(void)
{
  int val = -1;
  
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
  Wire.endTransmission(); // stop transmitting

  delay(15); // Wait 20ms for transmit

  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting

  delay(15); // Wait 20ms for transmit
  
  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

  if(2 <= Wire.available()) // if two bytes were received
  {
    val = Wire.read(); // receive high byte (overwrites previous reading)
    val = val << 8; // shift high byte to be high 8 bits
    val |= Wire.read(); // receive low byte as lower 8 bits
  }
  return val;
}
/*double getHeadDis() {
  double A1 = (double)analogRead(pin_head);
  double distance_head = exp(8.5841-log(A1));
//  Serial.println("head: "+ (String)distance_head);
  return distance_head;
}*/

void enableDisableSensor(int sensorPin){
  for (int i = 0; i < sensorPinsArraySize; i++){
      digitalWrite(sensorPins[i], LOW); // Turn off all sensors
  }
  digitalWrite(sensorPin, HIGH); // Turn on the selected sensor
  delay(1); // The sensor takes 1msec to wake
}

double getHeadDis() {
  enableDisableSensor(2);  
    int sum = 0; // Variable to store sum
    for(int i = 0; i < 2; i++){ 
        int val =  lidarGetRange();
        if(val<0 || val > 400){
          i--;
        }
        else{
        sum = sum + val;// Add up all of the readings
        }
    }
    sum = sum/2; // Divide the total by the number of readings to get the average
    return sum;
   
}
double getTailDis() {
  enableDisableSensor(3);  
      int sum = 0; // Variable to store sum
      for(int i = 0; i < 2; i++){ 
          int val =  lidarGetRange();
          if(val<0 || val > 400){
            i--;
          }
          else{
          sum = sum + val;// Add up all of the readings
          }
      }
      sum = sum/2; // Divide the total by the number of readings to get the average
      return sum;
}

boolean compareHeadTail(double head, double tail) {
    if (abs(head-tail) < 5) {
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
//  Serial.write("Steer Left:");
//  Serial.write("\n");
  
  if( (d >= 0.0 ) && (d <= 1.0))
  {
    double temp = min( (d * maxWheelOffset + wheelOffset), maxWheelOffset);
    
    wheels.write(90 + temp);
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
  {
    esc.write(90 - (s * maxSpeedOffset));
  }
}



void loop()
{
  if (xbee.available() > 0) {
    String msg  = "";

    // Read in message
    while(xbee.available() > 0) {
      msg += char(xbee.read());
    }
    if (msg.equals("START\n")) {
      Serial.println(msg);
      setVelocity(0.3);                          
    
    } else if (msg.equals("STOP\n")) {
      Serial.println(msg);
      setVelocity(0.0);
    }
  } else {
      head_sum = getHeadDis();
      tail_sum = getTailDis();
      if (compareHeadTail(head_sum, tail_sum)) {      
          steerRight(0.1);
      }
      Serial.println("head:" + (String)head_sum + "   tail:   "+ (String)tail_sum);

      head_sum = getHeadDis();
      tail_sum = getTailDis();
      if (!compareHeadTail(head_sum, tail_sum)) {
         steerTheCar(calcDistance(head_sum, tail_sum));  
      }
      head_sum = 0;
      tail_sum = 0;
 
   }
  
  delay(5);
}



