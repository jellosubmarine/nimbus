//#include <AFMotor.h>
//#include <Arduino.h>
//#include <SBUS2.h>
//
//#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))
#include <Servo.h>


#define THROTTLE_CH 1
#define STEERING_CH 2
#define CRANE_LIFT_CH 3
#define CRANE_EN_CH 6
#define MAGNET_CH 7
#define ARM_CH 8

#define MAX_TURN_SPEED 200
#define MAX_LINEAR_SPEED 200
#define THROTTLE_CENTER 1400
#define YAW_CENTER 1495
#define PITCH_CENTER 1488
#define ROLL_CENTER 1400
#define DEADZONE 80
#define SERVO_STOP_SPEED 1500

// Motor 1
int dir1PinA = 2;
int dir2PinA = 3;
int speedPinA = 5; // Needs to be a PWM pin to be able to control motor speed

// Motor 2
int dir1PinB = 4;
int dir2PinB = 7;
int speedPinB = 6; // Needs to be a PWM pin to be able to control motor speed

void setPwmFrequency(int pin) {
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | 0x01;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | 0x01;
    }
  } else if (pin == 3 || pin == 11) {
    TCCR2B = TCCR2B & 0b11111000 | 0x01;
  }
}
Servo myservo;  // create servo object to control a servo
Servo magnet;
void setup ()
{
  setPwmFrequency(speedPinA);
  setPwmFrequency(speedPinB);

  pinMode(13, OUTPUT);
  pinMode(dir1PinA,OUTPUT);
  pinMode(dir2PinA,OUTPUT);
  pinMode(speedPinA,OUTPUT);
  pinMode(dir1PinB,OUTPUT);
  pinMode(dir2PinB,OUTPUT);
  pinMode(speedPinB,OUTPUT);
  Serial.begin(100000, SERIAL_8E2);
  myservo.attach(9);
  magnet.attach(10);
  magnet.writeMicroseconds(2000);
}


void loop ()
{

  //Declare the variabes
  static byte          buffer[25];
  static int           channels[18];
  static int           errors = 0;
  static bool          failsafe = 0;
  static int           idx;
  static unsigned long last_refresh = 0;
  static int           lost = 0;
  byte b;
  int  i;



  //Check the serial port for incoming data
  //This could also be done via the serialEvent()
//  Serial.println(Serial1.available());
  if (Serial.available ()) {
    b = Serial.read ();

    //this is a new package and it' not zero byte then it's probably the start byte B11110000 (sent MSB)
    //so start reading the 25 byte package
    if (idx == 0 && b != 0x0F) {  // start byte 15?
      // error - wait for the start byte

    } else {
      buffer[idx++] = b;  // fill the buffer with the bytes until the end byte B0000000 is recived
    }

    if (idx == 25) {
      idx = 0;
      if (buffer[24] != 0x00) {
        errors++;
      } else
      {
        channels[1]  = ((buffer[1]    | buffer[2] << 8)                 & 0x07FF);
        channels[2]  = ((buffer[2] >> 3 | buffer[3] << 5)                 & 0x07FF);
        channels[3]  = ((buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10)  & 0x07FF);
        channels[4]  = ((buffer[5] >> 1 | buffer[6] << 7)                 & 0x07FF);
        channels[5]  = ((buffer[6] >> 4 | buffer[7] << 4)                 & 0x07FF);
        channels[6]  = ((buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9)   & 0x07FF);
        channels[7]  = ((buffer[9] >> 2 | buffer[10] << 6)                & 0x07FF);
        channels[8]  = ((buffer[10] >> 5 | buffer[11] << 3)                & 0x07FF);
        channels[9]  = ((buffer[12]   | buffer[13] << 8)                & 0x07FF);
        channels[10]  = ((buffer[13] >> 3 | buffer[14] << 5)                & 0x07FF);
        channels[11] = ((buffer[14] >> 6 | buffer[15] << 2 | buffer[16] << 10) & 0x07FF);
        channels[12] = ((buffer[16] >> 1 | buffer[17] << 7)                & 0x07FF);
        channels[13] = ((buffer[17] >> 4 | buffer[18] << 4)                & 0x07FF);
        channels[14] = ((buffer[18] >> 7 | buffer[19] << 1 | buffer[20] << 9)  & 0x07FF);
        channels[15] = ((buffer[20] >> 2 | buffer[21] << 6)                & 0x07FF);
        channels[16] = ((buffer[21] >> 5 | buffer[22] << 3)                & 0x07FF);
        channels[17] = ((buffer[23])      & 0x0001) ? 2047 : 0;
        channels[18] = ((buffer[23] >> 1) & 0x0001) ? 2047 : 0;

        failsafe = ((buffer[23] >> 3) & 0x0001) ? 1 : 0;
        if ((buffer[23] >> 2) & 0x0001) lost++;
//        Serial.print("Lost: ");
//        Serial.println(lost);
//        Serial.println(channels[0]);
        // Arm
        if (channels[8] < 1500) {
          analogWrite(speedPinA, 0);
          analogWrite(speedPinB, 0);
            digitalWrite(13, LOW);
          myservo.writeMicroseconds(SERVO_STOP_SPEED);
          magnet.writeMicroseconds(2000);
          return;
        }
        else {
          digitalWrite(13, HIGH);
        }
        //Linear
        if (channels[THROTTLE_CH] > THROTTLE_CENTER + DEADZONE) {
          analogWrite(speedPinA, map(channels[THROTTLE_CH], THROTTLE_CENTER + DEADZONE, 1700, MAX_LINEAR_SPEED-40, MAX_LINEAR_SPEED));//Sets speed variable via PWM
          digitalWrite(dir1PinA, HIGH);
          digitalWrite(dir2PinA, LOW);
          digitalWrite(13, HIGH);

        }
        else if (channels[THROTTLE_CH] < THROTTLE_CENTER - DEADZONE)
        {
          analogWrite(speedPinA, map(channels[THROTTLE_CH], THROTTLE_CENTER - DEADZONE,1100, MAX_LINEAR_SPEED-40, MAX_LINEAR_SPEED));
          digitalWrite(dir1PinA, LOW);
          digitalWrite(dir2PinA, HIGH);
          digitalWrite(13, LOW);
        }
        else {
          analogWrite(speedPinA, 0);
          digitalWrite(dir1PinA, LOW);
          digitalWrite(dir2PinA, HIGH);
        }

        //Turning
        if (channels[STEERING_CH] > ROLL_CENTER + DEADZONE) {
          analogWrite(speedPinB, map(channels[STEERING_CH], ROLL_CENTER + DEADZONE, 1700, MAX_TURN_SPEED-40, MAX_TURN_SPEED));//Sets speed variable via PWM
          digitalWrite(dir1PinB, LOW);
          digitalWrite(dir2PinB, HIGH);
        }
        else if (channels[STEERING_CH] < ROLL_CENTER - DEADZONE)
        {
          analogWrite(speedPinB, map(channels[STEERING_CH], ROLL_CENTER - DEADZONE, 1100, MAX_TURN_SPEED-40, MAX_TURN_SPEED));
          digitalWrite(dir1PinB, HIGH);
          digitalWrite(dir2PinB, LOW);
        }
        else {
          analogWrite(speedPinB, 0);
          digitalWrite(dir1PinB, LOW);
          digitalWrite(dir2PinB, HIGH);
        }

        //Crane servo
        if (channels[CRANE_EN_CH] > 1200) {
//          if (channels[CRANE_LIFT_CH] > 1507 + 10 || channels[CRANE_LIFT_CH] < 1507 - 10) {
            myservo.writeMicroseconds(channels[CRANE_LIFT_CH]);
//          }
        }
        else {
          myservo.writeMicroseconds(SERVO_STOP_SPEED);
        }
        magnet.writeMicroseconds(channels[7]);

        
      } //closing - else
    } //closing - if (idx == 25)
  } //closing - if (Serial.available ())
} //closing void loop












//
//AF_DCMotor motor_fwd(3, MOTOR12_64KHZ);
//AF_DCMotor motor_str(4, MOTOR12_64KHZ);

//void setup() {
//  pinMode(13, OUTPUT);
////  SBUS2_Setup();
////
////  motor_fwd.setSpeed(150);
////  motor_fwd.run(FORWARD);
////  motor_str.setSpeed(150);
////  motor_str.run(FORWARD);
//}
//
//void loop() {
//  if (SBUS_Ready()) { // SBUS Frames available -> Ready for getting Servo Data
//    for (uint8_t i = 0; i < 10; i++) {
//      channel[i] =
//          SBUS2_get_servo_data(i); // Channel = Servo Value of Channel 5
//    }
//    FrameErrorRate = SBUS_get_FER();
//
//    if (channel[ARM_CH] > 1400) {
//      digitalWrite(13, HIGH);
//    } else {
//      digitalWrite(13, LOW);
//      return;
//    }
//
//    if (channel[THROTTLE_CH] > UPPER_DEADZONE ||
//        channel[THROTTLE_CH] < LOWER_DEADZONE) {
//      int16_t diff = int16_t(channel[THROTTLE_CH]) - MIDDLE_VALUE;
//      uint8_t speed = uint8_t(float(abs(diff)) / SPEED_RANGE_FWD_MS * 255.0);
//
//      if (diff > 0) {
//        motor_fwd.run(BACKWARD);
//      } else {
//        motor_fwd.run(FORWARD);
//      }
//
//    } else {
//      motor_fwd.run(RELEASE);
//    }
//  }
//
//} // End of Loop()
