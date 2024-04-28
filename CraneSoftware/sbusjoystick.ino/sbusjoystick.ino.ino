#include <Arduino.h>
#include <SBUS2.h>

uint8_t FrameErrorRate = 0;
int16_t channel[18] = {0};

#define THROTTLE_CH 0
#define STEERING_CH 1
#define CRANE_LIFT_CH 2
#define CRANE_EN_CH 5
#define MAGNET_CH 6
#define ARM_CH 7

void setup() {
  pinMode(13, OUTPUT);
  SBUS2_Setup();
}

void loop() {
  if (SBUS_Ready()) { // SBUS Frames available -> Ready for getting Servo Data
    for (uint8_t i = 0; i < 10; i++) {
      channel[i] =
          SBUS2_get_servo_data(i); // Channel = Servo Value of Channel 5
    }
    FrameErrorRate = SBUS_get_FER();
  }
  if (channel[ARM_CH] > 1400) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
    return;
  }

  //    if (channel[

} // End of Loop()
