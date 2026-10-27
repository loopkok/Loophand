#include "Homing.h"
#include "HandConfig.h"

// ----------------------- Immutable baselines -----------------------
// Always define both symbols so linker can resolve them
const ServoData sd_base_left[7] = {
  {3186,2048,1},{2048,865,-1},{0,2980,1},{4095,817,-1},{4095,817,-1},{4095,817,-1},{4095,817,-1},
};
const ServoData sd_base_right[7] = {
  {910,2048,-1},{2048,3231,1},{4095,1115,-1},{0,3278,1},{0,3278,1},{0,3278,1},{0,3278,1},
};

// ----------------------- Utilities -----------------------
void resetSdToBaseline() {
#if defined(RIGHT_HAND)
  const ServoData* src = sd_base_right;
#elif defined(LEFT_HAND)
  const ServoData* src = sd_base_left;
#else
  #warning "No hand macro defined; defaulting to RIGHT_HAND baseline"
  const ServoData* src = sd_base_right;
#endif
  for (int i = 0; i < 7; ++i) sd[i] = src[i];
}

// ----------------------- Busy flag & homing core -----------------------
static volatile bool s_busy = false;
bool HOMING_isBusy() { return s_busy; }

static void zero_with_current(uint8_t servoID, int direction, int current_limit) {
  int current = 0;
  int position = 0;
  hlscl.ServoMode(servoID);
  hlscl.FeedBack(servoID);
  uint32_t t0 = millis();
  while (abs(current) < current_limit) {
    hlscl.WritePosEx(servoID, 50000 * direction, 10, 10, current_limit);
    current  = hlscl.ReadCurrent(servoID);
    position = hlscl.ReadPos(servoID);
    if (millis() - t0 > 25000) break; 
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  // Primary calibration at contact
  hlscl.WritePosEx(servoID, position, 60, 50, 1000);
  delay(30);
  hlscl.CalibrationOfs(servoID);
  delay(50);
  position = hlscl.ReadPos(servoID);

  if (servoID == 0) {
    // Thumb abduction: hold grasp posture for a moment
    hlscl.WritePosEx(servoID, sd[servoID].grasp_count, 60, 50, 500);
    delay(2000);
  } else if (servoID == 1) {
    // Thumb flexion: go to extend
    hlscl.WritePosEx(servoID, sd[servoID].extend_count, 60, 50, 500);
    delay(1000);
  } else if (servoID == 2) {
    // Thumb tendon: nudge and recalibrate, then extend
    hlscl.WritePosEx(servoID, position + (direction * 2048), 60, 50, 500);
    delay(1500);
    hlscl.CalibrationOfs(servoID);
    delay(500);
    hlscl.WritePosEx(servoID, sd[servoID].extend_count, 60, 50, 500);
    delay(1000);
  } else {
    // Fingers: nudge and recalibrate, then extend
    hlscl.WritePosEx(servoID, position + (direction * 2048), 60, 50, 500);
    delay(1500);
    hlscl.CalibrationOfs(servoID);
    delay(500);
    hlscl.WritePosEx(servoID, sd[servoID].extend_count, 60, 50, 500);
    delay(1000);
  }
}

void zero_all_motors() {
  resetSdToBaseline();
  if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);
  zero_with_current(SERVO_IDS[0],  sd[0].servo_direction, 750);   // Thumb Abduction
  zero_with_current(SERVO_IDS[1],  sd[1].servo_direction, 950);   // Thumb Flex
  zero_with_current(SERVO_IDS[2],  sd[2].servo_direction, 950);   // Thumb Tendon
  zero_with_current(SERVO_IDS[3],  sd[3].servo_direction, 950);   // Index
  zero_with_current(SERVO_IDS[4],  sd[4].servo_direction, 950);   // Middle
  zero_with_current(SERVO_IDS[5],  sd[5].servo_direction, 950);  // Ring
  zero_with_current(SERVO_IDS[6],  sd[6].servo_direction, 950);  // Pinky
  // Post-homing settling moves
  hlscl.WritePosEx(SERVO_IDS[0], sd[0].extend_count, 2400, 0, 1023);   // Thumb Abduction to extend
  hlscl.WritePosEx(SERVO_IDS[1], sd[1].extend_count, 2400, 0, 1023);   // Thumb Flexion to extend
  hlscl.WritePosEx(SERVO_IDS[2], sd[2].extend_count, 2400, 0, 1023);   // Thumb Tendon to extend
  hlscl.WritePosEx(SERVO_IDS[3], sd[3].extend_count, 2400, 0, 1023);   // Index to extend
  hlscl.WritePosEx(SERVO_IDS[4], sd[4].extend_count, 2400, 0, 1023);   // Middle to extend
  hlscl.WritePosEx(SERVO_IDS[5], sd[5].extend_count, 2400, 0, 1023);   // Ring to extend
  hlscl.WritePosEx(SERVO_IDS[6], sd[6].extend_count, 2400, 0, 1023);   // Pinky to extend
  if (gBusMux) xSemaphoreGive(gBusMux);
}

void HOMING_start() {
  s_busy = true;
  zero_all_motors();
  s_busy = false;
}