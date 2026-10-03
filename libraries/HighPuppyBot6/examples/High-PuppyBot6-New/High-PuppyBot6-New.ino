// High-PuppyBot6 — ภารกิจล่าสุดจาก KBIDE
#define PUPPYBOT_USE_VL53L0X 1
#include "RobotConfig.h"
#include "RobotMotion.h"
#include "RobotSensors.h"
#include "RobotMission.h"

void setup() {
  delay(1000);
  puppybot_setup();
  Serial.begin(115200);

  HighRGB_Begin();
  Set_Mode_Gyro = (0);
  init_encoder(3);
  selection_Encoder = 3;
  ConfigureMotion();
  ConfigureFloorSensors();
  ConfigureBridgeSensors();
  ConfigureRGB();
  ResetBridgeState();
  ResetReleasedColors();
  ConfigureServos();

  swpin = 27;
  HighWaitSW1AllSensors(true, 500);
  /*__PUPPYBOT_FEATURE_TFT__*/
  tft_.fillScreen(0x0);
  get_pulse_Encoder(1);
  get_pulse_Encoder(2);
  resetEncoder(1);
  resetEncoder(2);
  LeaveStart();
}

void loop() { RunLeftSearch(); }

void setup1() {
  delay(1000);
  highIMUStartupStage.store(1, std::memory_order_release);
  bool imuStarted = initIMU();
  if (!imuStarted) highIMUStartupStage.store(-1, std::memory_order_release);
#ifdef PUPPYBOT_USE_REAR_TCS
  if (imuStarted) {
    highIMUStartupStage.store(2, std::memory_order_release);
    set_TCS_back();
  }
#endif
  if (imuStarted) {
    highIMUStartupStage.store(3, std::memory_order_release);
    updateIMU_fine_offset_Yaw(300);
    offset_yaw_loop1 = getOffsetYaw();
    highIMUStartupStage.store(4, std::memory_order_release);
  }
#ifdef PUPPYBOT_USE_VL53L0X
  HighToF_BeginCore1();
#endif
  highIMUCoreReady.store(imuStarted, std::memory_order_release);
}

void loop1() {
  if (isIMUReady()) {
    updateIMU();
    pitch_loop1 = getPitchCached();
    roll_loop1 = getRollCached();
    yaw_loop1 = getYawCached();
    updateContinuousYawCached();
    Con_yaw_loop1 = continuousYaw + Ref_offset_Con;
  }
#ifdef PUPPYBOT_USE_REAR_TCS
  updateTCSBackCache();
#endif
  if (status_resetYaw == 1) {
    status_resetYaw = 0;
    resetContinuousYaw();
  }
  if (status_resetYaw180 == 1) {
    resetGyro();
    status_resetYaw180 = 0;
  }
  HighIMU_ServiceReset();
#ifdef PUPPYBOT_USE_VL53L0X
  HighToF_ServiceCore1();
#endif
}
