#pragma once
#include <HighPuppyBot6.h>
// State: 0=ครบระยะ/ข้ามอุปสรรค 1=เส้นดำ 2=ปล่อยสี 3=ขอบสะพาน
int State = 0;
int BridgeState = 0; // 0=พื้น 1=บนสะพาน 2=กำลังลง
bool ReleasedRed=false, ReleasedGreen=false, ReleasedBlue=false, ReleasedYellow=false;
bool Finished=false;
long Start_Encoder=0, CheckStartEncoder=0;
uint32_t StartTime=0, CheckStartTime=0;
constexpr long CELL_PULSES=2400, RGB_APPROACH_PULSES=410;
constexpr uint32_t SEARCH_TIMEOUT_MS=5000, APPROACH_TIMEOUT_MS=3000;
constexpr int EDGE_DISTANCE_MM=70;
constexpr float BRIDGE_UP_DEG=5, BRIDGE_DOWN_DEG=-4;
constexpr int SEARCH_SPEED=50, APPROACH_SPEED=17;
constexpr uint32_t SETTLE_MS=150, FINISH_PAUSE_MS=60000;
constexpr long FORK_PULSES=5700, CLIMB_PULSES=3700, DESCEND_PULSES=4200;
constexpr long FLOOR_BACK_PULSES=600, BRIDGE_BACK_PULSES=400, EDGE_BACK_PULSES=250, RELEASE_BACK_PULSES=2400;
constexpr int FORK_SPEED=30, BRIDGE_SPEED=40, BACK_SPEED=50;
// ระยะหน่วยพัลส์/ToF หน่วยมิลลิเมตร/Pitch หน่วยองศา
void ConfigureRGB();
void ConfigureMotion();
void ConfigureFloorSensors();
void ConfigureServos();
void CheckRGB();
void ResetReleasedColors();
void ResetBridgeState();
void ConfigureBridgeSensors();
void CrossForkObstacle();
void LeaveStart();
void ShowRGB();
void TurnLeft();
void ClimbBridge();
void ShowPitch();
void BackFromFloorLine();
void TurnRight();
void DescendBridge();
void BackFromBridgeLine();
void ShowToF();
void RunLeftSearch();
void BackFromBridgeEdge();
void BackAfterRelease();
void CheckObstacles();
void BackByState();

void ConfigureRGB() {
  HighRGB_SetTolerance(0.035, 0.7);

  HighRGB_SetReference(RGB_RED, 734, 225, 241, 1036);

  HighRGB_SetReference(RGB_GREEN, 323, 692, 399, 1390);

  HighRGB_SetReference(RGB_BLUE, 116, 227, 346, 649);

  HighRGB_SetReference(RGB_YELLOW, 1120, 994, 427, 2470);

  HighRGB_SetReference(RGB_WHITE, 1303, 1586, 1503, 4183);

  HighRGB_SetReference(RGB_BLACK, 70, 79, 71, 201);

  return;
}

void ConfigureMotion() {
  HighIMUSetMotionGuard(3000, 10000);
  set_data_for_turnDirection(1, 19, 69, 1.3, 0.1);

  return;
}

void ConfigureFloorSensors() {
  HighTC01SetThreshold(0, 952, 530);

  HighTC01SetThreshold(1, 962, 582);

  return;
}

void ConfigureServos() {
  servoRun(0, 90);
  servoRun(1, 90);
  servoRun(2, 20);

  return;
}

void ResetReleasedColors() {
  ReleasedRed = 0;
  ReleasedGreen = 0;
  ReleasedBlue = 0;
  ReleasedYellow = 0;
  Finished = 0;

  return;
}

void ResetBridgeState() {
  BridgeState = 0;

  return;
}

void ConfigureBridgeSensors() {
  PID_NumPin = 2;
  PID_set_Pin(0, 1, 19, 19, 19, 19, 19, 19);
  PID_set_Min(414, 433, 0, 0, 0, 0, 0, 0);
  PID_set_Max(958, 965, 0, 0, 0, 0, 0, 0);

  return;
}

void ConfigureRobot() {
 HighRGB_Begin(); Set_Mode_Gyro=0; init_encoder(3); selection_Encoder=3;
 ConfigureMotion(); ConfigureFloorSensors(); ConfigureBridgeSensors(); ConfigureRGB();
 ResetBridgeState(); ResetReleasedColors(); ConfigureServos();
}
