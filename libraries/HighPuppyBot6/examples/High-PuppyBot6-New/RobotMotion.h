#pragma once
// การเดิน ถอย และเลี้ยว; ค่าระยะหลักอยู่ RobotConfig.h

void CrossForkObstacle() {
  moveStraightPID_Encoder(0, Fix_direction(), FORK_SPEED, FORK_PULSES, 3, 0, 0.25, 0);

  return;
}

void LeaveStart() {
  moveStraightPID_Encoder(0, Fix_direction(), BACK_SPEED, CELL_PULSES, 3, 0, 0.25, 0);

  return;
}

void TurnLeft() {
  turnByAngle_fix(-90, 0);
  return;
}

void ClimbBridge() {
  moveStraightPID_Encoder(0, Fix_direction(), BRIDGE_SPEED, CLIMB_PULSES, 3, 0, 0.2, 0);

  return;
}

void BackFromFloorLine() {
  moveStraightPID_Encoder(1, Fix_direction(), BACK_SPEED, FLOOR_BACK_PULSES, 3, 0, 0.25, 0);

  return;
}

void TurnRight() {
  turnByAngle_fix(90, 0);
  return;
}

void DescendBridge() {
  moveStraightPID_Encoder(0, Fix_direction(), BRIDGE_SPEED, DESCEND_PULSES, 3, 0, 0.2, 0);

  return;
}

void BackFromBridgeLine() {
  moveStraightPID_Encoder(1, Fix_direction(), BACK_SPEED, BRIDGE_BACK_PULSES, 3, 0, 0.25, 0);

  return;
}

void BackFromBridgeEdge() {
  moveStraightPID_Encoder(1, Fix_direction(), BACK_SPEED, EDGE_BACK_PULSES, 3, 0, 0.25, 0);

  return;
}

void BackAfterRelease() {
  moveStraightPID_Encoder(1, Fix_direction(), BACK_SPEED, RELEASE_BACK_PULSES, 2.8, 0, 0.1, 0);

  return;
}

void BackByState() {
  if (State == 1) {
    if (BridgeState == 1) {
      BackFromBridgeLine();
    } else {
      BackFromFloorLine();
    }
  } else if (State == 2) {
    BackAfterRelease();
  } else if (State == 3) {
    BackFromBridgeEdge();
  }

  return;
}
