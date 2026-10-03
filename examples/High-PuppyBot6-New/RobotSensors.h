#pragma once
// ตรวจเส้น สี สะพาน และ ToF ตามลำดับเดิมจาก KBIDE

void CheckRGB() {
  if ((State == 1) && (BridgeState == 0)) {
    HighRGB_Update();
    if (

        HighRGB_Valid()) {
      if ((

              HighRGB_LastIsColor(RGB_RED)) &&
          (ReleasedRed == 0)) {
        ReleasedRed = 1;
        State = 2;
        servoRun(0, 170);
        delay(300);
        servoRun(0, 90);
        delay(200);
      } else if ((

                     HighRGB_LastIsColor(RGB_GREEN)) &&
                 (ReleasedGreen == 0)) {
        ReleasedGreen = 1;
        State = 2;
        servoRun(0, 10);
        delay(300);
        servoRun(0, 90);
        delay(200);
      } else if ((

                     HighRGB_LastIsColor(RGB_BLUE)) &&
                 (ReleasedBlue == 0)) {
        ReleasedBlue = 1;
        State = 2;
        servoRun(1, 170);
        delay(300);
        servoRun(1, 90);
        delay(200);
      } else if ((

                     HighRGB_LastIsColor(RGB_YELLOW)) &&
                 (ReleasedYellow == 0)) {
        ReleasedYellow = 1;
        State = 2;
        servoRun(1, 10);
        delay(300);
        servoRun(1, 90);
        delay(200);
      } else {
      }
      if ((ReleasedRed == 1) &&
          ((ReleasedGreen == 1) &&
           ((ReleasedBlue == 1) &&
            ((ReleasedYellow == 1) && (Finished == 0))))) {
        ao();
        servoRun(2, 70);
        delay(300);
        Finished = 1;
        delay(FINISH_PAUSE_MS);
      }
    } else {
    }
  }

  return;
}

void ShowRGB() {
  HighRGB_ReadAndShow();
  delay(SETTLE_MS);

  return;
}

void ShowPitch() {
  /*__PUPPYBOT_FEATURE_TFT__*/
  HighDisplayTextAt(String(pitch_loop1), HIGH_DISPLAY_POS_CENTER,
                    HIGH_DISPLAY_CENTER, 0, 0xffff);

  return;
}

void ShowToF() {
  /*__PUPPYBOT_FEATURE_TFT__*/
  HighDisplayTextAt(String(

                        HighToF_DistanceMM()),
                    HIGH_DISPLAY_POS_TOP, HIGH_DISPLAY_CENTER, 0, 0xffff);

  return;
}

void CheckObstacles() {
  ao();
  State = 0;
  if (BridgeState == 2) {
    return;
  }
  get_pulse_Encoder(1);
  get_pulse_Encoder(2);
  resetEncoder(1);
  resetEncoder(2);
  Start_Encoder = HighReadAverageEncoderPulse();
  StartTime = millis();
  while (
      (((((long)(HighReadAverageEncoderPulse()) - (long)(Start_Encoder)) >= 0
             ? ((long)(HighReadAverageEncoderPulse()) - (long)(Start_Encoder))
             : -(((long)(HighReadAverageEncoderPulse()) -
                  (long)(Start_Encoder)))) < (long)(CELL_PULSES)) &&
       ((SEARCH_TIMEOUT_MS) <= 0 || (unsigned long)(millis() - (unsigned long)(StartTime)) <
                           (unsigned long)(SEARCH_TIMEOUT_MS)))) {
    if (

        HighToF_Compare(1, EDGE_DISTANCE_MM)) {
      ao();
      State = 3;
      break;
    } else if (BridgeState == 1) {
      if ((Read_status_sensor(0)) || (Read_status_sensor(1))) {
        ao();
        State = 1;
        break;
      } else if (pitch_loop1 < BRIDGE_DOWN_DEG) {
        BridgeState = 2;
        DescendBridge();
        BridgeState = 0;
        break;
      } else {
        moveStraightPID_step(0, Fix_direction(), SEARCH_SPEED, 2.5, 0.25);
      }
    } else if (BridgeState == 0) {
      if (((ADC(0)) < HighTC01GetThreshold(0)) ||
          ((ADC(1)) < HighTC01GetThreshold(1))) {
        ao();
        State = 1;
        get_pulse_Encoder(1);
        get_pulse_Encoder(2);
        resetEncoder(1);
        resetEncoder(2);
        CheckStartEncoder = HighReadAverageEncoderPulse();
        CheckStartTime = millis();
        while ((((((long)(HighReadAverageEncoderPulse()) -
                   (long)(CheckStartEncoder)) >= 0
                      ? ((long)(HighReadAverageEncoderPulse()) -
                         (long)(CheckStartEncoder))
                      : -(((long)(HighReadAverageEncoderPulse()) -
                           (long)(CheckStartEncoder)))) < (long)(RGB_APPROACH_PULSES)) &&
                ((APPROACH_TIMEOUT_MS) <= 0 ||
                 (unsigned long)(millis() - (unsigned long)(CheckStartTime)) <
                     (unsigned long)(APPROACH_TIMEOUT_MS)))) {
          if ((IN(40)) == 0) {
            State = 0;
            CrossForkObstacle();
            break;
          } else {
            moveStraightPID_step(0, Fix_direction(), APPROACH_SPEED, 1, 0.1);
          }
        }
        ao();
        break;
      } else if (pitch_loop1 > BRIDGE_UP_DEG) {
        BridgeState = 1;
        ClimbBridge();
        break;
      } else {
        moveStraightPID_step(0, Fix_direction(), SEARCH_SPEED, 2.5, 0.25);
      }
    }
  }
  ao();
  delay(SETTLE_MS);
  CheckRGB();

  return;
}
