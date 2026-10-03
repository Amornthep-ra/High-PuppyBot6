
#ifndef _PUPPYBOT_IMU_H
#define _PUPPYBOT_IMU_H

#include <ICM20948wire1.h>
#include <Wire.h>
#include "pio_encoder.h"

#ifndef I2C_ADDR
#define I2C_ADDR 0x69
#endif



namespace PuppyBotIMU {
    static PioEncoder encoder_PIO_M1(17);
    static PioEncoder encoder_PIO_M2(19);
    
    static ICM20948 icm;
    static float continuousYaw = 0;
    static float offset_yaw = 0;
    static float read_Yaw = 0;
    static float newYaw = 0;
    static float pitch = 0;
    static float roll = 0;
    static float Ref_offset_Con = 0;
    static int state_begin_Encoder_M1 = 0;
    static int state_begin_Encoder_M2 = 0;
    static int Set_Mode_Gyro =0;
    static bool imuReady = false;
    static int imuInitErrorCode = 0;
    static bool imuPreResetAck = false;

    bool isIMUReady() { return imuReady; }

    void setModeGyro(int modeGyro){
        Set_Mode_Gyro = modeGyro;
    }
    void resetBoard() {
        Serial.println("Rebooting...");
        //while (1);
    }
    void init_encoder(int position_motor){
        if((position_motor == 1 || position_motor == 3) && !state_begin_Encoder_M1){
            pinMode(17, INPUT);
            pinMode(18, INPUT);
            encoder_PIO_M1.begin();
            state_begin_Encoder_M1 = 1;
        }
        if((position_motor == 2 || position_motor == 3) && !state_begin_Encoder_M2){
            pinMode(19, INPUT);
            pinMode(20, INPUT);
            encoder_PIO_M2.begin();
            state_begin_Encoder_M2 = 1;
        }
    }
    // The RP2350 reset button does not remove power from the ICM-20948.
    // Restore bank 0, then reset the sensor itself before loading its DMP.
    // A missing I2C acknowledgement is left to icm.begin() to report.
    bool resetICMBeforeInit() {
        Wire1.beginTransmission(I2C_ADDR);
        Wire1.write((uint8_t)0x7F); // REG_BANK_SEL
        Wire1.write((uint8_t)0x00); // user bank 0
        if (Wire1.endTransmission() != 0) return false;
        Wire1.beginTransmission(I2C_ADDR);
        Wire1.write((uint8_t)0x06); // PWR_MGMT_1
        Wire1.write((uint8_t)0x80); // DEVICE_RESET
        if (Wire1.endTransmission() != 0) return false;
        delay(100);
        return true;
    }
    bool initIMU() {
        imuReady = false;
        imuInitErrorCode = 0;
        Wire1.setSDA(6);
        Wire1.setSCL(7);
        Wire1.begin();
        Wire1.setClock(400000);
        imuPreResetAck = resetICMBeforeInit();
        if (!imuPreResetAck) Serial.println("ICM-20948 pre-reset I2C not acknowledged");
        // Select 6-axis fusion before begin(). The driver defaults to 9-axis
        // and otherwise treats a missing magnetometer as a fatal init error.
        icm.setFusionMode(FUSION_6_AXIS);
        int code = icm.begin();
        Wire1.setClock(400000);
        if (code != ICM_SUCCESS) {
            imuInitErrorCode = code;
            Serial.print("ICM-20948 initialization failed, code: ");
            Serial.println(code);
            return false;
        }

        icm.startSensor(INV_SENSOR_TYPE_MAGNETOMETER, 16000);
        icm.startSensor(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 16000);
        icm.startSensor(INV_SENSOR_TYPE_ROTATION_VECTOR, 16000);
        icm.setHighPowerMode(1);
        imuReady = true;
        Serial.println("ICM-20948 init ok");
        return true;
    }
    void updateIMU_fine_offset_Yaw(int round) {
      if (!imuReady) return;
      for (int Rread = 0; Rread < round; Rread++) {
        icm.update();
        if (icm.available()) {
          float qx = icm.x();
          float qy = icm.y();
          float qz = icm.z();
          float qw = icm.w();
          // Driver accessors x/y/z/w actually return event components w/x/y/z.
          // Preserve PuppyBot6's heading: 180 at identity, clockwise positive.
          read_Yaw = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / PI;
          if (read_Yaw < 0) { read_Yaw += 360; }
          //Serial.print("  Yaw: "); Serial.println(Yaw);
          icm.clearAvailable();
        }
        delay(10);
      }
      offset_yaw = read_Yaw;
    }

    void updateIMU() {
        if (!imuReady) return;
        icm.update();
        float qx = icm.x();
        float qy = icm.y();
        float qz = icm.z();
        float qw = icm.w();
        // Names follow the legacy wrapper, not conventional quaternion names:
        // qx=scalar w, qy=vector x, qz=vector y, qw=vector z.
        newYaw = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / PI;
        
        float sinp = 2 * (qw * qy - qz * qx);
        pitch = (fabs(sinp) >= 1) ? copysign(M_PI / 2, sinp) : asin(sinp);
        pitch *= 180.0 / M_PI;

        float siny_cosp = 2 * (qw * qz + qx * qy);
        float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        roll = atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;

        // Legacy behavior: swap pitch/roll to match older board orientation.
        // If this is wrong for your setup, remove the two lines below.
        float __tmp = roll;
        roll = pitch;
        pitch = __tmp;

        // Make "nose up" positive on the swapped pitch axis.
        // If you want the opposite sign, remove or negate this line.
        pitch = -pitch;

        // Make "left tilt" positive on the swapped roll axis.
        // If you want the opposite sign, remove or negate this line.
        roll = -roll;
    }

    float getYaw() {
        updateIMU();
        if (isnan(newYaw)) return read_Yaw;
        if (newYaw < 0) newYaw += 360;
        if (newYaw > 360) newYaw -= 360;
        read_Yaw = newYaw;
        return read_Yaw;
    }
    void updateContinuousYaw() {
    static float lastYaw = 0;
    static bool firstRun = true;

    updateIMU(); // อ่านค่าจาก IMU

    float currentYaw = getYaw(); // อ่านค่า getYaw() แค่ครั้งเดียว

    if (firstRun) {
      continuousYaw = currentYaw;
      lastYaw = currentYaw;
      firstRun = false;
      return;
    }

    // คำนวณ deltaYaw และ normalize ให้เป็นช่วง -180 ถึง 180 องศา
    float deltaYaw = currentYaw - lastYaw;
    if (deltaYaw > 180.0) deltaYaw -= 360.0;
    if (deltaYaw < -180.0) deltaYaw += 360.0;

    // อัปเดต continuousYaw
    continuousYaw += deltaYaw;
    
    // อัปเดต lastYaw
    lastYaw = currentYaw;
  }
    int get_pulse_Encoder(int position_motor){
        int count_encoder = 0;
        if(position_motor == 1){
            if(state_begin_Encoder_M1 == 0){
            state_begin_Encoder_M1 = 1;
            encoder_PIO_M1.begin();
            
            }
            count_encoder =  encoder_PIO_M1.getCount();
        }
        
        else{
            if(state_begin_Encoder_M2 == 0){
            state_begin_Encoder_M2 = 1;
            encoder_PIO_M2.begin();
            
            }
            count_encoder =  encoder_PIO_M2.getCount();
        }
        return count_encoder;
        
    }
    void resetEncoder(int position_motor){
        if(position_motor == 1){
            encoder_PIO_M1.reset();
        }
        else{
            encoder_PIO_M2.reset();
        }
        
    }


    float getPitch() { updateIMU(); return pitch; }
    float getRoll() { updateIMU(); return roll; }
    // Cached getters: assume updateIMU() already called in this loop.
    float getPitchCached() { return pitch; }
    float getRollCached() { return roll; }
    float getYawCached() {
        if (isnan(newYaw)) return read_Yaw;
        float yaw = newYaw;
        if (yaw < 0) yaw += 360;
        if (yaw > 360) yaw -= 360;
        read_Yaw = yaw;
        return yaw;
    }
    void updateContinuousYawCached() {
      static float lastYaw = 0;
      static bool firstRun = true;

      float currentYaw = getYawCached();
      if (firstRun) {
        continuousYaw = currentYaw;
        lastYaw = currentYaw;
        firstRun = false;
        return;
      }

      float deltaYaw = currentYaw - lastYaw;
      if (deltaYaw > 180.0) deltaYaw -= 360.0;
      if (deltaYaw < -180.0) deltaYaw += 360.0;

      continuousYaw += deltaYaw;
      lastYaw = currentYaw;
    }
    float getOffsetYaw() { return offset_yaw; }
    float getContinuousYaw() { updateContinuousYaw(); return continuousYaw + Ref_offset_Con ; }
    
    void resetGyro() {float y = getContinuousYaw();   
        Ref_offset_Con = 180.0f - y;
        continuousYaw = y;  }
    void resetContinuousYaw() { continuousYaw = getYaw(); }
}

using namespace PuppyBotIMU;  // ทำให้เรียกใช้งานง่ายขึ้นโดยไม่ต้องพิมพ์ PuppyBotIMU::

#endif  // _PUPPYBOT_IMU_H


