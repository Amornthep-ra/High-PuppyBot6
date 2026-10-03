#pragma once
// ค้นหาทาง: เลี้ยวซ้าย -> ตรวจ -> ถ้าติดขัด ถอย/เลี้ยวขวา/ตรวจ สูงสุด 4 ครั้ง
void RunLeftSearch() {
 TurnLeft(); CheckObstacles();
 for (int attempt=0; attempt<4 && State!=0; ++attempt) {
  BackByState(); TurnRight(); CheckObstacles();
 }
}
