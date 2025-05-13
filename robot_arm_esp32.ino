#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include "armDriver.h"

// 馬達控制腳位
const int IN1_FrontLeft = 18, IN2_FrontLeft = 19;
const int IN3_FrontRight = 16, IN4_FrontRight = 17;
const int IN1_RearLeft = 25, IN2_RearLeft = 26;
const int IN3_RearRight = 32, IN4_RearRight = 33;

// 手臂參數
#define UPDATE_ARM_DELAY 1.0
#define SERIAL_READ_DELAY 10
#define SERIAL_WRITE_DELAY 250
const uint8_t NUM_OF_SERVOS = 5;
const uint8_t servoMinAngles[] = {0, 0, 0, 0, 0};
const uint8_t servoMaxAngles[] = {180, 120, 160, 180, 70};

float currentAngles[NUM_OF_SERVOS];
uint8_t targetAngles[NUM_OF_SERVOS] = {50, 30, 40, 60, 60};

// Task Handles
TaskHandle_t armControlTask, serialCommunicationTask, serialWriterTask;

// 前進
void moveForward() {
  digitalWrite(IN1_FrontLeft, HIGH);  digitalWrite(IN2_FrontLeft, LOW);
  digitalWrite(IN3_FrontRight, HIGH); digitalWrite(IN4_FrontRight, LOW);
  digitalWrite(IN1_RearLeft, HIGH);   digitalWrite(IN2_RearLeft, LOW);
  digitalWrite(IN3_RearRight, HIGH);  digitalWrite(IN4_RearRight, LOW);
}

// 後退
void moveBackward() {
  digitalWrite(IN1_FrontLeft, LOW);  digitalWrite(IN2_FrontLeft, HIGH);
  digitalWrite(IN3_FrontRight, LOW); digitalWrite(IN4_FrontRight, HIGH);
  digitalWrite(IN1_RearLeft, LOW);   digitalWrite(IN2_RearLeft, HIGH);
  digitalWrite(IN3_RearRight, LOW);  digitalWrite(IN4_RearRight, HIGH);
}

// 左轉
void turnLeft() {
  digitalWrite(IN1_FrontLeft, LOW);  digitalWrite(IN2_FrontLeft, HIGH);
  digitalWrite(IN3_FrontRight, LOW); digitalWrite(IN4_FrontRight, HIGH);
  digitalWrite(IN1_RearLeft, HIGH);   digitalWrite(IN2_RearLeft, LOW);
  digitalWrite(IN3_RearRight, HIGH);  digitalWrite(IN4_RearRight, LOW);
}

// 右轉
void turnRight() {
  digitalWrite(IN1_FrontLeft, HIGH);  digitalWrite(IN2_FrontLeft, LOW);
  digitalWrite(IN3_FrontRight, HIGH); digitalWrite(IN4_FrontRight, LOW);
  digitalWrite(IN1_RearLeft, LOW);   digitalWrite(IN2_RearLeft, HIGH);
  digitalWrite(IN3_RearRight, LOW);  digitalWrite(IN4_RearRight, HIGH);
}

// 停止
void stopAll() {
  digitalWrite(IN1_FrontLeft, LOW);  digitalWrite(IN2_FrontLeft, LOW);
  digitalWrite(IN3_FrontRight, LOW); digitalWrite(IN4_FrontRight, LOW);
  digitalWrite(IN1_RearLeft, LOW);   digitalWrite(IN2_RearLeft, LOW);
  digitalWrite(IN3_RearRight, LOW);  digitalWrite(IN4_RearRight, LOW);
}

// 手臂控制任務
void armControlTaskFunction(void *parameter) {
  ArmManager armManager(NUM_OF_SERVOS, servoMinAngles, servoMaxAngles);
  for (;;) {
    for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
      armManager.setServoTargetAngle(i, targetAngles[i]);
    }
    armManager.moveArm();
    armManager.getCurrentAngles(currentAngles);
    vTaskDelay(UPDATE_ARM_DELAY / portTICK_PERIOD_MS);
  }
}

// Serial 接收任務
void serialCommunicationTaskFunction(void *parameter) {
  String input = "";
  const uint8_t armPose1[NUM_OF_SERVOS] = {50, 100, 60, 60, 60}; // 前手臂放下
  const uint8_t armPose2[NUM_OF_SERVOS] = {50, 30, 40, 60, 60};  // 手臂歸位
  const uint8_t armPose3[NUM_OF_SERVOS] = {50, 30, 120, 60, 60}; // 後手臂放下

  for (;;) {
    if (Serial.available()) {
      input = Serial.readStringUntil('\n');

      // ✅ 單字元快捷鍵判斷
      if (input.length() == 1) {
        char cmd = input.charAt(0);
        switch (cmd) {
          case 'w': moveForward(); break;
          case 's': moveBackward(); break;
          case 'a': turnLeft(); break;
          case 'd': turnRight(); break;
          case 'z': stopAll(); break;
          case '3': memcpy(targetAngles, armPose1, NUM_OF_SERVOS); break;
          case '2': memcpy(targetAngles, armPose2, NUM_OF_SERVOS); break;
          case '1': memcpy(targetAngles, armPose3, NUM_OF_SERVOS); break;
        }

        continue; // 不做 JSON 處理
      }

      // ✅ JSON 判斷邏輯維持不變
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, input);
      if (error) {
        Serial.print("JSON error: "); Serial.println(error.c_str());
      } else {
        if (doc.containsKey("servo_target_angles")) {
          JsonArray angles = doc["servo_target_angles"].as<JsonArray>();
          if (angles.size() == NUM_OF_SERVOS) {
            for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
              targetAngles[i] = angles[i];
            }
          }
        }
        if (doc.containsKey("move")) {
          String action = doc["move"];
          if (action == "forward") moveForward();
          else if (action == "backward") moveBackward();
          else stopAll();
        }
      }
    }
    vTaskDelay(SERIAL_READ_DELAY / portTICK_PERIOD_MS);
  }
}


// Serial 傳送任務
void serialWriterTaskFunction(void *parameter) {
  for (;;) {
    StaticJsonDocument<256> doc;
    JsonArray angles = doc.createNestedArray("servo_current_angles");
    for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
      angles.add(currentAngles[i]);
    }
    String out;
    serializeJson(doc, out);
    Serial.println(out);
    vTaskDelay(SERIAL_WRITE_DELAY / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  // 輪子腳位初始化
  pinMode(IN1_FrontLeft, OUTPUT);  pinMode(IN2_FrontLeft, OUTPUT);
  pinMode(IN3_FrontRight, OUTPUT); pinMode(IN4_FrontRight, OUTPUT);
  pinMode(IN1_RearLeft, OUTPUT);   pinMode(IN2_RearLeft, OUTPUT);
  pinMode(IN3_RearRight, OUTPUT);  pinMode(IN4_RearRight, OUTPUT);
  stopAll();

  // 初始化手臂角度
  for (uint8_t i = 0; i < NUM_OF_SERVOS; i++) {
    currentAngles[i] = servoMinAngles[i];
  }

  // 建立三個任務
  xTaskCreatePinnedToCore(armControlTaskFunction, "ArmControl", 2048, NULL, 1, &armControlTask, 0);
  xTaskCreatePinnedToCore(serialCommunicationTaskFunction, "SerialComm", 2048, NULL, 2, &serialCommunicationTask, 1);
  xTaskCreatePinnedToCore(serialWriterTaskFunction, "SerialWriter", 2048, NULL, 1, &serialWriterTask, 0);
}

void loop() {
  // 不需要執行任何程式碼
}
