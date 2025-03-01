#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/can.h"
#include <math.h>

// GPIO Pins
#define LPWM_PIN        GPIO_NUM_23    // PWM สำหรับเดินหน้า
#define RPWM_PIN        GPIO_NUM_22    // PWM สำหรับถอยหลัง
#define ENCODER_A       GPIO_NUM_34    // Rotary Encoder Channel A
#define ENCODER_B       GPIO_NUM_35    // Rotary Encoder Channel B
#define LIMIT_SWITCH_PIN GPIO_NUM_32    // Limit Switch

// Configurations
const int32_t PULSE_PER_REV     = 13100;    // จำนวนพัลส์ต่อรอบ
const float   MAX_ANGLE         = 360.0;    // องศาสูงสุดของมอเตอร์
const float   LIMIT_SWITCH_ANGLE = 0;        // องศาที่ชนลิมิตสวิตช์ (สำหรับ Homing)
float         max_speed         = 55.0;     // ความเร็วสูงสุด (%)
uint16_t      motor_id          = 0x400;    // ID ของมอเตอร์ (0x200: Base, 0x300: Shoulder, 0x400: Elbow)
const float   home_speed        = -30.0;    // ความเร็วกลับ Home (ค่าลบหมายถึงถอยหลัง)
bool          first_reset_angle = false;    // สำหรับรีเซ็ต encoder ครั้งแรก

// PID Parameters
float kp = 48.0;  // Proportional Gain
float ki = 0.0;  // Integral Gain
float kd = 1.0;  // Derivative Gain

// Shared Variables
volatile int32_t encoder_position = 0;  // ตำแหน่งปัจจุบันของมอเตอร์
float target_angle = 0;                 // ตำแหน่งเป้าหมาย
float integral     = 0;                 // ค่า Integral
float previous_error = 0;               // ค่า Error ก่อนหน้า

volatile int encoder_value_a = 0; // ค่า encoder A
volatile int encoder_value_b = 0; // ค่า encoder B
volatile int total_value     = 0; // ผลรวมของ encoder A และ B
volatile int last_A = LOW;
volatile int last_B = LOW;
volatile float angle = 0;         // มุมปัจจุบันที่คำนวณได้
volatile float reset_offset = 0;  // ค่าชดเชยมุม (ใช้สำหรับ Homing)

// State Machine สำหรับการควบคุมมอเตอร์
typedef enum {
  STATE_NORMAL,   // โหมดควบคุม PID ตาม target_angle
  STATE_HOMING    // โหมด Homing: ขับมอเตอร์ถอยหลังจนกว่าลิมิตสวิตช์จะถูกกระตุ้น
} MotorState;

volatile MotorState currentState = STATE_NORMAL;

// Mutex สำหรับป้องกันการเข้าถึงตัวแปรที่แชร์
SemaphoreHandle_t encoderMutex;

// Task Handles
TaskHandle_t encoderTaskHandle;
TaskHandle_t motorControlTaskHandle;
TaskHandle_t communicationTaskHandle;

// Function Prototypes
float calculateAngle(int32_t pulses);
int32_t calculatePulses(float angle);
void setMotorSpeed(float speed);
void calculatePID();
void handleCANMessage(const can_message_t* rx_message);
void startHoming();

//-------------------------------------------------------
//   INTERRUPT SERVICE ROUTINES (ISR)
//-------------------------------------------------------

// ISR สำหรับ Encoder A
void IRAM_ATTR encoder_isr_a() {
  int current_A = gpio_get_level(ENCODER_A);
  int current_B = gpio_get_level(ENCODER_B);
  if (current_A != last_A) {
    if (current_B != current_A) {
      encoder_value_a++;
      total_value++;
    } else {
      encoder_value_a--;
      total_value--;
    }
  }
  last_A = current_A;
}

// ISR สำหรับ Encoder B
void IRAM_ATTR encoder_isr_b() {
  int current_A = gpio_get_level(ENCODER_A);
  int current_B = gpio_get_level(ENCODER_B);
  if (current_B != last_B) {
    if (current_A == current_B) {
      encoder_value_b++;
      total_value++;
    } else {
      encoder_value_b--;
      total_value--;
    }
  }
  last_B = current_B;
}

//-------------------------------------------------------
//         TASKS
//-------------------------------------------------------

// Task 1: อ่าน Encoder และคำนวณมุม
void encoderTask(void* pvParameters) {
  while (1) {
    xSemaphoreTake(encoderMutex, portMAX_DELAY);
    angle = calculateAngle(total_value) - reset_offset;
    // ถ้า limit switch ถูกกระตุ้นในช่วงเริ่มต้นและยังไม่ได้รีเซ็ต
    if (gpio_get_level(LIMIT_SWITCH_PIN) == 0 && !first_reset_angle) {
      Serial.println("Limit Switch Triggered (Initial Reset)!");
      encoder_value_a = 0;
      encoder_value_b = 0;
      total_value = 0;
      reset_offset = 0;  // ตั้งค่าชดเชยให้ตรงกับมุม 0
      first_reset_angle = true;
    }
    xSemaphoreGive(encoderMutex);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Task 2: ควบคุมมอเตอร์ (รวม PID และ Homing) โดยใช้ state machine
void motorControlTask(void* pvParameters) {
  while (1) {
    if (currentState == STATE_NORMAL) {
      calculatePID();
    }
    else if (currentState == STATE_HOMING) {
      // โหมด Homing: ขับมอเตอร์ถอยหลังจนกว่าลิมิตสวิตช์จะถูกกระตุ้น
      if (gpio_get_level(LIMIT_SWITCH_PIN) == 1) {
        // ถ้ายังไม่ชนลิมิตสวิตช์ ให้ขับมอเตอร์ถอยหลังด้วย home_speed
        int duty = (int)(fabs(home_speed) * (255.0 / 100.0));
        analogWrite(LPWM_PIN, 0);
        analogWrite(RPWM_PIN, duty);
      } else {
        // เมื่อชนลิมิตสวิตช์แล้ว ให้หยุดมอเตอร์และรีเซ็ต encoder
        analogWrite(LPWM_PIN, 0);
        analogWrite(RPWM_PIN, 0);
        Serial.println("Reached Home Position (Homing)");
        xSemaphoreTake(encoderMutex, portMAX_DELAY);
        encoder_value_a = 0;
        encoder_value_b = 0;
        total_value = 0;
        reset_offset = 0;  // เนื่องจากตำแหน่ง Home คือ 0 องศา
        first_reset_angle = true;
        xSemaphoreGive(encoderMutex);
        currentState = STATE_NORMAL;  // กลับสู่โหมดควบคุมตาม target_angle
        target_angle = LIMIT_SWITCH_ANGLE;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Task 3: สื่อสารผ่าน CAN Bus
void communicationTask(void* pvParameters) {
  can_message_t rx_message;
  while (1) {
    if (can_receive(&rx_message, pdMS_TO_TICKS(10)) == ESP_OK) {
      handleCANMessage(&rx_message);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

//-------------------------------------------------------
//      CAN MESSAGE HANDLING
//-------------------------------------------------------

void handleCANMessage(const can_message_t* rx_message) {
  if (rx_message->identifier == motor_id) {
    uint8_t command = rx_message->data[0];
    if (command == 0x01) {  // Set Target Angle
      int16_t raw_value = (rx_message->data[1] << 8) | rx_message->data[2];
      target_angle = raw_value / 100.0;  // แปลงเป็น float
      Serial.printf("Received raw_value: %d\n", raw_value);
      Serial.printf("Set Target Angle: %.2f°\n", target_angle);
    }
    else if (command == 0x02) {  // Send Current Angle
      float current_angle;
      xSemaphoreTake(encoderMutex, portMAX_DELAY);
      current_angle = angle;
      xSemaphoreGive(encoderMutex);
      
      can_message_t tx_message;
      tx_message.flags = CAN_MSG_FLAG_NONE;
      tx_message.identifier = motor_id;
      tx_message.data_length_code = 3;
      tx_message.data[0] = 0x02;
      uint16_t encoded_angle = (uint16_t)(current_angle * 100);
      tx_message.data[1] = (encoded_angle >> 8) & 0xFF;
      tx_message.data[2] = encoded_angle & 0xFF;
      
      if (can_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK) {
        Serial.printf("Sent Current Angle: %.2f°\n", current_angle);
      } else {
        Serial.println("Failed to Send CAN Message");
      }
    }
    else if (command == 0x03) {  // Home Command
      Serial.println("Home Command Received");
      startHoming();
    }
  }
}

// เริ่มกระบวนการ Homing (เปลี่ยน state)
void startHoming() {
  Serial.println("Starting Homing Process...");
  // ตั้งค่าให้ Homing ทำงานได้แม้จะมีการรีเซ็ตใน encoderTask
  first_reset_angle = false;
  currentState = STATE_HOMING;
}

//-------------------------------------------------------
//         PID CALCULATION
//-------------------------------------------------------

void calculatePID() {
  float current_angle;
  xSemaphoreTake(encoderMutex, portMAX_DELAY);
  current_angle = angle;
  xSemaphoreGive(encoderMutex);
  
  // คำนวณ error จาก target_angle และ current_angle
  float error = target_angle - current_angle;
  
  // ตัวอย่างนี้ใช้ integral แบบพื้นฐาน (ไม่สะสมตามเวลา)
  integral = error;
  float derivative = error - previous_error;
  previous_error = error;
  
  float output = kp * error + ki * integral + kd * derivative;
  
  // จำกัด output ให้อยู่ในช่วง ±max_speed
  if (output > max_speed) output = max_speed;
  if (output < -max_speed) output = -max_speed;
 
  Serial.print(">");           // ดูpid
  setMotorSpeed(output);
  Serial.print("Angle: ");
  Serial.print(current_angle);
  Serial.print(" | Target: ");
  Serial.println(target_angle);
}

//-------------------------------------------------------
//         MOTOR CONTROL
//-------------------------------------------------------

void setMotorSpeed(float speed) {
  int duty_cycle = (int)(fabs(speed) * (255.0 / 100.0));
  if (speed < -1) {
    // หมุนถอยหลัง
    analogWrite(LPWM_PIN, 0);
    analogWrite(RPWM_PIN, duty_cycle);
  }
  else if (speed > 1) {
    // หมุนเดินหน้า
    analogWrite(LPWM_PIN, duty_cycle);
    analogWrite(RPWM_PIN, 0);
  }
  else {
    // หยุดมอเตอร์เมื่อค่า speed ใกล้ 0
    analogWrite(LPWM_PIN, 0);
    analogWrite(RPWM_PIN, 0);
  }
}

//-------------------------------------------------------
//       UTILITY FUNCTIONS
//-------------------------------------------------------

float calculateAngle(int32_t pulses) {
  return ((float)pulses * MAX_ANGLE) / PULSE_PER_REV;
}

int32_t calculatePulses(float angle) {
  return (int32_t)((angle * PULSE_PER_REV) / MAX_ANGLE);
}

// หากต้องการใช้งาน LEDC (สำหรับ ESP32) สามารถใช้ฟังก์ชันนี้แทน analogWrite
void setupLEDC() {
  ledcSetup(0, 5000, 8);  // Channel 0, Frequency 5kHz, Resolution 8-bit
  ledcAttachPin(LPWM_PIN, 0);
  
  ledcSetup(1, 5000, 8);  // Channel 1, Frequency 5kHz, Resolution 8-bit
  ledcAttachPin(RPWM_PIN, 1);
}

//-------------------------------------------------------
//         SETUP & LOOP
//-------------------------------------------------------

void setup() {
  Serial.begin(115200);
  
  // ตั้งค่าขา PWM
  gpio_set_direction(LPWM_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(RPWM_PIN, GPIO_MODE_OUTPUT);
  
  // ตั้งค่าขา Encoder และ Limit Switch
  gpio_set_direction(ENCODER_A, GPIO_MODE_INPUT);
  gpio_set_pull_mode(ENCODER_A, GPIO_PULLUP_ONLY);
  gpio_set_direction(ENCODER_B, GPIO_MODE_INPUT);
  gpio_set_pull_mode(ENCODER_B, GPIO_PULLUP_ONLY);
  gpio_set_direction(LIMIT_SWITCH_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(LIMIT_SWITCH_PIN, GPIO_PULLUP_ONLY);
  
  // Attach Interrupts สำหรับ Encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoder_isr_b, CHANGE);
  
  // Initialize CAN Bus
  can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(GPIO_NUM_26, GPIO_NUM_27, CAN_MODE_NORMAL);
  can_timing_config_t t_config  = CAN_TIMING_CONFIG_500KBITS();
  can_filter_config_t f_config   = CAN_FILTER_CONFIG_ACCEPT_ALL();
  
  if (can_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to Install CAN Driver");
    return;
  }
  
  if (can_start() != ESP_OK) {
    Serial.println("Failed to Start CAN Driver");
    return;
  }
  
  // สร้าง Mutex
  encoderMutex = xSemaphoreCreateMutex();
  
  // สร้าง Tasks
  xTaskCreate(encoderTask, "Encoder Task", 1048, NULL, 1, &encoderTaskHandle);
  xTaskCreate(motorControlTask, "Motor Control Task", 2500, NULL, 1, &motorControlTaskHandle);
  xTaskCreate(communicationTask, "Communication Task", 2048, NULL, 1, &communicationTaskHandle);
}

void loop() {
  // ว่างไว้ เนื่องจาก Tasks จัดการทุกงาน
}
