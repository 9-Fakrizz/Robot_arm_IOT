#include "driver/can.h"
#include <math.h>


// CAN Bus Pin Definitions
#define CAN_TX_PIN GPIO_NUM_26  // TX Pin สำหรับส่งข้อมูล
#define CAN_RX_PIN GPIO_NUM_27  // RX Pin สำหรับรับข้อมูล


float x = 0.0;  // Test X position
float y = 220.0;  // Test Y position
float z = 220.0;   // Test Z position

struct answer {
  float theta0 = 0;
  float theta1 = 0;
  float theta2 = 0;
};

struct link_robot_t {
  float l1 = 220.0;
  float l2 = 220.0;
};

link_robot_t robot_link_1 = {};


// Global Variables สำหรับมุมเป้าหมายของแต่ละ Joint
float theta1, theta2, theta3;
float x_input = 0;
float y_input = 0;
float z_input = 0;

// Function Prototypes
void sendTargetAngleCommand(uint16_t motor_id, float target_angle);
void sendTargetHomeCommand(uint16_t motor_id);
void receiveAcknowledgment(uint16_t motor_id);
void setupCAN();
void handleSerialInput();
void setHomeFlow();

// Setup
void setup() {
    Serial.begin(115200);
    setupCAN();
    Serial.println("Master Ready");
    Serial.println("Enter target position in the format: X,Y,Z");
    Serial.println("Or type 'SH' to initiate Home Flow");
}

// Loop
void loop() {
    handleSerialInput();
    // Optionally: Uncommentเพื่อรอรับ acknowledgment จาก slave
    // receiveAcknowledgment(0x200);
    delay(100); // หน่วงเวลาเล็กน้อย เพื่อให้การอ่าน Serial ทำงานได้ราบรื่น
}

//---------------------------------------------------
// CAN Bus Setup Function
//---------------------------------------------------
void setupCAN() {
    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, CAN_MODE_NORMAL);
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

    Serial.println("CAN Bus Initialized");
}

//---------------------------------------------------
// Serial Input Handling Function
//---------------------------------------------------
char receivedData[30];
void handleSerialInput() {
    // ตรวจสอบว่ามีข้อมูลเข้ามาทาง Serial หรือไม่
    if (Serial.available() > 0) {
        int len = Serial.readBytesUntil('\n', receivedData, sizeof(receivedData) - 1);
        if (len > 0) {
            receivedData[len] = '\0'; // เติม null terminator ให้กับสตริง

            // ตรวจสอบคำสั่ง Home (รองรับ "SH", "SH,1", "SH,2", "SH,3")
            if (strcmp(receivedData, "SH") == 0) {
                setHomeFlow();
                Serial.println("Sent Home Flow Command");
            }
            else if (strcmp(receivedData, "SH,1") == 0) {
                sendTargetHomeCommand(0x200);
                Serial.println("Sent Home Flow Command 1");
            }
            else if (strcmp(receivedData, "SH,2") == 0) {
                sendTargetHomeCommand(0x300);
                Serial.println("Sent Home Flow Command 2");
            }
            else if (strcmp(receivedData, "SH,3") == 0) {
                sendTargetHomeCommand(0x400);
                Serial.println("Sent Home Flow Command 3");
            }
            // ตรวจสอบรูปแบบ input แบบ "X,Y,Z" สำหรับเปลี่ยนมุมทั้ง 3 ตัวพร้อมกัน

            else if (sscanf(receivedData, "%f,%f,%f", &x_input, &y_input, &z_input) == 3) {
                // x_input -= 30;
                // y_input -= 25;
                // z_input -= 15;
                
                answer output_answer_1 = calculate_IK(x_input, y_input, z_input, robot_link_1, 1);
                answer output_answer_2 = calculate_IK(x_input, y_input, z_input, robot_link_1, -1);
                // Print the results
                Serial.println("Inverse Kinematics Results (Solution 1):");
                Serial.print("Theta 0: "); Serial.println(output_answer_1.theta0);
                Serial.print("Theta 1: "); Serial.println(output_answer_1.theta1);
                Serial.print("Theta 2: "); Serial.println(output_answer_1.theta2);
                Serial.println("\nInverse Kinematics Results (Solution 2):");
                Serial.print("Theta 0: "); Serial.println(output_answer_2.theta0);
                Serial.print("Theta 1: "); Serial.println(output_answer_2.theta1);
                Serial.print("Theta 2: "); Serial.println(output_answer_2.theta2);;
                // Verify using Forward Kinematics
                float fk_x, fk_y, fk_z;
                calculate_FK(output_answer_1.theta0, output_answer_1.theta1, output_answer_1.theta2, robot_link_1, fk_x, fk_y, fk_z);
                Serial.println("\nForward Kinematics Check:");
                Serial.print("Calculated X: "); Serial.println(fk_x);
                Serial.print("Calculated Y: "); Serial.println(fk_y);
                Serial.print("Calculated Z: "); Serial.println(fk_z);
                // Compare with original values
                Serial.println("\nError Check:");
                Serial.print("Error in X: "); Serial.println(fabs(fk_x - x_input));
                Serial.print("Error in Y: "); Serial.println(fabs(fk_y - y_input));
                Serial.print("Error in Z: "); Serial.println(fabs(fk_z - z_input));

                float fk_x2, fk_y2, fk_z2;
                calculate_FK(output_answer_2.theta0, output_answer_2.theta1, output_answer_2.theta2, robot_link_1, fk_x2, fk_y2, fk_z2);
                Serial.println("\nForward Kinematics Check:");
                Serial.print("Calculated X2: "); Serial.println(fk_x);
                Serial.print("Calculated Y2: "); Serial.println(fk_y);
                Serial.print("Calculated Z2: "); Serial.println(fk_z);
                //Compare with original values
                Serial.println("\nError Check:");
                Serial.print("Error in X2: "); Serial.println(fabs(fk_x - x_input));
                Serial.print("Error in Y2: "); Serial.println(fabs(fk_y - y_input));
                Serial.print("Error in Z2: "); Serial.println(fabs(fk_z - z_input));
                Serial.println();

                bool allow_flag = false;
                float theta1, theta2, theta3;
                theta1 = output_answer_1.theta0;
                theta2 = output_answer_1.theta1;
                theta3 = output_answer_1.theta2 + theta2;
              

                Serial.printf(" Sol 1 is Base=%.2f°, Shoulder=%.2f°, Elbow=%.2f°\n", theta1, theta2, theta3);
                
                ///// Dead Zone Condition ////
                if(theta1 < -155 || theta1 > 155 ){
                  Serial.println("Dead Zone for Base Link !");
                }
                else{
                  if(theta2 < 40 || theta2 > 115){
                    Serial.println("Dead Zone for Link2 (sol1)!");
                  }
                  else{
                    if(theta2 + theta3 > 91 || theta3 < -15){
                      Serial.println("Dead Zone for Link3 (sol1)!");
                    }
                    else{
                      Serial.printf(" All PASS for Sol 1");
                      allow_flag = true;
                    }
                  }
                }

                if(allow_flag){
                  sendTargetAngleCommand(0x200, theta1);  // Motor 0x200: Base
                  sendTargetAngleCommand(0x400, 90);
                  delay(5000);
                  sendTargetAngleCommand(0x300, theta2);  // Motor 0x300: Shoulder
                  delay(8000);
                  sendTargetAngleCommand(0x400, theta3);  // Motor 0x400: Elbow
                  Serial.printf("Target angle set (Sol 1): Base=%.2f°, Shoulder=%.2f°, Elbow=%.2f°\n", theta1, theta2, theta3);
                }
                else{
                  theta1 = output_answer_2.theta0;
                  theta2 = output_answer_2.theta1;
                  theta3 = output_answer_2.theta2 + theta2;
                

                  Serial.printf(" Sol 2 is Base=%.2f°, Shoulder=%.2f°, Elbow=%.2f°\n", theta1, theta2, theta3);
                  
                  ///// Dead Zone Condition ////
                  if(theta1 < -155 || theta1 > 155 ){
                    Serial.println("Dead Zone for Base Link !");
                  }
                  else{
                    if(theta2 < 40 || theta2 > 115){
                      Serial.println("Dead Zone for Link2 (sol2)!");
                    }
                    else{
                      if(theta3 > 90 || theta3 < -60){
                        Serial.println("Dead Zone for Link3 (sol2)!");
                      }
                      else{
                        Serial.printf(" All PASS for Sol 2");
                        allow_flag = true;
                      }
                    }
                  }

                  if(allow_flag){
                    sendTargetAngleCommand(0x200, theta1);  // Motor 0x200: Base
                    sendTargetAngleCommand(0x400, 90);
                    delay(5000);
                    sendTargetAngleCommand(0x300, theta2);  // Motor 0x300: Shoulder
                    delay(8000);
                    sendTargetAngleCommand(0x400, theta3);  // Motor 0x400: Elbow
                    Serial.printf("Target angle set (Sol 2): Base=%.2f°, Shoulder=%.2f°, Elbow=%.2f°\n", theta1, theta2, theta3);
                  } 
                }
        
            }
            // ตรวจสอบรูปแบบ input แบบ "motor,angle" สำหรับเปลี่ยนมุมทีละตัว
            else {
                int motor;
                float angle;
                if (sscanf(receivedData, "%d,%f", &motor, &angle) == 2) {
                    switch (motor) {
                        case 1:
                            sendTargetAngleCommand(0x200, angle);
                            Serial.printf("Target angle set for Base: %.2f°\n", angle);
                            break;
                        case 2:
                            sendTargetAngleCommand(0x300, angle);
                            Serial.printf("Target angle set for Shoulder: %.2f°\n", angle);
                            break;
                        case 3:
                            sendTargetAngleCommand(0x400, angle);
                            Serial.printf("Target angle set for Elbow: %.2f°\n", angle);
                            break;
                        default:
                            Serial.println("Error: Invalid motor number. Use 1 for Base, 2 for Shoulder, 3 for Elbow");
                            break;
                    }
                }
                else {
                    Serial.println("Error: Invalid input format. Use X,Y,Z or 'SH' or motor,angle");
                }
            }
        }
    }
}

//---------------------------------------------------
// CAN Command Functions
//---------------------------------------------------

// ส่งคำสั่งเป้าหมายองศาไปยังมอเตอร์ที่ระบุ
void sendTargetAngleCommand(uint16_t motor_id, float target_angle) {
    can_message_t message;
    message.flags = CAN_MSG_FLAG_NONE;
    message.identifier = motor_id;       // ใช้ motor_id เป็น CAN ID
    message.data_length_code = 3;          // 3 ไบต์: [Command, High Byte, Low Byte]

    message.data[0] = 0x01;                // Command: Set Target Angle
    // แปลงค่าองศาเป็น 16-bit โดยคูณด้วย 100 เพื่อความแม่นยำ
    uint16_t encoded_angle = (uint16_t)(target_angle * 100);
    message.data[1] = (encoded_angle >> 8) & 0xFF; // High Byte
    message.data[2] = encoded_angle & 0xFF;        // Low Byte

    if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        Serial.printf("Sent Target Angle %.2f° to Motor ID: 0x%X\n", target_angle, motor_id);
    } else {
        Serial.println("Failed to Send Target Angle Command");
    }
}

// ส่งคำสั่ง Home (กลับ Home) ให้กับมอเตอร์ที่ระบุ
void sendTargetHomeCommand(uint16_t motor_id) {
    can_message_t message;
    message.flags = CAN_MSG_FLAG_NONE;
    message.identifier = motor_id;
    message.data_length_code = 1;
    message.data[0] = 0x03;  // Command: Home

    if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        Serial.printf("Sent Home Command to Motor ID: 0x%X\n", motor_id);
    } else {
        Serial.println("Failed to Send Home Command");
    }
}

// ฟังก์ชันส่งคำสั่ง Home Flow ให้กับมอเตอร์ที่ต้องการ (สามารถปรับเปลี่ยนได้ตามลำดับ)
void setHomeFlow() {
    sendTargetHomeCommand(0x200);
    sendTargetHomeCommand(0x400);
    delay(6000);
    sendTargetAngleCommand(0x400,45.0);
    delay(5000);
    sendTargetHomeCommand(0x300);
    delay(10000);
    sendTargetHomeCommand(0x400);

}

// รับข้อความยืนยัน (Acknowledgment) จาก Slave (ตัวอย่าง)
// สามารถนำไปใช้เพิ่มเติมได้หากต้องการตรวจสอบผลการส่งคำสั่ง
void receiveAcknowledgment(uint16_t motor_id) {
    can_message_t rx_message;
    if (can_receive(&rx_message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        if (rx_message.identifier == motor_id) {
            Serial.printf("Acknowledgment received from Motor ID: 0x%X\n", motor_id);
        } else {
            Serial.printf("Unexpected response from ID: 0x%X\n", rx_message.identifier);
        }
    } else {
        Serial.printf("No response from Motor ID: 0x%X\n", motor_id);
    }
}
answer calculate_IK(float x, float y, float z, link_robot_t robot_link_1, int solution) {
  float l1 = robot_link_1.l1;
  float l2 = robot_link_1.l2;

  float w = sqrt(pow(x, 2) + pow(y, 2));
  float a = pow(z, 2) + pow(w, 2) - pow(l1, 2) - pow(l2, 2);
  float b = 2 * l1 * l2;
  float c = a / b;

  // Prevent invalid acos input
  c = constrain(c, -1.0, 1.0);

  float theta2 = acos(c);
  if (solution == -1) theta2 = -theta2; // Alternative solution

  float a1 = l1 + (l2 * cos(theta2));
  float b1 = l2 * sin(theta2);

  float theta1 = atan2((z * a1 - w * b1), (w * a1 + z * b1));
  float theta0 = atan2(x, y);

  // Convert to degrees
  theta0 = theta0 * 180.0 / M_PI;
  theta1 = theta1 * 180.0 / M_PI;
  theta2 = theta2 * 180.0 / M_PI;

  answer output_answer = {};
  output_answer.theta0 = theta0;
  output_answer.theta1 = theta1;
  output_answer.theta2 = theta2;
  return output_answer;
}


void calculate_FK(float theta0, float theta1, float theta2, link_robot_t robot_link_1, float &out_x, float &out_y, float &out_z) {
  float l1 = robot_link_1.l1;
  float l2 = robot_link_1.l2;

  // Convert degrees back to radians
  theta0 = theta0 * M_PI / 180.0;
  theta1 = theta1 * M_PI / 180.0;
  theta2 = theta2 * M_PI / 180.0;

  // Forward Kinematics Equations
  float w = (l1 * cos(theta1)) + (l2 * cos(theta1 + theta2));
  out_x = w * sin(theta0);
  out_y = w * cos(theta0);
  out_z = (l1 * sin(theta1)) + (l2 * sin(theta1 + theta2));

}
