#include "EncoderPCNT.h"
#include "pidController.h"
#include "RTOS.h"
#include <math.h>

#define encoder_pub_speed 10 //ms

EncoderPCNT enc1(GPIO_NUM_13, GPIO_NUM_14, PCNT_UNIT_0);
EncoderPCNT enc2(GPIO_NUM_22, GPIO_NUM_23, PCNT_UNIT_1);

pidController pid1(1.0, 0.5, 0.1);
pidController pid2(1.0, 0.5, 0.1);

class Motor
{
private:
    int pwmPin;
    int pinA;
    int pinB;
    int _speed;

public:
    Motor(int pwm, int a, int b) : pwmPin(pwm), pinA(a), pinB(b) {}

    void motorStop(){
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
        analogWrite(pwmPin, 0);
    }

    void begin(){
        pinMode(pwmPin, OUTPUT);
        pinMode(pinA, OUTPUT);
        pinMode(pinB, OUTPUT);
    }

    void setSpeed(double speed){ /* speed 範圍 -100 到 100 */
        if (speed > 100.0) speed = 100.0;
        if (speed < -100.0) speed = -100.0;
        _speed = (int)speed;
        // 使用浮點映射：輸出範圍 -255 .. 255，並取絕對值給 PWM
        double pwmVal = (speed / 100.0) * 255.0; // -255 .. 255
        int pwm = (int)(fabs(pwmVal) + 0.5);     // magnitude 0..255

        if (speed > 0.0)
        {
            digitalWrite(pinA, HIGH);
            digitalWrite(pinB, LOW);
        }
        else if (speed < 0.0)
        {
            digitalWrite(pinA, LOW);
            digitalWrite(pinB, HIGH);
        }
        else
        {
            digitalWrite(pinA, HIGH);
            digitalWrite(pinB, HIGH);
        }
        analogWrite(pwmPin, pwm);
    }

    int getSpeed()
    {
        return _speed;
    }
};

Motor motor1(4, 17, 16);
Motor motor2(21, 18, 19);

const int enablePin = 5;

bool is_enabled = false;

void motor_contorl_task(void *param)
{
    const double dt = 0.1; // 控制迴圈時間間隔（秒）
    const double setpoint1 = 100.0; // 馬達1目標速度
    const double setpoint2 = 100.0; // 馬達2目標速度

    while (true)
    {
        // 獲取編碼器計數
        int32_t count1 = enc1.get_count();
        int32_t count2 = enc2.get_count();

        // 計算實際速度（假設每次迴圈為 dt 秒）
        double actualSpeed1 = (double)count1 / dt;
        double actualSpeed2 = (double)count2 / dt;

        // 使用 PID 控制器計算輸出
        double output1 = pid1.compute(setpoint1, actualSpeed1, 255, -255, dt);
        double output2 = pid2.compute(setpoint2, actualSpeed2, 255, -255, dt);

        // 設置馬達速度
        motor1.setSpeed(output1);
        motor2.setSpeed(output2);

        // 重置編碼器計數
        enc1.resetEncoder();
        enc2.resetEncoder();

        vTaskDelay(pdMS_TO_TICKS((int)(dt * 1000))); // 延遲 dt 毫秒
    }
}

void encoder_read_task(void *param)
{   
    while (true)
    {
        while (is_enabled)
        {
            int32_t count1 = enc1.get_count();
            int32_t count2 = enc2.get_count();
            Serial.printf("%d,%d\n", count1, count2);
            vTaskDelay(pdMS_TO_TICKS(encoder_pub_speed));   // 每 10 ms讀取一次編碼器數值
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    
}

void set_motor_speed_task(void *param)
{
    while (true)
    {
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            String command = input.substring(0, 3);
            if (command == "sta" && !is_enabled){
                digitalWrite(enablePin, HIGH);
                enc1.resetEncoder();
                enc2.resetEncoder();
                Serial.println("sta");
                is_enabled = true;
            }

            else if (command == "end" && is_enabled){
                is_enabled = false;
                motor1.motorStop();
                motor2.motorStop();
                digitalWrite(enablePin, LOW);
            }
            else if (command == "res"){
                    enc1.resetEncoder();
                    enc2.resetEncoder();
                }

            else if (command == "sms"){
                int commaIndex = input.indexOf(',', 4);
                if (commaIndex != -1)
                {
                    String speed1Str = input.substring(4, commaIndex);
                    String speed2Str = input.substring(commaIndex + 1);
                    double speed1 = speed1Str.toDouble();
                    double speed2 = speed2Str.toDouble();
                    motor1.setSpeed(speed1);
                    motor2.setSpeed(speed2);
                }
            }
            else {
                //Serial.println("Unknown command");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void feedback_motorSpeed(void *param)
{
    while (true)
    {
        Serial.printf("Motor1 Speed: %d, Motor2 Speed: %d\n", motor1.getSpeed(), motor2.getSpeed());
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void setup()
{
    Serial.begin(115200);
    enc1.begin();
    enc2.begin();
    motor1.begin();
    motor2.begin();
    pinMode(enablePin, OUTPUT);

    // xTaskCreatePinnedToCore(motor_contorl_task, "Motor Control Task", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(encoder_read_task, "Encoder Read Task", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(set_motor_speed_task, "Set Motor Speed Task", 2048, NULL, 1, NULL, 1);
    // xTaskCreatePinnedToCore(feedback_motorSpeed, "Feedback Motor Speed Task", 2048, NULL, 1, NULL, 1);
}

void loop()
{
}