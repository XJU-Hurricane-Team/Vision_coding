#include <Arduino.h>
#include <Esp32McpwmMotor.h>

Esp32McpwmMotor motor; //创建一个名为motor的对象，用于控制电动机

void setup() {
    motor.attachMotor(0,22,23); //将电动机0连接到引脚22和引脚23
    motor.attachMotor(1,12,13); //将电动机1链接到引脚12和引脚13

}
void loop()
{
    motor.updateMotorSpeed(0,70); //设置电动机0的速度（占空比）为负70%
    motor.updateMotorSpeed(1,70); //设置电动机1的速度（占空比）为正70%
    delay(2000); //延迟2s
    motor.updateMotorSpeed(0,-70); // 设置电动机0的速度（占空比）为正70%
    motor.updateMotorSpeed(1,-70); // 设置电动机1的速度（占空比）为负70%
    delay(2000); // 延迟2s
}