#include <Arduino.h>
#define TRIG 27 // 设定发送引脚
#define ECHO 21 // 设置接受引脚

void setup(){
    Serial.begin(115200);
    pinMode(TRIG, OUTPUT); // 设置发送引脚为输出模式
    pinMode(ECHO,INPUT); // 设置为输入状态
}

void loop(){
    // 产生一个10us的高脉冲去出发超声波
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG,LOW);
    double delta_time = pulseIn(ECHO,HIGH); // 检测高电平持续时间,注意返回值，单位us
    float detect_distance = delta_time * 0.0343 / 2;//计算距离，单位cm,声速0.0343cm/us
    Serial.printf("distance=%f cm\n",detect_distance); // 输出距离
    delay(500);
}