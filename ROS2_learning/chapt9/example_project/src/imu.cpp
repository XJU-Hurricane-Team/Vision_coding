#include "Wire.h" // 引入Wire库，用于I2C通信
#include <MPU6050_light.h> // 引入MPU6050库，用于与MPU6050传感器通信

MPU6050 mpu(Wire);   // 创建MPU6050对象，使用Wire对象进行通信
unsigned long timer = 0; // 用于计时的变量

void setup() {
    Serial.begin(115200);  // 初始化串口通信,波特率为115200
    Wire.begin(18,19); // 初始化I2C通信，SDA连接18号引脚，SCL连接19号引脚
    byte status = mpu.begin();  //启动MPU6050传感器并获取状态
    Serial.print(F("MPU6050 status:"));
    Serial.println(status);

    while(status!=0){}  // 如果无法连接到MPU6050,停止一切
    Serial.println(F("Calculating offsets,do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(); // 计算陀螺仪和加速度计的偏移量
    Serial.println("Done!\n");
}

void loop(){
    mpu.update(); // 更新MPU6050传感器的数据
    if ((millis() - timer)> 10 ){
        // 每10 ms输出一次数据
        Serial.print("\tX : ");
        // 输出x轴的倾斜角度
        Serial.print(mpu.getAngleX());
        Serial.pirnt("\tY : ");
        // 输出y轴的倾斜角度
        Serial.print(mpu.getAngleY());
        Serial.print("\tZ : ");
        // 输出z轴的旋转角度
        Serial.println(mpu.getAngleZ());
        timer = millis();

    }
}