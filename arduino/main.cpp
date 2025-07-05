// SPDX-FileCopyrightText: 2025 LICO-META
//
// SPDX-License-Identifier: GPL-3.0-only

// 引入所需的库
#include <SPI.h>
#include "max6675.h"

// 定义 MAX6675 的引脚
const int thermoDO = 19;  // SO (Serial Out, 数据输出, 根据实际连接调整)
const int thermoCS = 5;   // CS (Chip Select, 片选, 根据实际连接调整)
const int thermoCLK = 18; // SCK (Serial Clock, 时钟, 根据实际连接调整)

// 定义继电器引脚
const int relayPin = 23; // 用于控制固态继电器 (SSR) 的引脚, 根据实际连接调整

// 使用定义的引脚初始化 MAX6675 对象
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// --- 移动平均滤波器设置 ---
const int numReadings = 10; // 用于平均的样本数，可以调整
float readings[numReadings];  // 存储历史读数的数组
int readIndex = 0;            // 当前读数的索引
float total = 0;              // 历史读数的总和
float average = 0;            // 平均温度

void setup() {
  // 启动串口通信，波特率为 115200
  Serial.begin(115200);
  
  // 设置继电器引脚为输出，并默认关闭
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  delay(500);
  
  // 用初始读数填充滤波器数组，避免启动时平均值为0
  for (int i = 0; i < numReadings; i++) {
    readings[i] = thermocouple.readCelsius();
    total = total + readings[i];
    delay(250); // 等待传感器转换
  }
  average = total / numReadings;
  
  Serial.println("Filter initialized. Sending data...");
}

void loop() {
  // --- 检查来自计算机的控制指令 ---
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '1') {
      digitalWrite(relayPin, HIGH); // 指令 '1'：打开继电器
    } else if (command == '0') {
      digitalWrite(relayPin, LOW);  // 指令 '0'：关闭继电器
    }
  }

  // --- 温度测量与平滑 ---
  // 从总和中减去最旧的读数
  total = total - readings[readIndex];
  // 读取新温度并存入数组
  readings[readIndex] = thermocouple.readCelsius();
  // 将新读数加到总和中
  total = total + readings[readIndex];
  // 索引前进到下一个位置
  readIndex = readIndex + 1;

  // 如果索引超出数组范围，则回到开头
  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  // 计算新的平均值
  average = total / numReadings;

  // 只打印浮点数值，方便外部程序解析
  Serial.println(average);

  // 每 250ms 发送一次数据, 发送过快会影响温度传感器稳定性
  delay(250);
}