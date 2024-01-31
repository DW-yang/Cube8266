#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

#define WIFI_TIMEOUTms 5000 // WiFi连接超时时间
// 电机驱动板引脚
#define INT1 4  // D2
#define INT2 14 // D5
#define INT3 12 // D6
#define INT4 13 // D7
#define HEARTBEATINTV 1000  // 心跳包发送间隔
uint8_t controller[] = {0xA0, 0x76, 0x4E, 0x36, 0xE3, 0x6C};  // 控制端MAC地址

// 消息结构体
typedef struct cube_message
{
  bool isCMD;
  uint8_t data[8];
} cube_message;

// global variables:
cube_message msg;
unsigned long lastHeartBeat;
bool isControllerOnline = false;
unsigned long prevMillis = 0;

// function declarations:
void onDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len)
{
  memcpy(&msg, incomingData, sizeof(msg));
}

void onDataSend(uint8_t *mac_addr, uint8_t status)
{
  if (status == 0){
    isControllerOnline = true;
    lastHeartBeat = millis();
  }
  else {
    isControllerOnline = false;
  }
}

void setup()
{
  // 初始化串口:
  Serial.begin(115200);
  delay(100);
  Serial.println("Cube8266 power on.");
  // pin设置:
  pinMode(INT1, OUTPUT);
  pinMode(INT2, OUTPUT);
  pinMode(INT3, OUTPUT);
  pinMode(INT4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(INT1, LOW);
  digitalWrite(INT2, LOW);
  digitalWrite(INT3, LOW);
  digitalWrite(INT4, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Pin set ok.");
  // espnow配置:
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) // 初始化espnow
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(onDataSend);
  esp_now_add_peer(controller, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("ESPNOW started.");
  Serial.println(WiFi.macAddress());
}

void loop()
{ 
  if (millis() - prevMillis > HEARTBEATINTV){
    prevMillis = millis();
    uint8_t data[] = {"HeartBeat"};
    esp_now_send(controller, data, sizeof(data));
    Serial.println("Send HeartBeat Package.");
    Serial.println(isControllerOnline);
    Serial.println(lastHeartBeat);
    if (!isControllerOnline){
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
  if (!isControllerOnline){
    analogWrite(INT1, 0);
    analogWrite(INT2, 0);
    analogWrite(INT3, 0);
    analogWrite(INT4, 0);
  }
  if (isControllerOnline){
    digitalWrite(LED_BUILTIN, LOW);
  }
  if (isControllerOnline && (!msg.isCMD)){
    analogWrite(INT1, msg.data[0]);
    analogWrite(INT2, msg.data[1]);
    analogWrite(INT3, msg.data[2]);
    analogWrite(INT4, msg.data[3]);
    Serial.printf("Wrote: int1 = %d, int2 = %d, int3 = %d, int4 = %d\n", msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
  }
}
