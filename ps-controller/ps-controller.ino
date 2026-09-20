#include <Arduino.h>
#include <SPI.h>
#include "mcp_canbus.h"
#include "driver/twai.h"
#include "ps_logic.h"

MCP_CAN CAN(CS);

unsigned long _lastPumpHearbeat = 0;

// Keep alive fields
const unsigned long _keepAliveIntervalMs = 2000; // 2160?
unsigned long _lastKeepAliveTs = 0;
uint8_t _lastKeepAliveCounterIndex = 0; // To increment each time a keep alive is sent

// Status send
const unsigned long _statusSendIntervalMs = 200;
unsigned long _lastStatusSendTs = 0;

// Haltech duty cycle
double _dutyCycle = 80.0;
unsigned long _lastHaltechTs = 0;

// Pump speed
const unsigned long _pumpSendIntervalMs = 72;
unsigned long _lastPumpSendTs = 0;
unsigned short _lastPumpSpeed; // 0 - 6000, 1 is full speed

void setup() {
  Serial.begin(115200);
  //while (!Serial)
  //  ;
  Serial.println("Starting...");
  pinMode(LED_BUILTIN, OUTPUT);

  initCanBus();
}

bool initCanBus() {
  Serial.println("Initializing builtin CAN peripheral");
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN1_TX, (gpio_num_t)CAN1_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if(twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("CAN1 Driver initialized");
  } else {
    Serial.println("Failed to initialze CAN1 driver");
    return false;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("CAN1 interface started");
  } else {
    Serial.println("Failed to start CAN1");
    return false;
  }

  // uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
  // if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
  //   Serial.println("CAN1 Alerts reconfigured");
  // } else {
  //   Serial.println("Failed to reconfigure alerts");
  //   return false;
  // }

  if (CAN_OK == CAN.begin(CAN_1000KBPS)) {
    Serial.println("CAN2 interface started");
  } else {
    Serial.println("Failed to start CAN2");
    while (1);
  }

  return true;
}

void loop() {
  unsigned long currentTs = millis();

  // Send pump keep alive every 2 seconds
  if ((currentTs - _lastKeepAliveTs) >= _keepAliveIntervalMs) {
    sendPumpKeepAlive();
    _lastKeepAliveTs = currentTs;
  }

  // Receive pump data
  rxHeartbeat();
  bool isPumpOnline = getPumpOnline();

  // Receive haltech data
  rxHaltechDutyCycle();
  bool isHaltechOnline = getHaltechOnline();

  // Send status update
  if ((currentTs - _lastStatusSendTs) >= _statusSendIntervalMs) {
    sendControllerStatus(isPumpOnline, isHaltechOnline);
    _lastStatusSendTs = currentTs;
  }

  if (!isPumpOnline || !isHaltechOnline) {
    delay(10);
    return;
  } 

  // Determine pump speed
  _lastPumpSpeed = psConvertDutyCycle(_dutyCycle);

  // Send speed to pump every 72ms
  currentTs = millis();
  if ((currentTs - _lastPumpSendTs) >= 72) {
    _lastPumpSendTs = currentTs;
    sendPumpSpeed(_lastPumpSpeed);
  }

  delay(1);
}

/**
 * @brief  Receives heartbeat message from PS pump on CAN 1.
 */
static void rxHeartbeat() {
  digitalWrite(LED_BUILTIN, HIGH);
  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.identifier == PS_CAN_ID_PUMP_HEARTBEAT) {
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print("CAN1 RX PS:");
      printCanData(message.data_length_code, message.data);
      _lastPumpHearbeat = millis();
    }
  }
}

static bool getPumpOnline() {
  return psIsOnline(_lastPumpHearbeat, millis());
}

/**
 * @brief  Sends the rotating keep alive message to the PS pump on CAN 1.
 */
static void sendPumpKeepAlive() {
  twai_message_t message = {};
  message.identifier = PS_CAN_ID_PUMP_KEEP_ALIVE;
  message.extd = 1;
  message.rtr = 0;
  message.data_length_code = PS_PUMP_FRAME_LEN;

  // Build the frame with the current counter value in byte 0.
  psBuildKeepAliveFrame(message.data, _lastKeepAliveCounterIndex);

  // Rotate through the 4 values
  _lastKeepAliveCounterIndex = psNextKeepAliveIndex(_lastKeepAliveCounterIndex);

  // Send the message
  if (twai_transmit(&message, pdMS_TO_TICKS(50)) == ESP_OK) {
    Serial.print("CAN1: sent pump keep alive ");
    Serial.println(message.data[0], HEX);
  } else {
    Serial.println("CAN1: failed to send pump keep alive");
  }
}

/**
 * @brief  Sends status such as to AIM for a display on CAN 2.
 */
static void sendControllerStatus(bool isPumpOnline, bool isHaltechOnline) {
  uint8_t msg[PS_STATUS_FRAME_LEN] = { 0, 0, 0, 0, 0 };
  psBuildStatusFrame(msg, isPumpOnline, isHaltechOnline, _dutyCycle, _lastPumpSpeed);

  if (CAN_OK == CAN.sendMsgBuf(PS_CAN_ID_STATUS, 1, PS_STATUS_FRAME_LEN, msg)) {
    //Serial.println("CAN2: sent status");
    Serial.printf("CAN2: sent status:");
    printCanData(PS_STATUS_FRAME_LEN, msg);
  } else {
    Serial.println("CAN2: Failed to send status");
  }
}

/**
 * @brief  Receives duty cycle from Haltech IO Box A on CAN 2.
 */
static bool rxHaltechDutyCycle() {
  unsigned char len = 0;
  unsigned char buff[8];

  while (CAN_MSGAVAIL == CAN.checkReceive()) {
    //Serial.print("CAN2: Received ");

    CAN.readMsgBuf(&len, buff);
    unsigned long id = CAN.getCanId();
    //Serial.printf("packet with id 0x%x", id); 
    //printCanData(len, buff);

    // IO Box A DPO 1
    // https://www.ptmotorsport.com.au/how-to-get-can-messages-into-haltech-elite-and-nexus-ecus/
    if (id == PS_CAN_ID_HALTECH_DPO1) {
      //Serial.printf("packet with id 0x%x", id); 
      //printCanData(8, buff);
      _dutyCycle = psDecodeHaltechDutyCycle(buff[0]);
      //Serial.print("Raw Duty Cycle = ");
      //Serial.printf(" %02X ", buff[0]);
      //Serial.printf(" %02X ", buff[1]);
      Serial.print("Duty Cycle = ");
      Serial.println(_dutyCycle); 
      _lastHaltechTs = millis();
      return true;
    }
  }

  return false;
}

static bool getHaltechOnline() {
  return psIsOnline(_lastHaltechTs, millis());
}

/**
 * @brief  Sends pump value to the power steering pump on CAN 1.
 */
static void sendPumpSpeed(unsigned short speed){
  twai_message_t message = {};
  message.identifier = PS_CAN_ID_PUMP_SPEED;
  message.extd = 1;
  message.rtr = 0;
  message.data_length_code = PS_PUMP_FRAME_LEN;
  psBuildSpeedFrame(message.data, speed);

  if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.print("CAN1: sent pump speed ");
    Serial.println(speed);
  } else {
    Serial.println("CAN1: Failed send pump speed");
  }
}

static void printCanData(unsigned char len, unsigned char buff[8]) {
  for (int i = 0; i < len; i++) {
      Serial.printf(" %02X", buff[i]);
  }
  Serial.println();
}
