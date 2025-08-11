#include <Arduino.h>
#include <SPI.h>
#include "mcp_canbus.h"
#include "driver/twai.h"

MCP_CAN CAN(CS);

unsigned long _lastPumpHearbeat = 0;

// Keep alive fields
const unsigned long _keepAliveIntervalMs = 2000; // 2160?
unsigned long _lastKeepAliveTs = 0;
uint8_t _keepAliveFrame[8] = { 0x00, 0x00, 0x22, 0xE0, 0x41, 0x90, 0x00, 0x00 };
uint8_t _keepAliveCounterValues[4] = { 0x00, 0x40, 0x80, 0xC0 };
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
uint8_t _speedFrame[8] = { 0xBB, 0x00, 0x3F, 0xFF, 0x06, 0xE0, 0x00, 0x00 }; // Bytes 6 and 7 are the speed

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
  _lastPumpSpeed = convertDutyCycle(_dutyCycle);

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
    if (message.identifier == 0x1B200002) {
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print("CAN1 RX PS:");
      printCanData(message.data_length_code, message.data);
      _lastPumpHearbeat = millis();
    }
  }
}

static bool getPumpOnline() {
  if (_lastPumpHearbeat == 0) return false;
  return (millis() - _lastPumpHearbeat) < 1000;
}

/**
 * @brief  Sends the rotating keep alive message to the PS pump on CAN 1.
 */
static void sendPumpKeepAlive() {
  twai_message_t message;
  message.identifier = 0x1AE0092C;
  message.extd = 1;
  message.rtr = 0;
  message.data_length_code = 8;
  memcpy(message.data, _keepAliveFrame, sizeof(_keepAliveFrame));

  // Update the first byte to the next keep alive value. 
  message.data[0] = _keepAliveCounterValues[_lastKeepAliveCounterIndex];
  
  // Rotate through the 4 values
  _lastKeepAliveCounterIndex++;
  if (_lastKeepAliveCounterIndex > 3) 
    _lastKeepAliveCounterIndex = 0;

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
  uint8_t msg[5] = { 0, 0, 0, 0, 0 };
  
  // Status byte
  if (isPumpOnline && isHaltechOnline)
    msg[0] = 1;
  else if (!isPumpOnline && !isHaltechOnline)
    msg[0] = 2;
  else if (!isPumpOnline)
    msg[0] = 3;
  else if (!isHaltechOnline)
    msg[0] = 4;
  else
    msg[0] = 0;

  // Duty Cycle
  uint16_t value = (uint16_t)(_dutyCycle * 10);
  msg[1] = (uint8_t)((value & 0xFF00) >> 8);
  msg[2] = (uint8_t)((value & 0x00FF));

  // Pump value
  msg[3] = (uint8_t)((_lastPumpSpeed & 0xFF00) >> 8);
  msg[4] = (uint8_t)((_lastPumpSpeed & 0x00FF));

  if (CAN_OK == CAN.sendMsgBuf(0x100D0001, 1, 5, msg)) {
    //Serial.println("CAN2: sent status");
    Serial.printf("CAN2: sent status:");
    printCanData(5, msg);
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
    if (id == 0x2D0) {
      //Serial.printf("packet with id 0x%x", id); 
      //printCanData(8, buff);
      _dutyCycle = ((double)buff[0]) / 2.5;
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
  if (_lastHaltechTs == 0) return false;
  return (millis() - _lastHaltechTs) < 1000;
}

/**
 * @brief  Gets the pump value from duty cycle percentage where 1 is full on.
 */
static uint16_t convertDutyCycle(double dutyCycle) {
    if (dutyCycle > 100.0) dutyCycle = 100.0; // clamp to max
    if (dutyCycle < 0.0) dutyCycle = 0.0;     // clamp to min

    if (dutyCycle == 0.0)
        return 0;

    // Map 0–100% -> 1–6000, inverted
    uint16_t value = 1 + (uint16_t)((6000 - 1) * (1.0 - dutyCycle / 100.0));
    return value;
}

/**
 * @brief  Sends pump value to the power steering pump on CAN 1.
 */
static void sendPumpSpeed(unsigned short speed){
  twai_message_t message;
  message.identifier = 0x02104136;
  message.extd = 1;
  message.rtr = 0;
  message.data_length_code = 8;
  memcpy(message.data, _speedFrame, sizeof(_speedFrame));
  message.data[6] = (uint8_t)((speed & 0xFF00) >> 8);
  message.data[7] = (uint8_t)((speed & 0x00FF));

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
