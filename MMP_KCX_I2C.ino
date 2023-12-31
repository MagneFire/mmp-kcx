#include <Wire.h>
//#include <LowPower.h>
#include <avr/power.h>
#include <avr/sleep.h>

#define I2C_SDA 2
#define I2C_SCL 3
#define CON_PIN 4
#define HEADPHONE_EN_PIN 5
#define POWER_DETECT_PIN 6

#define I2C_ADDR        0x17

#define MMP_KCZ_STATUS        0x00
#define MMP_KCZ_POWER         0x01
#define MMP_KCZ_CONNECT       0x02
#define MMP_KCZ_HEADPHONE     0x03
#define MMP_KCZ_VOLUME        0x04
#define MMP_KCZ_AUTO_SWITCH   0x05

#define MMP_KCZ_LINK_MAC_NUM  0x06
#define MMP_KCZ_LINK_NAME_NUM 0x07
#define MMP_KCZ_LINK_MAC      0x20
#define MMP_KCZ_LINK_NAME     0x30

#define STATUS_POWERED     0x00
#define STATUS_CONNECTED   0x01
#define STATUS_HEADPHONE   0x02
#define STATUS_AUTO_SWITCH 0x03

#define CONNECTION_STATE_POLL_INTERVAL 5000

uint32_t prev_poll_time = 0;

bool is_powered = true;
bool is_connected = false;
bool is_headphone = false;
bool is_auto_switch = true;

uint8_t volume = 0;
uint8_t link_mac_num = 0;
uint8_t link_name_num = 0;

bool set_power = true;
bool set_power_value = false;
bool set_pair = false;
bool set_pair_value = false;
bool set_headphone = false;
bool set_headphone_value = false;
bool set_volume = false;
uint8_t set_volume_value = 0;
bool set_auto_switch = false;
bool set_auto_switch_value = false;

bool get_power = true;
bool get_pair = true;
bool get_volume = true;

int reg = 0;

void receiveEvent(int len);
void requestEvent();

void handleRead(uint8_t offset);
void handleWrite(uint8_t offset);
void writeRegister(uint8_t offset);

bool sendCommand(const char *command, uint8_t (*callback)(const char*, uint8_t));
uint8_t genericCommandHandler(const char *response, uint8_t response_pos);

bool isPowered();
void setPower(bool power_on);
//void pair();
void connect(bool connect);
bool isPaired();
void setHeadphone(bool headphone_on);

uint8_t getVolumeCallback(const char *response, uint8_t response_pos);
uint8_t getVolume();
void setVolume(uint8_t set_volume);

void powerDown();

#define COMMAND_TIMEOUT 1000

typedef enum CommandResult_t
{
  COMMAND_CONTINUE,
  COMMAND_OK,
  COMMAND_ERROR,
} CommandResult_t;

char *strnstr(const char *s, const char *find, size_t slen)
{
  char c, sc;
  size_t len;

  if ((c = *find++) != '\0') {
    len = strlen(find);
    do {
      do {
        if (slen-- < 1 || (sc = *s++) == '\0')
          return (NULL);
      } while (sc != c);
      if (len > slen)
        return (NULL);
    } while (strncmp(s, find, len) != 0);
    s--;
  }
  return ((char *)s);
}

void setup() {
  pinMode(I2C_SDA, INPUT);
  pinMode(I2C_SCL, INPUT);
  pinMode(POWER_DETECT_PIN, INPUT);
  digitalWrite(CON_PIN, HIGH);
  //  digitalWrite(HEADPHONE_EN_PIN, HIGH);
  pinMode(CON_PIN, OUTPUT);
  //  pinMode(HEADPHONE_EN_PIN, INPUT_PULLUP);
  pinMode(HEADPHONE_EN_PIN, INPUT);
  Serial.begin(115200);
  Serial1.begin(115200);

  Wire.onReceive(receiveEvent);
  Wire.onRequest(handleRead);
  Wire.begin(I2C_ADDR);
  pinMode(I2C_SDA, INPUT);
  pinMode(I2C_SCL, INPUT);

  // Wait for the BT chip to be online.
  for (uint8_t i = 0; i < 10; i++) {
    isPowered();
    if (is_powered) break;
  }
}

void loop() {
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
  // Echo Bluetooth data (for debugging)
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  //  Serial.println(millis());
  //  delay(100);
  if (millis() - prev_poll_time > CONNECTION_STATE_POLL_INTERVAL) {
    prev_poll_time = millis();
    isPaired();
    bool powerState = digitalRead(POWER_DETECT_PIN);
    Serial.print("Power state ");
    Serial.println(powerState);
    if (powerState == LOW && is_powered) {
      Serial.println("Device was shut down, powerdown now.");
      setPower(false);
    }
  }
  if (set_power) {
    set_power = false;
    setPower(set_power_value > 0);
  }
  if (set_pair) {
    set_pair = false;
    connect(set_pair_value > 0);
    //    pair();
  }
  if (set_headphone) {
    set_headphone = false;
    setHeadphone(set_headphone_value > 0);
  }
  if (set_volume) {
    set_volume = false;
    setVolume(set_volume_value);
  }
  if (set_auto_switch) {
    set_auto_switch = false;
    setAutoSwitch(set_auto_switch_value);
  }

  if (get_power) {
    get_power = false;
    isPowered();
  }
  if (get_pair) {
    get_pair = false;
    isPaired();
  }
  if (get_volume) {
    get_volume = false;
    getVolume();
  }
}

uint8_t genericCommandHandler(const char *response, uint8_t response_pos)
{
  //  Serial.print("genericCommandHandler ");
  //  Serial.print(response);
  //  Serial.print(" ");
  //  Serial.println(response_pos);
  if (strnstr(response, "OK+", response_pos)) {
    //    Serial.println("RESP is OK+");
    return COMMAND_OK;
  }
  if (strnstr(response, "CMD ERR", response_pos)) {
    return COMMAND_ERROR;
  }

  return COMMAND_CONTINUE;
}

bool sendCommand(const char *command, uint8_t (*callback)(const char*, uint8_t))
{
  //  return false;
  char response[200] = {0};
  uint8_t response_pos = 0;
  uint32_t uptime = millis();
  //  Serial.println("P1");
  Serial.print("cmd: ");
  Serial.println(command);
  Serial1.write(command);
  //  Serial.println("P2");
  while (millis() - uptime < COMMAND_TIMEOUT) {
    //    Serial.print("P3 ");
    //    Serial.println(millis() - uptime);
    while (Serial1.available()) {
      //      Serial.println("P4");
      response[response_pos++] = Serial1.read();
      response_pos %= sizeof(response);
      response[response_pos] = '\0';
      //      Serial.print("R ");
      //      Serial.println(response_pos);

      uint8_t cb_result = COMMAND_CONTINUE;

      char *eol = NULL;
      do {
        eol = (char *)memchr(response, '\r', response_pos);
        if (eol == NULL) {
          eol = (char *)memchr(response, '\n', response_pos);
        }
        if (eol == NULL) {
          break;
        }

        uint8_t line_length = eol - response;

        Serial.print("> ");
        Serial.println(response);

        if (callback) {
          cb_result = callback(response, line_length);
        } else {
          cb_result = genericCommandHandler(response, line_length);
        }

        if (cb_result == COMMAND_OK) {
          return true;
        }
        if (cb_result == COMMAND_ERROR) {
          return false;
        }

        char * remainder = eol + 1;
        size_t remainder_length = response_pos - line_length - 1;

        memmove(response, remainder, remainder_length);
        response[remainder_length] = '\0';
        response_pos = remainder_length;
      } while (eol != NULL);
    }
    delay(100);
  }
  if (millis() - uptime >= COMMAND_TIMEOUT) {
    Serial.println("Timeout while executing command.");
  }
  return false;
}

void powerUp()
{
  set_sleep_mode(SLEEP_MODE_IDLE);
//    power_adc_disable();
//    power_usart0_enable();
//    power_spi_enable();
//    power_timer1_enable();
//    power_timer2_enable();
//    power_timer3_enable();
//    power_usart1_enable();
    power_usb_enable();
}

void powerDown()
{
  Serial.println("Powerdown enter");
  //  // Power down.
//    SMCR |= (1<<SM1);
//    power_adc_disable();
//    power_usart0_disable();
//    power_spi_disable();
//  //  power_twi_disable();
//    power_timer1_disable();
//    power_timer2_disable();
//    power_timer3_disable();
//    power_usart1_disable();
    power_usb_disable();
  //   // Freeze the USB Clock
  //  USBCON |= (1 << FRZCLK);
  //  // Disable the USB Clock (PPL)
  //  PLLCSR &= ~(1 << PLLE);
  //  // Disable the USB
  //  USBCON &=  ~(1 << USBE);
  //  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  Serial.println("Powerdown leave");
}

bool isPowered()
{
  is_powered = sendCommand("AT+\r\n", NULL);
  return is_powered;
}

void setPower(bool power_on)
{
  is_powered = isPowered();
  if (is_powered == power_on) {
    return;
  }

  if (power_on) {
    powerUp();
    // Wakes the chip when in poweroff.
    digitalWrite(CON_PIN, LOW);
    delay(100);
    digitalWrite(CON_PIN, HIGH);
  }
  if (!power_on) {
    sendCommand("AT+POWER_OFF\r\n", NULL);
    if (is_auto_switch) {
      setHeadphone(false);
    }
    is_connected = false;
    powerDown();
  }

  is_powered = power_on;
}

void connect(bool connect)
{
  is_powered = isPowered();
  if (!is_powered) return;

  if (is_connected == connect) {
    return;
  }

  if (connect) {
    sendCommand("AT+PAIR\r\n", NULL);
  } else {
    sendCommand("AT+DISCON\r\n", NULL);
  }
  is_connected = connect;
}

//void pair()
//{
//  is_powered = isPowered();
//  if (!is_powered) return;
//  sendCommand("AT+PAIR\r\n", NULL);
//}


uint8_t getPairedCallback(const char *response, uint8_t response_pos)
{
  char *endptr;
  //  Serial.print("getPairedCallback ");
  //  Serial.print(response);
  //  Serial.print(" ");
  //  Serial.println(response_pos);

  if (strnstr(response, "OK+STATUS:", response_pos)) {
    char *space = (char *)memchr(response, ':', response_pos);
    if (space == NULL) {
      return COMMAND_CONTINUE;
    }
    is_connected = strtoul(space + 1, &endptr, 10) > 0;
    Serial.print("PAIRED ");
    Serial.println(is_connected);
    //    Serial.println(" RESP PAIR OK");
    return COMMAND_OK;
  }
  return COMMAND_CONTINUE;
}

bool isPaired()
{
  is_powered = isPowered();
  if (!is_powered) return;

  sendCommand("AT+STATUS?\r\n", getPairedCallback);
  //  Serial.println("isPaired: END");
  if (is_auto_switch) {
    setHeadphone(is_connected);
  }
  return is_connected;
}

void setHeadphone(bool headphone_on)
{
  if (is_headphone == headphone_on) {
    return;
  }

  //  Serial.print("setHeadphone ");
  //  Serial.print(is_headphone);
  //  Serial.print(" -> ");
  //  Serial.println(headphone_on);

  if (headphone_on) {
    digitalWrite(HEADPHONE_EN_PIN, LOW);
    pinMode(HEADPHONE_EN_PIN, OUTPUT);
  }
  if (!headphone_on) {
    pinMode(HEADPHONE_EN_PIN, INPUT);
  }

  is_headphone = headphone_on;
}

void setAutoSwitch(bool enable)
{
  if (is_auto_switch == enable) {
    return;
  }

  is_auto_switch = enable;
}

bool getAutoSwitch()
{
  return is_auto_switch;
}

uint8_t getVolumeCallback(const char *response, uint8_t response_pos)
{
  char *endptr;
  //  Serial.print("getVolumeCallback ");
  //  Serial.print(response);
  //  Serial.print(" ");
  //  Serial.println(response_pos);

  if (strnstr(response, "OK+VOL=", response_pos)) {
    char *space = (char *)memchr(response, '=', response_pos);
    if (space == NULL) {
      return COMMAND_CONTINUE;
    }
    volume = strtoul(space + 1, &endptr, 10);
    Serial.print("VOL ");
    Serial.println(volume);
    //    Serial.println("RESP VOL OK");
    return COMMAND_OK;
  }

  if (strnstr(response, "CMD ERR", response_pos)) {
    return COMMAND_ERROR;
  }
  return COMMAND_CONTINUE;
}

uint8_t getVolume()
{
  is_powered = isPowered();
  if (!is_powered) return;
  //  Serial.println("getVolume: START");
  sendCommand("AT+VOL?\r\n", getVolumeCallback);
  //  Serial.println("getVolume: END");
  return volume;
}

void setVolume(uint8_t set_volume)
{
  char buffer[20];
  if (set_volume > 31) set_volume = 31;
  is_powered = isPowered();
  if (!is_powered) return;
  if (volume == set_volume) {
    return;
  }

  snprintf(buffer, sizeof(buffer), "AT+VOL=%d\r\n", set_volume);
  uint8_t cmd_result = sendCommand(buffer, NULL);
  if (cmd_result) {
    volume = set_volume;
  } else {
    Serial.println("Failure setting volume");
  }
}

void handleWrite(uint8_t offset)
{
  uint8_t value = 0;

  if (Wire.available() > 0) {
    value = Wire.read();
  } else {
    Serial.println("No value written to register");
    return;
  }
  //  Serial.print("Write: ");
  //  Serial.print(offset);
  //  Serial.print(" ");
  //  Serial.println(value);

  switch (offset) {
    case MMP_KCZ_POWER:
      set_power_value = value;
      set_power = true;
      //      setPower(value);
      break;
    case MMP_KCZ_CONNECT:
      set_pair_value = value;
      set_pair = true;
      //      pair();
      break;
    case MMP_KCZ_HEADPHONE:
      set_headphone_value = value;
      set_headphone = true;
      //      setHeadphone(value);
      break;
    case MMP_KCZ_VOLUME:
      set_volume_value = value;
      set_volume = true;
      //      setVolume(value);
      break;
    case MMP_KCZ_AUTO_SWITCH:
      set_auto_switch_value = value;
      set_auto_switch = true;
      break;
    default:
      Serial.println("Invalid write request.");
      break;
  }
}

//void writeRegister(uint8_t offset)
//{
//  int rx = offset;
//  Serial.print("WRITE [");
//  Serial.print(rx);
//  Serial.println("]");
//  while(Wire.available() > 0){
//    rx %= sizeof(regbuffer);
//    regbuffer[rx++] = Wire.read();
//  }
//}

void receiveEvent(int len) {
  int rx = 0;
  Serial.print("[receiveEvent] Received Request:");
  Serial.println(len);
  //  Serial.println();
  if (len == 1) { // One Byte Data received -> Read Request Address
    reg = Wire.read();
    //    Serial.print("Register Requested: ");
    //    Serial.println(reg, HEX);
  } else {
    reg = -1;
    rx = Wire.read();
    //    writeRegister(rx);
    handleWrite(rx);
    //    while(Wire.available() > 0){
    //      rx %= sizeof(regbuffer);
    //      regbuffer[rx++] = Wire.read();
    //    }
  }
}

void handleRead()
{
  //  Serial.println("handleRead");
  if (reg < 0) return;
  uint8_t status_byte = 0;
  switch (reg) {
    case MMP_KCZ_STATUS:
      //      isPowered();
      //      isPaired();
      //      get_power = true;
      //      get_pair = true;
      status_byte |= (is_powered ? 0x01 : 0x00) << STATUS_POWERED;
      status_byte |= (is_connected ? 0x01 : 0x00) << STATUS_CONNECTED;
      status_byte |= (is_headphone ? 0x01 : 0x00) << STATUS_HEADPHONE;
      status_byte |= (is_auto_switch ? 0x01 : 0x00) << STATUS_AUTO_SWITCH;
      //      Serial.print("MMP_KCZ_STATUS: ");
      //      Serial.println(status_byte);
      Wire.write(status_byte);
      break;
    case MMP_KCZ_POWER:
      //      get_power = true;
      //      Serial.print("MMP_KCZ_POWER: ");
      //      Serial.println(is_powered);
      Wire.write(is_powered);
      break;
    case MMP_KCZ_CONNECT:
      //      get_pair = true;
      //      Serial.print("MMP_KCZ_CONNECT: ");
      //      Serial.println(is_connected);
      Wire.write(is_connected);
      break;
    case MMP_KCZ_HEADPHONE:
      //      Serial.print("MMP_KCZ_HEADPHONE: ");
      //      Serial.println(is_headphone);
      Wire.write(is_headphone);
      break;
    case MMP_KCZ_VOLUME:
      //      getVolume();
      get_volume = true;
      //      Serial.print("MMP_KCZ_VOLUME: ");
      //      Serial.println(volume);
      Wire.write(volume);
      break;
    case MMP_KCZ_AUTO_SWITCH:
      Wire.write(is_auto_switch);
      break;
  }
}

//void requestEvent(){
//  if (reg < 0) return;
//  handleRead();
//  // Read event
////  int p = reg % sizeof(regbuffer); // Repeat Buffer
////  unsigned char c = regbuffer[p];
////  Serial.println("[requestEvent] Received Request.");
////  Wire.write(c);
//}
