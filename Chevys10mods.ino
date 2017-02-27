/*
  Teensy 3.2 Pin List and usage in this project.

  Pin     Used    Purpose
  ___     ____    _______
  GND     Yes     Gee, let me think....
  0       Yes     RX from jrk12v12
  1       Yes     TX to jrk12v12
  2       Yes     RC Input CH1
  3       Yes     RC Input CH2
  4       Yes     RC Input CH3
  5
  6       Yes     Power/Charge Relay
  7       Yes     RX from roboclaw
  8       yes     TX to roboclaw
  9       Yes     RX from Bluetooth
  10      Yes     TX to Bluetooth
  11
  12
  13      Yes     Onboard LED for debug visual feedback
  14
  15
  16
  17
  18
  19
  20
  21
  22      Yes     Bluetooth RTS
  23      Yes     Bluetooth CTS





*/
#include <EEPROMex.h>
#include "Arduino.h"
#include "RoboClaw.h"
#include <TimeLib.h>


const int maxAllowedWrites = 80;
const int memBase          = 350;

void setSteering(int target, String src);
void setThrottle(long speed, String src);
void setThrottle(long speed, int diffValue, String src);
void RCInput();
void RCchannel1();
void RCchannel2();
void RCchannel3();
void setupRC();
void exeCmd();
void setup();
void loop();


bool needBTConfig = false;
bool connRC = true;
bool connBT = false;
bool inputRC = true;
bool inputBT = false;
bool enableDiffSteer = true;
float DiffRate = 0.25;

int lastSteering;
long lastSpeed;

// RoboRemo for Android control of a powerwheels type childs car, upgraded with linear actuator driven by Pololu jrk 12v12,
// Roboclaw 60A for driving the Motors, and A bluecreations bc127 module for connectivity and music streaming
#define debug false
#define debugrc false
#define debugsource false

RoboClaw roboclaw(&Serial3, 10000);
#define address 0x80

int addressMaxForward;
int addressMaxReverse;

//int intMaxForward = 127;
//int intMaxReverse = -48;
long intMaxForward = 32767;
long intMaxReverse = -32768;

int rcCh1[2] = {982, 1976};
int rcCh2[2] = {986, 1985};


long lastheartbeat;
int failsafe;

//RC Section
#define RCPIN1 2
#define RCPIN2 3
#define RCPIN3 4

volatile uint16_t channel1;
volatile uint16_t channel2;
volatile uint16_t channel3;
volatile uint16_t channel_start1;
volatile uint16_t channel_start2;
volatile uint16_t channel_start3;
int rcprevsteeringInput;
int rcsteeringInput;
long rcprevthrottleInput;
long rcthrottleInput;
int steeringmod;

void setSteering(int target, String src) {
  if (debugsource) {
    Serial.println(src + (String)" - " + (String)target);
  }
  steeringmod = abs(map(target, 40, 3900, -4096, 4096));
  if (connBT && !inputBT) {
    Serial2.print("ch1 ");
    Serial2.print(target);
    Serial2.print("\r");
  }
  unsigned char targetcmd[5] = {0xAA, 11, 0xc0 + (target & 0x1f), (target >> 5) & 0x7f};
  lastSteering = target;
  Serial1.write(targetcmd, 4);

}

void setThrottle(long speed, String src) {
  if (debugsource) {
    Serial.println(src + (String)" - " + (String)speed);
  }
  if (connBT && !inputBT) {
    Serial2.print("ch2 ");
    Serial2.print(speed);
    Serial2.print('\r');
  }
  // if (speed < 0) {
  lastSpeed = speed;
  roboclaw.DutyM1M2(address, map(speed, -32768, 32767, intMaxReverse, intMaxForward), map(speed, -32768, 32767, intMaxReverse, intMaxForward));
  //   }
  // else {
  //   roboclaw.DutyM1M2(address, map(speed, 0, 32760, 0, intMaxForward), map(speed, 0, 32760, 0, intMaxForward));
  // }
  /* THIS WORKED
        if (speed < 0) {
    roboclaw.BackwardM1(address, abs(map(speed, -127, -1, intMaxReverse, -1)));
    roboclaw.BackwardM2(address, abs(map(speed, -127, -1, intMaxReverse, -1)));
    }
    else {
    roboclaw.ForwardM1(address, map(speed, 0, 127, 0, intMaxForward));
    roboclaw.ForwardM2(address, map(speed, 0, 127, 0, intMaxForward));
    }
  */
  /*
      The duty value is signed and the range is -32760 to +32760(eg. +-100% duty).
      The accel value range is 0 to 655359(eg maximum acceleration rate is -100% to 100% in 100ms).
      bool DutyM1(uint8_t address, uint16_t duty);
    bool DutyM2(uint8_t address, uint16_t duty);
    bool DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2);
    bool DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel);
    bool DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel);
    bool DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2);
    bool ReadTemp(uint8_t address, uint16_t &temp);
    bool ReadTemp2(uint8_t address, uint16_t &temp);

  */
}

void setThrottle(long speed, int diffValue, String src) {
  if (debugsource) {
    Serial.println(src + (String)" - " + (String)speed);
  }
  if (connBT && !inputBT) {
    Serial2.print("ch2 ");
    Serial2.print(speed);
    Serial2.print('\r');
  }
  lastSpeed = speed;
  int activemod;
  if (diffValue > 5) {
    roboclaw.DutyM1M2(address, map(speed, -600, 600, intMaxReverse + steeringmod, intMaxForward - steeringmod), map(speed, -600, 600, intMaxReverse, intMaxForward));
    if (connBT && !inputBT) {
      Serial2.print("spd ");
      Serial2.print(map(speed, -600, 600, intMaxReverse, intMaxForward));
      Serial2.print("\rlw ");
      Serial2.print(map(speed, -600, 600, intMaxReverse, intMaxForward));
      Serial2.print("\rrw ");
      Serial2.print(map(speed, -600, 600, intMaxReverse + steeringmod, intMaxForward - steeringmod));
      Serial2.print('\r');
    }
  }
  else if (diffValue < -5) {
    roboclaw.DutyM1M2(address, map(speed, -600, 600, intMaxReverse, intMaxForward), map(speed, -600, 600, intMaxReverse + steeringmod, intMaxForward - steeringmod));
    if (connBT && !inputBT) {
      Serial2.print("spd ");
      Serial2.print(map(speed, -600, 600, intMaxReverse, intMaxForward));
      Serial2.print("\rlw ");
      Serial2.print(map(speed, -600, 600, intMaxReverse + steeringmod, intMaxForward - steeringmod));
      Serial2.print("\rrw ");
      Serial2.print(map(speed, -600, 600, intMaxReverse, intMaxForward));
      Serial2.print('\r');
    }
  }
  else {
    // if (speed < 0) {
    roboclaw.DutyM1M2(address, map(speed, -600, 600, intMaxReverse, intMaxForward), map(speed, -600, 600, intMaxReverse, intMaxForward));
    if (connBT && !inputBT) {
      Serial2.print("spd ");
      Serial2.print(map(speed, -600, 600, intMaxReverse, intMaxForward));
      Serial2.print("\rlw ");
      Serial2.print(map(speed, -600, 600,  intMaxReverse, intMaxForward));
      Serial2.print("\rrw ");
      Serial2.print(map(speed, -600, 600, intMaxReverse, intMaxForward));
      Serial2.print('\r');
    }
  }
  /* THIS WORKED
        if (speed < 0) {
    roboclaw.BackwardM1(address, abs(map(speed, -127, -1, intMaxReverse, -1)));
    roboclaw.BackwardM2(address, abs(map(speed, -127, -1, intMaxReverse, -1)));
    }
    else {
    roboclaw.ForwardM1(address, map(speed, 0, 127, 0, intMaxForward));
    roboclaw.ForwardM2(address, map(speed, 0, 127, 0, intMaxForward));
    }
  */
  /*
      The duty value is signed and the range is (long)-32760 to +32760(eg. +-100% duty).
      The accel value range is 0 to 655359(eg maximum acceleration rate is -100% to 100% in 100ms).
      bool DutyM1(uint8_t address, uint16_t duty);
    bool DutyM2(uint8_t address, uint16_t duty);
    bool DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2);
    bool DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel);
    bool DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel);
    bool DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2);
    bool ReadTemp(uint8_t address, uint16_t &temp);
    bool ReadTemp2(uint8_t address, uint16_t &temp);

  */
}

void RCInput() {
  if (debug) Serial.println(channel3);

  if (channel1 > 900 && channel1 < 2200) {
    if (channel1 < rcCh1[0]) rcCh1[0] = channel1;
    if (channel1 > rcCh1[1]) rcCh1[1] = channel1;
    rcsteeringInput = map(channel1, rcCh1[0], rcCh1[1], 5, 3900);

    int rcsteeringdiff = rcsteeringInput - rcprevsteeringInput;
    if (abs(rcsteeringdiff) > 25) {
      rcprevsteeringInput = rcsteeringInput;
      setSteering(rcprevsteeringInput, (String)"RC");
      if (enableDiffSteer) {
        setThrottle(rcprevthrottleInput, map(channel1, rcCh1[0], rcCh1[1], -100, 100), (String)"RC");
      }
    }
  }
  if (debugrc) {
    SerialUSB.print(channel1);
    SerialUSB.print("\t(");
    SerialUSB.print(rcprevsteeringInput);
    SerialUSB.print(") ");
    SerialUSB.print("\t(");
    SerialUSB.print(rcsteeringInput);
    SerialUSB.println(") ");
  }

  if (channel2 > 900 && channel2 < 2200) {
    //    if (channel2 < rcCh2[0]) rcCh2[0] = channel2;
    //    if (channel2 > rcCh2[1]) rcCh2[1] = channel2;
    rcthrottleInput = map(channel2, rcCh2[0], rcCh2[1], -600, 600);
    int rcthrottlediff = rcthrottleInput - rcprevthrottleInput;
    if (abs(rcthrottlediff) > 5) {
      rcprevthrottleInput = rcthrottleInput;
      if (enableDiffSteer) {
        setThrottle(rcprevthrottleInput, map(channel1, rcCh1[0], rcCh1[1], -100, 100), (String)"RC");
      }
      else {
        setThrottle(rcprevthrottleInput, (String)"RC");
      }
    }
  }
  if (debugrc) {
    SerialUSB.print(channel2);
    SerialUSB.print("\t(");
    SerialUSB.print(rcprevthrottleInput);
    SerialUSB.print(") ");
    SerialUSB.print("\t(");
    SerialUSB.print(rcsteeringInput);
    SerialUSB.println(") ");
  }

  if (debugrc) {
    SerialUSB.print(channel3);
    SerialUSB.print("(");
    SerialUSB.print(channel3);
    SerialUSB.println(")");
  }

}

void RCchannel1() {
  // If the pin is HIGH, start a timer
  if (digitalRead(RCPIN1) == HIGH) {
    channel_start1 = micros();
  } else {
    // The pin is now LOW so output the difference
    // between when the timer was started and now
    channel1 = (uint16_t) (micros() - channel_start1);
  }
}

void RCchannel2() {
  // If the pin is HIGH, start a timer
  if (digitalRead(RCPIN2) == HIGH) {
    channel_start2 = micros();
  } else {
    // The pin is now LOW so output the difference
    // between when the timer was started and now
    channel2 = (uint16_t) (micros() - channel_start2);
  }
}
void RCchannel3() {
  // If the pin is HIGH, start a timer
  if (digitalRead(RCPIN3) == HIGH) {
    channel_start3 = micros();
  } else {
    // The pin is now LOW so output the difference
    // between when the timer was started and now
    channel3 = (uint16_t) (micros() - channel_start3);
  }
}

void setupRC() {
  pinMode(RCPIN1, INPUT);
  pinMode(RCPIN2, INPUT);
  pinMode(RCPIN3, INPUT);

  // Attach an interrupt handler to be called whenever
  // the pin changes from LOW to HIGH or vice versa
  attachInterrupt(RCPIN1, RCchannel1, CHANGE);
  attachInterrupt(RCPIN2, RCchannel2, CHANGE);
  attachInterrupt(RCPIN3, RCchannel3, CHANGE);
}

//End RC


// I am using Serial3 for the roboclaw, Serial1 for the jrk and Serial2 for the bc127.


const int ledPin =  13;      // the number of the LED pin

int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated

long interval = 1000;           // interval at which to blink (milliseconds)

int throttleInput = 0;
int prevthrottleInput = 0;
int steeringInput = 2048;
int prevsteeringInput = 2048;

uint16_t voltage;


char cmd[100];
int cmdIndex;

long lastCmdTime = 60000;

boolean cmdStartsWith(char *st) {
  for (int i = 0; ; i++) {
    if (st[i] == 0) return true;
    if (cmd[i] == 0) return false;
    if (cmd[i] != st[i]) return false;;
  }
  return false;
}

void exeCmd() {
  if (ledState == LOW)
    ledState = HIGH;
  else
    ledState = LOW;

  lastCmdTime = millis();

  if ( cmdStartsWith("ERROR") ) {
    if (Serial) {
      Serial.print("ERROR MSG FROM BT - ");
      Serial.println(cmd);
    }
  }

  if ( cmdStartsWith("CONTROL") ) {
    inputBT = !inputBT;
    prevthrottleInput = lastSpeed;
    prevsteeringInput = lastSteering;
    inputRC = !inputRC;
    rcprevthrottleInput = lastSpeed;
    rcprevsteeringInput = lastSteering;

    if (Serial) Serial.println("Control swapped!");
  }

  if ( cmdStartsWith("RECV") ) {
    Serial2.print("ENTER_DATA_MODE 15\r");
    connBT = true;
    if (SerialUSB) SerialUSB.println("BT Connected");
  }

  if ( cmdStartsWith("hb" )) {
    if (!connBT) {
      connBT = true;
    }
    lastheartbeat = millis();
    if (failsafe > 0) failsafe = 0;
    bool *valid;
//    voltage=roboclaw.ReadMainBatteryVoltage(address);
//    Serial2.print("msg ");
//    Serial2.print((String)voltage);
//    Serial2.print("\r");

    Serial2.print((String)"msg " + (String)year() + "-" + (String)month() + "-" + (String)day() + " " + (String)hour() + ":" + (String)minute() + ":" + (String)second() + "\r");
//    Serial2.print((String)"msg " + (String)rcCh1[0] + "-" + (String)rcCh1[1] + "   /   " + (String)rcCh2[0] + "-" + (String)rcCh2[1] + "\r");
    Serial2.print((String)"led1 1\rledrc " + (String)inputRC + "\rledbt " + (String)inputBT + "\r");
    Serial2.print("logch1 " + (String)lastSteering + "\rlogch2 " + (String)lastSpeed + "\r");
  }
  if (cmdStartsWith("OPEN_OK 15 SPP") || cmdStartsWith("OPEN_OK 25 SPP")) {
    int intbtID =  (int)atof(cmd + 8);
    if (Serial) Serial.println((String)"Entering Data Mode for ID " + (String)intbtID);
    Serial2.print((String)"ENTER_DATA_MODE " + (String)intbtID + "\r");
    connBT = true;
    if (Serial) Serial.println("BT Connected");

  }
  if (cmdStartsWith("CLOSE_OK 15 SPP") || cmdStartsWith("CLOSE_OK 25 SPP")) {
    if (Serial) Serial.println("BT Disconnected");
    connBT = false;
    if (inputBT) inputBT = false;
    if (connRC) inputRC = true;

  }

  //OPEN_OK 15 SPP
  //CLOSE_OK 15 SPP


  if (inputBT) {
    if ( cmdStartsWith("ch") ) {
      int ch = cmd[2] - '0';
      if (ch == 1 && cmd[3] == ' ') {
        steeringInput = constrain((int)atof(cmd + 4), 0, 4095);
        int diff = steeringInput - prevsteeringInput;
        if (abs(diff) > 20) {
          prevsteeringInput = steeringInput;
          setSteering(prevsteeringInput, (String)"BT");
        }
      }
      if (ch == 2 && cmd[3] == ' ') {
        throttleInput = (int)atof(cmd + 4);
        int throttlediff = throttleInput - prevthrottleInput;
        if (abs(throttlediff) > 5) {
          prevthrottleInput = throttleInput;
          setThrottle(prevthrottleInput, (String)"BT");
        }
      }
    }
  }

  // use accelerometer:
  // example: set acc y id to "ca1"
  /*
    if ( cmdStartsWith("ca") ) {
      int ch = cmd[2] - '0';
      if (ch >= 0 && ch <= 9 && cmd[3] == ' ') {
        //        chVal[ch] = (usMax + usMin) / 2 + (int)( atof(cmd + 4) * 51 ); // 9.8*51 = 500 => 1000 .. 2000
      }
    }
  */
  // invert accelerometer:
  // example: set acc y id to "cb1"
  /*
    if ( cmdStartsWith("cb") ) {
      int ch = cmd[2] - '0';
      if (ch >= 0 && ch <= 9 && cmd[3] == ' ') {
        //        chVal[ch] = (usMax + usMin) / 2 - (int)( atof(cmd + 4) * 51 ); // 9.8*51 = 500 => 1000 .. 2000
      }
    }
  */
}

void setup() {
  setSyncProvider(getTeensy3Time);
  if (needBTConfig == true) {
    SerialUSB.begin(115200);
    Serial2.begin(115200);
    Serial2.print("\rrestore\r");
    Serial2.begin(9600);
    delay(10000);
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    Serial2.print("\rrestore\r");
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    delay(2000);

    Serial2.print("\rset usb_host=off\r");
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    SerialUSB.println("");
    Serial2.print("set audio=0\r");
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    SerialUSB.println("");
    Serial2.print("set name=Chevy_S10_Radio\r");
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    SerialUSB.println("");
    Serial2.print("set name_short=S10\r");
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    SerialUSB.println("");
    Serial2.print("set audio_analog=15 15 0 OFF\r");
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    SerialUSB.println("");
    Serial2.print("set autoconn=0\r");
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    SerialUSB.println("");
    Serial2.print("set BT_VOL_CONFIG=F E F\r");
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    SerialUSB.println("");
    Serial2.print("set CONN_TO=0\r");
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    SerialUSB.println("");
    Serial2.print("set discoverable=1 0\r");
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    SerialUSB.println("");
    Serial2.print("set ENABLE_BATT_IND=OFF\r");
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    Serial2.print("set gpio_config=OFF 0 0\rset profiles=0 0 3 0 3 1 1 0 0 0 0 0\r");
    delay(2000);
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    SerialUSB.println("");
    Serial2.print("set uart_config=115200 on 0\r");
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    Serial2.print("write\r");
    delay(2000);
    Serial2.print("reset\r");
    delay(2000);
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }
    delay(2000);
    if (Serial2.available()) {
      while (Serial2.available() > 0) {
        SerialUSB.write(Serial2.read());
      }
    }

  }

  setupRC();
  pinMode(ledPin, OUTPUT);

  delay(2000);
  Serial.begin(115200);
  roboclaw.begin(38400);
  Serial2.begin(115200);
  Serial1.begin(38400);
  Serial2.attachRts(22);
  Serial2.attachCts(23);
  delay(2000);
  Serial.println("Ready to start");  //  for(int i=0; i<chCount; i++) {
  Serial2.print("config\r");
  cmdIndex = 0;
}
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

char c = 0;
uint16_t ch1prev = 0;
uint16_t ch2prev = 0;

void loop() {
  if (1 == 2) {
    Serial.print("ch1:");
    Serial.print(rcCh1[0]);
    Serial.print("-");
    Serial.print(rcCh1[1]);
    Serial.print("ch2:");

    Serial.print(rcCh2[0]);
    Serial.print("-");
    Serial.println(rcCh2[1]);
  }
  if (inputBT) {
    unsigned long currentMillis = millis();
    if (Serial && debug) Serial.println("FS Check");
    if (failsafe == 2 && currentMillis - lastheartbeat > 5000) {
      if (Serial) {
        Serial.println("BT Failsafe level 3 - BT Controls disabled, RC enabled");
      }
      inputBT = false;
      if (connBT) {
        Serial2.print("MSG Control disabled due to slow or missing heartbeat. Please use the RC remote\r");
      }
      if (connRC) {
        inputRC = true;
      }
      failsafe = 3;
    } else if (failsafe == 1 && currentMillis - lastheartbeat > 3000) {
      if (Serial) {
        Serial.println("BT Failsafe level 2 - Throttle cut to 0");
      }
      setThrottle(0, (String)"FS");
      failsafe = 2;
    } else if (failsafe == 0 && currentMillis - lastheartbeat > 1500) {
      if (Serial) {
        Serial.println("BT Failsafe level 1 - Throttle cut by 50%");
      }
      setThrottle(prevthrottleInput / 2, (String)"FS");
      failsafe = 1;
    } else {
      if (Serial && debug) Serial.println("FS Passed");
      failsafe = 0;
    }
  }
  digitalWrite(ledPin, ledState);
  if (inputRC) {
    RCInput();
  }

  if (Serial.available()) {
    while (Serial.available() > 0) {
      if (!connBT) {
        Serial2.write(Serial.peek());
      }
      c = (char)Serial1.read();
    }
  }


  if (Serial1.available()) {
    while (Serial1.available() > 0) {
      Serial.print(Serial1.peek(), HEX);
      c = (char)Serial1.read();
    }
  }

  if (Serial2.available()) {
    while (Serial2.available() > 0) {
      Serial.write(Serial2.peek());
      c = (char)Serial2.read();
      if (c == '\r') {
        SerialUSB.println("");
        cmd[cmdIndex] = 0;
        exeCmd();  // execute the command
        cmdIndex = 0; // reset the cmdIndex
      } else {
        cmd[cmdIndex] = c;
        if (cmdIndex < 99) {
          cmdIndex++;
        } else {
          cmd[cmdIndex] = 0;
          cmdIndex = 0;
        }
      }

    }
  }
}

