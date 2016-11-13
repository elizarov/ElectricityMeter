/*
 Hardware.
   Sparkfun RS485 Breakout
   D2 - RX-I
   D3 - TX-O
   D4 - RTS Green Led
   
   LCD circuit with Adafrunt I2C / SPI backback + 20x4 LCD
   A4 - DAT
   A5 - CLK
*/

// include the library code:
#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <FixNum.h>
#include <BlinkLed.h>
#include <Timeout.h>
#include <FmtRef.h>
#include <Uptime.h>

#include "crc.h"
#include "xprint.h"

//------- ALL TIME DEFS ------

const long INITIAL_DUMP_INTERVAL = 2000;
const long PERIODIC_DUMP_INTERVAL = 60000;
const long PERIODIC_DUMP_SKEW = 5000;

const unsigned int REGULAR_BLINK_INTERVAL = 1000; // 1 sec

const unsigned long RS485_TIMEOUT = 100;
const unsigned long RS485_DELAY = 10;
const unsigned long UPDATE_PERIOD = 500;

//------- HARDWARE ------

const uint8_t BLINK_LED_PIN = LED_BUILTIN;

const uint8_t RS485_RXI_PIN = 2;
const uint8_t RS485_TXO_PIN = 3;
const uint8_t RS485_RTS_PIN = 4;

const long RS485_BAUD = 9600;

Adafruit_LiquidCrystal lcd(0);
BlinkLed blinkLed(BLINK_LED_PIN);
SoftwareSerial rs485(RS485_TXO_PIN, RS485_RXI_PIN);

Timeout updateTimeout(0);

//------- STATE ------

const char STATUS_UNKNOWN = '?';
const char STATUS_OK = '*';

const uint32_t INVALID_VALUE = 0x7fffffff;

char status = STATUS_UNKNOWN;
fixnum32_1 volts[4];
fixnum32_1 amps[4]; 
fixnum32_1 watts[4];
fixnum32_1 hertz;

//------- LCD ------

inline void updateLCDSummary() {
  //              01234567890123456789
  char buf[21] = "?  ??.?Hz     ?????W";
  buf[0] = status;
  hertz.format(buf + 3, 4, FMT_RIGHT | 1);
  watts[0].format(buf + 14, 5, FMT_RIGHT | 0);
  lcd.setCursor(0, 0);
  lcd.print(buf);
}

inline void updateLCDPhase(uint8_t i) {
  //              01234567890123456789
  char buf[21] = "I: ???V ??.?A ?????W";
  buf[0] = '0' + i;
  volts[i].format(buf + 3, 3, FMT_RIGHT | 0);
  amps[i].format(buf + 8, 4, FMT_RIGHT | 1);
  watts[i].format(buf + 14, 5, FMT_RIGHT | 0);
  lcd.setCursor(0, i);
  lcd.print(buf);
}

inline void updateLCD() {
  updateLCDSummary();
  for (uint8_t i = 1; i <= 3; i++)
    updateLCDPhase(i);
}

//------- RS485 -------

uint8_t sendReceiveRaw(uint8_t* req, uint8_t req_size, uint8_t* resp, uint8_t resp_size) {
  computeCRC(req, req_size - 2);
  digitalWrite(RS485_RTS_PIN, 1); // write 
  delay(RS485_DELAY);
  rs485.write(req, req_size);
  digitalWrite(RS485_RTS_PIN, 0); // read
  Timeout timeout(RS485_TIMEOUT);
  uint8_t n = 0;
  while (!timeout.check() && n < resp_size) {
    if (rs485.available())
      resp[n++] = rs485.read();
  }
  return n;
}

bool sendReceive(uint8_t* req, uint8_t req_size, uint8_t* resp, uint8_t resp_size) { 
  uint8_t n = sendReceiveRaw(req, req_size, resp, resp_size);
  if (n < resp_size) 
    return false;
  uint8_t c0 = resp[resp_size - 2];  
  uint8_t c1 = resp[resp_size - 1];
  computeCRC(resp, resp_size - 2);
  return c0 == resp[resp_size - 2] && c1 == resp[resp_size - 1];  
}

bool openChannel() {
  const uint8_t req_size = 11;
  uint8_t req[req_size];
  req[0] = 0x00;
  req[1] = 0x01; // open channel
  req[2] = 0x01; // first level
  for (uint8_t i = 3; i < 9; i++)
    req[i] = 0x01; // password  
  const uint8_t resp_size = 4;
  byte resp[resp_size];   
  if (!sendReceive(req, req_size, resp, resp_size))
    return false;
  return resp[1] == 0x00; // Ok
}

int32_t readValue(uint8_t code) {
  const uint8_t req_size = 6;
  uint8_t req[req_size];
  req[0] = 0x00;
  req[1] = 0x08; // read
  req[2] = 0x11; // extra params
  req[3] = code;
  const uint8_t resp_size = 6;
  uint8_t resp[resp_size];
  if (!sendReceive(req, req_size, resp, resp_size)) 
    return INVALID_VALUE;
  return ((uint32_t)(resp[1] & (uint8_t)0x3f) << 16) |
    ((uint32_t)resp[2]) | ((uint32_t)resp[3] << 8);  
}

void updateValues() {
  if (!openChannel()) {
    status = STATUS_UNKNOWN;
    return;
  }
  status = STATUS_OK;
  for (uint8_t i = 1; i <= 3; i++)
    volts[i] = fixnum32_2(readValue(0x10 + i));
  for (uint8_t i = 1; i <= 3; i++)
    amps[i] = fixnum32_3(readValue(0x20 + i));
  for (uint8_t i = 0; i <= 3; i++)
    watts[i] = fixnum32_2(readValue(0x00 + i));
  hertz = fixnum32_2(readValue(0x40));  
}

void updateState() {
  if (updateTimeout.check()) {
    updateTimeout.reset(UPDATE_PERIOD);
    updateValues();
    updateLCD();
  }
}

//------- DUMP STATE -------

const char HIGHLIGHT_CHAR = '*';

bool firstDump = true; 

Timeout dump(INITIAL_DUMP_INTERVAL);
char dumpSLine[] = "[E:99999 f99.9;u00000000]* ";
char dumpPLine[] = "[Ex:99999 v999 a99.9]";

char* highlightPtr = FmtRef::find(dumpSLine, HIGHLIGHT_CHAR);

FmtRef wsRef(dumpSLine);
FmtRef fsRef(dumpSLine, 'f');
FmtRef uRef(dumpSLine, 'u');

char* phasePtr = FmtRef::find(dumpPLine, 'x');

FmtRef wpRef(dumpPLine);
FmtRef vpRef(dumpPLine, 'v');
FmtRef apRef(dumpPLine, 'a');

const char DUMP_REGULAR = 0;
const char DUMP_FIRST = HIGHLIGHT_CHAR;

void makeDump(char dumpType) {
  if (status != STATUS_OK)
    return; // no communication with meter -- do not report
  // format summary data
  wsRef = watts[0];
  fsRef = hertz;
  uRef = uptime();
  // format terminator char
  if (dumpType == DUMP_REGULAR) {
    *highlightPtr = 0;
  } else {
    char* ptr = highlightPtr;
    *(ptr++) = dumpType;
    if (dumpType != HIGHLIGHT_CHAR)
      *(ptr++) = HIGHLIGHT_CHAR; // must end with highlight (signal) char
    *ptr = 0; // and the very last char must be zero
  }
  // print summary
  waitPrintln(dumpSLine);
  // format at print each phase
  for (uint8_t i = 1; i <= 3; i++) {
    *phasePtr = i + '0';
    wpRef = watts[i];
    vpRef = volts[i];
    apRef = amps[i];
    Serial.println(dumpPLine);
  }
  // prepare for next dump
  dump.reset(PERIODIC_DUMP_INTERVAL + random(-PERIODIC_DUMP_SKEW, PERIODIC_DUMP_SKEW));
  firstDump = false;
}

inline void dumpState() {
  if (dump.check()) {
    if (firstDump)
      makeDump(DUMP_FIRST);
    else 
      makeDump(DUMP_REGULAR);
  }
}

//------- SETUP & MAIN -------

void setup() {
  lcd.begin(20, 4);
  rs485.begin(RS485_BAUD);
  pinMode(RS485_RTS_PIN, OUTPUT);
  setupPrint();
  waitPrintln("{E:Electicity meter started}*");
}

void loop() {
  updateState();
  dumpState();
  blinkLed.blink(REGULAR_BLINK_INTERVAL);

}
