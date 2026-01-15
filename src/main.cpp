#include <SPI.h>
#include <Arduino.h>
#include <SdFat.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Servo.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>

#define SD_CS 11
#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 8
#define MOTOR_1 22
#define MOTOR_2 24
#define MOTOR_3 26
#define MOTOR_4 28
#define DROP_BTN 30
#define Sensor_PIN 32

#define MAX_SCHEDULES 12
#define MAX_GROUPED 12
#define MAX_MEDS_PER_TIME 3
#define TEMP_BUFFER_SIZE 64

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
RTC_DS3231 rtc;
SdFat SD;
File file;
Servo servo1, servo2, servo3, servo4;

bool filestat = false;
bool receiving = false;
unsigned long receiveStartTime = 0;
unsigned long lastByteTime = 0;
File streamingFile;
bool streamingActive = false;

char notificationMessage[200] = "";
unsigned long notificationStartTime = 0;
volatile bool sdBusy = false;
DateTime rtctime;

int currentMenuPage = 0;
unsigned long lastMenuUpdate = 0;
bool showNotification = false;
bool motorStates[4] = {false, false, false, false};

bool tftNeedsUpdate = true;            // Flag to trigger TFT update
uint8_t lastDisplayedMinute = 255;     // Track last displayed minute (255 = uninitialized)
bool lastNotificationState = false;    // Track notification state changes
bool lastFilestat = false;             // Track filestat changes
int lastGroupedCount = 0;              // Track schedule changes
unsigned long lastCountdownUpdate = 0; // Track countdown display updates
int lastCountdownValue = -1;           // Track last displayed countdown

void requestTFTUpdate()
{
  tftNeedsUpdate = true;
}

struct TubeMapping
{
  char tubeName[8];
  int servoIndex;
  int motorPin;
  Servo *servo;
};

TubeMapping tubeMappings[4] = {
    {"tube1", 0, MOTOR_1, &servo1},
    {"tube2", 1, MOTOR_2, &servo2},
    {"tube3", 2, MOTOR_3, &servo3},
    {"tube4", 3, MOTOR_4, &servo4}};

struct MedicationTime
{
  char time[6];
  char dosage[16];
  char medication[24];
  char tube[8];
  int amount;
};

MedicationTime schedules[MAX_SCHEDULES];
int scheduleCount = 0;

struct GroupedMedication
{
  char time[6];
  char medications[MAX_MEDS_PER_TIME][24];
  char dosages[MAX_MEDS_PER_TIME][16];
  char tubes[MAX_MEDS_PER_TIME][8];
  int amounts[MAX_MEDS_PER_TIME];
  int count;
};

GroupedMedication groupedSchedules[MAX_GROUPED];
int groupedCount = 0;

bool setupMode = false;
int currentTubeSetup = 0;
int totalTubesNeeded = 0;
char setupInstructions[200];
bool waitingForDropButton = false;

static bool triggerSetupAfterBT = false;

char setupTubes[10][8];
int setupTubeCount = 0;

#define SPI_SPEED_SD 4000000   // SD: 4 MHz (stable for Mega 2560)
#define SPI_SPEED_TFT 27000000 // TFT: 27 MHz (ST7789 max)
#define CS_SETTLE_TIME 20      // microseconds for CS settling

void sendDummyClocks(uint8_t count)
{
  for (uint8_t i = 0; i < count; i++)
  {
    SPI.transfer(0xFF);
  }
}

void openServo(Servo &servo, int standbyPos = 91, int openPos = 55)
{
  Serial.println(F("Opening servo"));
  servo.write(openPos);
  delay(600);
  servo.write(standbyPos);
}

void closeServo(Servo &servo, int standbyPos = 91, int closePos = 125)
{
  Serial.println(F("Closing servo"));
  servo.write(closePos);
  delay(600);
  servo.write(standbyPos);
}

void triggerMotor(int motorPin, bool turnOn)
{
  if (turnOn)
  {
    Serial.print(F("Starting motor on pin "));
    Serial.println(motorPin);
    digitalWrite(motorPin, HIGH);
  }
  else
  {
    Serial.print(F("Stopping motor on pin "));
    Serial.println(motorPin);
    digitalWrite(motorPin, LOW);
  }
}

TubeMapping *getTubeMapping(const char *tubeName)
{
  for (int i = 0; i < 4; i++)
  {
    if (strcmp(tubeMappings[i].tubeName, tubeName) == 0)
    {
      return &tubeMappings[i];
    }
  }
  return nullptr;
}

void dispenseFromTube(const char *tubeName)
{
  TubeMapping *mapping = getTubeMapping(tubeName);
  if (mapping == nullptr)
  {
    Serial.print(F("Unknown tube: "));
    Serial.println(tubeName);
    return;
  }

  Serial.print(F("Dispensing from "));
  Serial.println(tubeName);

  openServo(*mapping->servo);
  delayMicroseconds(200);

  triggerMotor(mapping->motorPin, true);
  motorStates[mapping->servoIndex] = true;

  unsigned long startTime = millis();
  bool dispensingComplete = false;

  const unsigned long STABILIZE_DELAY = 500;
  while (millis() - startTime < STABILIZE_DELAY)
  {
    delay(1);
  }

  const unsigned long TIMEOUT_MS = 30000;
  while (millis() - startTime < TIMEOUT_MS)
  {
    int sensorState = digitalRead(Sensor_PIN);

    delayMicroseconds(50);
    if (sensorState == LOW && digitalRead(Sensor_PIN) == LOW)
    {
      Serial.println(F("Beam blocked → stopping motor"));
      dispensingComplete = true;
      break;
    }

    delayMicroseconds(200);
  }

  triggerMotor(mapping->motorPin, false);
  motorStates[mapping->servoIndex] = false;
  delayMicroseconds(100);
  closeServo(*mapping->servo);

  if (dispensingComplete)
  {
    Serial.println(F("Dispensing complete."));
    dispensingComplete = false;
  }
  else
  {
    Serial.println(F("Timeout: No detection."));
  }
}

void handleDispensing()
{
  Serial.println(F("DROP button pressed - starting dispensing sequence"));

  char currentTime[6];
  sprintf(currentTime, "%02d:%02d", rtctime.hour(), rtctime.minute());

  GroupedMedication *currentGroup = nullptr;
  for (int i = 0; i < groupedCount; i++)
  {
    if (strcmp(groupedSchedules[i].time, currentTime) == 0)
    {
      currentGroup = &groupedSchedules[i];
      break;
    }
  }

  if (currentGroup == nullptr)
  {
    Serial.println(F("No medications scheduled for current time"));
    return;
  }

  for (int i = 0; i < currentGroup->count; i++)
  {
    Serial.print(F("Dispensing medication "));
    Serial.print(i + 1);
    Serial.print(F(" of "));
    Serial.print(currentGroup->count);
    Serial.print(F(": "));
    Serial.println(currentGroup->medications[i]);

    dispenseFromTube(currentGroup->tubes[i]);

    if (i < currentGroup->count - 1)
    {
      Serial.println(F("Waiting before next tube..."));
      delay(2000);
    }
  }

  Serial.println(F("Dispensing sequence complete"));
  showNotification = false;
  requestTFTUpdate();
}

inline void deselectAll()
{
  digitalWrite(SD_CS, HIGH);
  digitalWrite(TFT_CS, HIGH);
  delayMicroseconds(CS_SETTLE_TIME);
  sendDummyClocks(2);
}

inline void selectTFT()
{
  deselectAll();
  delay(1);                            // Added 1ms delay for stability
  SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz on Mega
  digitalWrite(TFT_CS, LOW);
  delayMicroseconds(50); // Increased settle time
}

inline void selectSD()
{
  deselectAll();
  delay(1);                            // Added 1ms delay for stability
  SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (safe init)
  digitalWrite(SD_CS, LOW);
  delayMicroseconds(50); // Increased settle time
}

void drawLoadingBar(int progress, int x, int y, int width, int height)
{
  tft.drawRect(x, y, width, height, ST77XX_WHITE);
  int fillWidth = (progress * (width - 2)) / 100;
  if (fillWidth > 0)
  {
    tft.fillRect(x + 1, y + 1, fillWidth, height - 2, ST77XX_GREEN);
  }
}

void drawSpinner(int x, int y, int radius, int angle)
{
  tft.fillCircle(x, y, radius + 2, ST77XX_BLACK);

  for (int i = 0; i < 8; i++)
  {
    int segmentAngle = (angle + i * 45) % 360;
    int brightness = 255 - (i * 30);
    if (brightness < 50)
      brightness = 50;

    int x1 = x + (radius - 3) * cos(segmentAngle * PI / 180);
    int y1 = y + (radius - 3) * sin(segmentAngle * PI / 180);
    int x2 = x + radius * cos(segmentAngle * PI / 180);
    int y2 = y + radius * sin(segmentAngle * PI / 180);

    uint16_t color = tft.color565(brightness, brightness, brightness);
    tft.drawLine(x1, y1, x2, y2, color);
    tft.drawLine(x1 + 1, y1, x2 + 1, y2, color);
  }
}

void releaseSD()
{
  digitalWrite(SD_CS, HIGH);
}

int findMatchingBrace(const String &str, int start)
{
  int braceCount = 0;
  for (int i = start; i < str.length(); i++)
  {
    if (str[i] == '{')
      braceCount++;
    else if (str[i] == '}')
    {
      braceCount--;
      if (braceCount == 0)
        return i;
    }
  }
  return -1;
}

void animatedIntro()
{
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);

  tft.setTextSize(3);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(50, 100);
  tft.println(F("MedDispenser"));

  tft.setTextSize(1);
  tft.setCursor(80, 140);
  tft.println(F("Initializing..."));

  delay(2000);
}

void groupMedicationsByTime()
{
  groupedCount = 0;

  for (int i = 0; i < scheduleCount; i++)
  {
    int groupIndex = -1;

    for (int j = 0; j < groupedCount; j++)
    {
      if (strcmp(groupedSchedules[j].time, schedules[i].time) == 0)
      {
        groupIndex = j;
        break;
      }
    }

    if (groupIndex == -1)
    {
      groupIndex = groupedCount;
      strcpy(groupedSchedules[groupIndex].time, schedules[i].time);
      groupedSchedules[groupIndex].count = 0;
      groupedCount++;
    }

    int medIndex = groupedSchedules[groupIndex].count;
    if (medIndex < MAX_MEDS_PER_TIME)
    {
      strcpy(groupedSchedules[groupIndex].medications[medIndex], schedules[i].medication);
      strcpy(groupedSchedules[groupIndex].dosages[medIndex], schedules[i].dosage);
      strcpy(groupedSchedules[groupIndex].tubes[medIndex], schedules[i].tube);
      groupedSchedules[groupIndex].amounts[medIndex] = schedules[i].amount;
      groupedSchedules[groupIndex].count++;
    }
  }
}

bool startStreamingSave()
{
  const char *tmpName = "data.tmp";

  if (sdBusy)
  {
    Serial.println(F("startStreamingSave: SD busy, abort."));
    return false;
  }
  sdBusy = true;

  selectSD();

  if (SD.exists(tmpName))
  {
    SD.remove(tmpName);
    delay(100);
  }

  streamingFile = SD.open(tmpName, O_WRITE | O_CREAT | O_TRUNC);
  if (!streamingFile)
  {
    Serial.println(F("startStreamingSave: ERROR opening temp for write!"));
    deselectAll();
    sdBusy = false;
    return false;
  }

  streamingActive = true;
  Serial.println(F("Started streaming save to SD"));
  return true;
}

bool writeStreamingChunk(const String &chunk)
{
  if (!streamingActive || !streamingFile)
  {
    return false;
  }

  size_t written = streamingFile.print(chunk);
  streamingFile.flush();

  if (written != chunk.length())
  {
    Serial.println(F("writeStreamingChunk: ERROR incomplete write!"));
    return false;
  }
  return true;
}

void endStreamingSave()
{
  if (streamingFile)
  {
    streamingFile.close();
  }
  streamingActive = false;

  deselectAll();
  sdBusy = false;

  Serial.println(F("Streaming save complete"));
}

bool finishStreamingSave()
{
  if (!streamingActive)
    return false;

  const char *tmpName = "data.tmp";
  const char *finalName = "data.json";

  streamingFile.sync();
  streamingFile.close();
  streamingActive = false;
  delay(50);

  if (SD.exists(finalName))
  {
    bool removed = false;
    for (int attempt = 0; attempt < 6; ++attempt)
    {
      delay(40);
      removed = SD.remove(finalName);
      Serial.print(F("finishStreamingSave: remove final attempt "));
      Serial.print(attempt);
      Serial.print(F(" -> "));
      Serial.println(removed ? F("ok") : F("fail"));
      if (removed)
        break;
    }
  }

  bool renamed = SD.rename(tmpName, finalName);
  Serial.print(F("finishStreamingSave: rename -> "));
  Serial.println(renamed ? F("ok") : F("fail"));

  if (!renamed)
  {
    Serial.println(F("finishStreamingSave: fallback copy starting..."));
    File r = SD.open(tmpName, FILE_READ);
    if (!r)
    {
      Serial.println(F("finishStreamingSave: fallback: cannot open temp for read."));
      sdBusy = false;
      return false;
    }

    if (SD.exists(finalName))
    {
      SD.remove(finalName);
      delay(10);
    }

    File f2 = SD.open(finalName, O_WRITE | O_CREAT | O_TRUNC);
    if (!f2)
    {
      Serial.println(F("finishStreamingSave: fallback: cannot open final for write."));
      r.close();
      sdBusy = false;
      return false;
    }

    char buffer[32];
    while (r.available())
    {
      int bytesRead = r.readBytes(buffer, sizeof(buffer));
      f2.write((const uint8_t *)buffer, bytesRead);
    }

    f2.sync();
    f2.close();
    r.close();
    Serial.println(F("finishStreamingSave: fallback copy complete"));

    if (SD.exists(tmpName))
    {
      SD.remove(tmpName);
    }
  }

  sdBusy = false;
  Serial.println(F("Streaming save completed successfully"));
  delay(500);
  return true;
}

int timeToMinutes(const char *timeStr)
{
  int hours, minutes;
  if (sscanf(timeStr, "%d:%d", &hours, &minutes) != 2)
  {
    return -1;
  }
  return hours * 60 + minutes;
}

int findNextMedication()
{
  int currentMinutes = rtctime.hour() * 60 + rtctime.minute();
  int closestIndex = -1;
  int minDifference = 24 * 60;

  for (int i = 0; i < groupedCount; i++)
  {
    int scheduleMinutes = timeToMinutes(groupedSchedules[i].time);
    if (scheduleMinutes == -1)
      continue;

    int difference = scheduleMinutes - currentMinutes;
    if (difference < 0)
      difference += 24 * 60;

    if (difference < minDifference)
    {
      minDifference = difference;
      closestIndex = i;
    }
  }
  return closestIndex;
}

bool checkMedicationTime()
{
  char currentTime[6];
  sprintf(currentTime, "%02d:%02d", rtctime.hour(), rtctime.minute());

  for (int i = 0; i < groupedCount; i++)
  {
    if (strcmp(groupedSchedules[i].time, currentTime) == 0)
    {
      if (groupedSchedules[i].count == 1)
      {
        snprintf(notificationMessage, sizeof(notificationMessage),
                 "TIME TO TAKE: %s - %s",
                 groupedSchedules[i].medications[0],
                 groupedSchedules[i].dosages[0]);
      }
      else
      {
        snprintf(notificationMessage, sizeof(notificationMessage),
                 "TIME TO TAKE %d MEDS: %s (%s)",
                 groupedSchedules[i].count,
                 groupedSchedules[i].medications[0],
                 groupedSchedules[i].dosages[0]);

        if (groupedSchedules[i].count > 1 && strlen(notificationMessage) < 150)
        {
          char temp[50];
          snprintf(temp, sizeof(temp), " + %s (%s)",
                   groupedSchedules[i].medications[1],
                   groupedSchedules[i].dosages[1]);
          strncat(notificationMessage, temp, sizeof(notificationMessage) - strlen(notificationMessage) - 1);
        }
      }
      return true;
    }
  }
  return false;
}

bool loadScheduleData()
{
  if (sdBusy)
  {
    Serial.println(F("loadScheduleData: SD busy, abort"));
    return false;
  }
  sdBusy = true;

  selectSD();

  File f = SD.open("data.json", FILE_READ);
  if (!f)
  {
    Serial.println(F("Cannot find data.json"));
    deselectAll();
    sdBusy = false;
    return false;
  }

  size_t fileSize = f.size();
  Serial.print(F("loadScheduleData: fileSize = "));
  Serial.println(fileSize);
  if (fileSize == 0)
  {
    Serial.println(F("loadScheduleData: file empty"));
    f.close();
    deselectAll();
    sdBusy = false;
    return false;
  }

  StaticJsonDocument<128> filter;
  filter[0]["tube"] = true;
  filter[0]["type"] = true;
  filter[0]["amount"] = true;
  filter[0]["time_to_take"][0]["time"] = true;
  filter[0]["time_to_take"][0]["dosage"] = true;

  scheduleCount = 0;

  StaticJsonDocument<1024> doc;
  ReadBufferingStream in(f, 32);
  DeserializationError err = deserializeJson(doc, in, DeserializationOption::Filter(filter));

  f.close();

  if (err)
  {
    Serial.print(F("JSON parse error: "));
    Serial.println(err.c_str());
    sdBusy = false;
    return false;
  }

  if (!doc.is<JsonArray>())
  {
    Serial.println(F("JSON root is not an array"));
    sdBusy = false;
    return false;
  }

  JsonArray arr = doc.as<JsonArray>();
  for (JsonObject med : arr)
  {
    const char *tubeC = med["tube"] | "";
    const char *typeC = med["type"] | "";
    int amount = med["amount"] | 0;

    JsonArray times = med["time_to_take"].as<JsonArray>();
    if (times.isNull())
      continue;

    for (JsonObject t : times)
    {
      if (scheduleCount >= MAX_SCHEDULES)
        break;
      const char *timeC = t["time"] | "";
      const char *dosageC = t["dosage"] | "";

      strncpy(schedules[scheduleCount].tube, tubeC, sizeof(schedules[scheduleCount].tube) - 1);
      schedules[scheduleCount].tube[sizeof(schedules[scheduleCount].tube) - 1] = '\0';

      strncpy(schedules[scheduleCount].medication, typeC, sizeof(schedules[scheduleCount].medication) - 1);
      schedules[scheduleCount].medication[sizeof(schedules[scheduleCount].medication) - 1] = '\0';

      schedules[scheduleCount].amount = amount;

      strncpy(schedules[scheduleCount].time, timeC, sizeof(schedules[scheduleCount].time) - 1);
      schedules[scheduleCount].time[sizeof(schedules[scheduleCount].time) - 1] = '\0';

      strncpy(schedules[scheduleCount].dosage, dosageC, sizeof(schedules[scheduleCount].dosage) - 1);
      schedules[scheduleCount].dosage[sizeof(schedules[scheduleCount].dosage) - 1] = '\0';

      scheduleCount++;
    }
  }

  sdBusy = false;
  groupMedicationsByTime();
  Serial.print(F("Loaded "));
  Serial.print(scheduleCount);
  Serial.println(F(" medication schedules"));

  releaseSD();

  return scheduleCount > 0;
}

void drawHeader()
{
  tft.fillRect(0, 0, 320, 35, ST77XX_BLUE);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 8);
  if (rtctime.hour() < 10)
    tft.print("0");
  tft.print(rtctime.hour());
  tft.print(":");
  if (rtctime.minute() < 10)
    tft.print("0");
  tft.print(rtctime.minute());

  tft.setTextSize(1);
  tft.setCursor(10, 22);
  tft.print(rtctime.day());
  tft.print("/");
  tft.print(rtctime.month());
  tft.print("/");
  tft.print(rtctime.year());

  tft.setTextSize(1);
  tft.setCursor(200, 8);
  tft.print(F("STATUS: "));
  tft.setTextColor(filestat ? ST77XX_GREEN : ST77XX_RED);
  tft.print(filestat ? F("READY") : F("ERROR"));

  tft.fillRect(290, 8, 20, 12, ST77XX_GREEN);
  tft.drawRect(289, 7, 22, 14, ST77XX_WHITE);
  tft.fillRect(311, 10, 3, 8, ST77XX_WHITE);
}

void drawGroupedMedicationCard(int x, int y, int width, int height, GroupedMedication group, bool isNext = false)
{
  uint16_t cardColor = isNext ? ST77XX_YELLOW : ST77XX_WHITE;
  uint16_t textColor = isNext ? ST77XX_BLACK : ST77XX_BLACK;

  tft.fillRoundRect(x, y, width, height, 8, cardColor);
  tft.drawRoundRect(x, y, width, height, 8, isNext ? ST77XX_RED : ST77XX_BLUE);

  tft.setTextSize(2);
  tft.setTextColor(textColor);
  tft.setCursor(x + 8, y + 8);
  tft.print(group.time);

  if (group.count > 1)
  {
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(x + width - 50, y + 8);
    tft.print(group.count);
    tft.print(F(" MEDS"));
  }

  tft.setTextSize(1);
  tft.setTextColor(textColor);
  tft.setCursor(x + 8, y + 32);
  tft.print(group.medications[0]);
  tft.print(F(" - "));
  tft.print(group.dosages[0]);

  if (group.count > 1)
  {
    tft.setCursor(x + 8, y + 45);
    tft.print(group.medications[1]);
    tft.print(F(" - "));
    tft.print(group.dosages[1]);
  }

  if (group.count > 2)
  {
    tft.setCursor(x + 8, y + 58);
    tft.print(F("+ "));
    tft.print(group.count - 2);
    tft.print(F(" more medications"));
  }
  else if (group.count <= 2)
  {
    tft.setCursor(x + 8, y + 58);
    tft.print(group.tubes[0]);
    if (group.count == 2)
    {
      tft.print(F(", "));
      tft.print(group.tubes[1]);
    }
  }

  if (isNext)
  {
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(x + width - 35, y + height - 15);
    tft.print(F("NEXT"));
  }
}

void drawNotification()
{
  if (!showNotification)
    return;

  int notifHeight = 80;
  if (strlen(notificationMessage) > 50)
  {
    notifHeight = 100;
  }

  int currentCountdown = 300 - (millis() - notificationStartTime) / 1000;

  bool needFullRedraw = (lastCountdownValue == -1) ||
                        (millis() - lastCountdownUpdate > 10000); // Update every 10 seconds

  if (needFullRedraw)
  {
    tft.fillRect(10, 80, 300, notifHeight, ST77XX_RED);
    tft.drawRect(9, 79, 302, notifHeight + 2, ST77XX_WHITE);

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(15, 90);
    tft.print(F("!! MEDICATION ALERT !!"));

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    int lineY = 105;
    int charsPerLine = 35;

    int msgLen = strlen(notificationMessage);
    int pos = 0;

    while (pos < msgLen && lineY < 80 + notifHeight - 20)
    {
      int lineEnd = pos + charsPerLine;
      if (lineEnd > msgLen)
        lineEnd = msgLen;

      if (lineEnd < msgLen)
      {
        while (lineEnd > pos && notificationMessage[lineEnd] != ' ')
        {
          lineEnd--;
        }
        if (lineEnd == pos)
          lineEnd = pos + charsPerLine;
      }

      tft.setCursor(15, lineY);
      for (int i = pos; i < lineEnd; i++)
      {
        tft.print(notificationMessage[i]);
      }

      pos = lineEnd;
      if (pos < msgLen && notificationMessage[pos] == ' ')
        pos++;
      lineY += 12;
    }

    tft.setTextSize(1);
    tft.setCursor(15, 80 + notifHeight - 25);
    tft.print(F("Press DROP button to dispense"));

    tft.fillRect(15, 80 + notifHeight - 15, 150, 10, ST77XX_RED);
    tft.setCursor(15, 80 + notifHeight - 15);
    tft.print(F("Auto-dismiss in "));
    tft.print(currentCountdown);
    tft.print(F("s"));

    lastCountdownUpdate = millis();
    lastCountdownValue = currentCountdown;
  }

  if (millis() - notificationStartTime > 300000)
  {
    showNotification = false;
    lastCountdownValue = -1; // Reset for next notification
  }
}

void startTubeSetupMode()
{
  setupMode = true;
  currentTubeSetup = 0;
  waitingForDropButton = false;

  setupTubeCount = 0;

  for (int i = 0; i < scheduleCount; i++)
  {
    bool tubeExists = false;

    for (int j = 0; j < setupTubeCount; j++)
    {
      if (strcmp(setupTubes[j], schedules[i].tube) == 0)
      {
        tubeExists = true;
        break;
      }
    }

    if (!tubeExists && setupTubeCount < 10)
    {
      strcpy(setupTubes[setupTubeCount], schedules[i].tube);
      setupTubeCount++;
    }
  }

  totalTubesNeeded = setupTubeCount;

  Serial.println(F("Starting tube setup mode"));
  Serial.print(F("Total unique tubes to configure: "));
  Serial.println(totalTubesNeeded);

  Serial.println(F("Unique tubes found:"));
  for (int i = 0; i < setupTubeCount; i++)
  {
    Serial.print(F("- "));
    Serial.println(setupTubes[i]);
  }
}

void showTubeSetupScreen()
{
  tft.fillScreen(ST77XX_BLACK);
  drawHeader();

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(50, 50);
  tft.print(F("TUBE SETUP"));

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(20, 80);
  tft.print(F("Tube "));
  tft.print(currentTubeSetup + 1);
  tft.print(F(" of "));
  tft.print(totalTubesNeeded);

  if (currentTubeSetup < setupTubeCount)
  {
    const char *currentTubeName = setupTubes[currentTubeSetup];

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(20, 100);
    tft.print(F("Put these medications:"));

    int medCount = 0;
    int displayY = 120;
    int totalAmount = 0;

    for (int i = 0; i < scheduleCount; i++)
    {
      if (strcmp(schedules[i].tube, currentTubeName) == 0)
      {
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(20, displayY);
        tft.print(schedules[i].medication);
        tft.print(F(" ("));
        tft.print(schedules[i].amount);
        tft.print(F("g)"));

        totalAmount += schedules[i].amount;
        medCount++;
        displayY += 15;

        if (medCount >= 4)
        {
          tft.setCursor(20, displayY);
          tft.print(F("+ more..."));
          break;
        }
      }
    }

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(20, 190);
    tft.print(F("Into TUBE: "));
    tft.print(currentTubeName);

    tft.setCursor(20, 205);
    tft.print(F("Total: "));
    tft.print(totalAmount);
    tft.print(F("g"));
  }

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(20, 220);
  if (waitingForDropButton)
  {
    bool blink = (millis() / 500) % 2;
    if (blink)
    {
      tft.print(F("Press DROP button when done"));
    }
  }
  else
  {
    tft.print(F("Place medication in tube"));
    tft.setCursor(20, 235);
    tft.print(F("then press DROP button"));
  }

  int barWidth = 280;
  int barHeight = 10;
  int barX = 20;
  int barY = 250;

  tft.drawRect(barX, barY, barWidth, barHeight, ST77XX_WHITE);
  int progress = (currentTubeSetup * barWidth) / totalTubesNeeded;
  tft.fillRect(barX + 1, barY + 1, progress, barHeight - 2, ST77XX_GREEN);
}

void handleTubeSetupButton()
{
  Serial.print(F("Tube "));
  Serial.print(currentTubeSetup + 1);
  Serial.println(F(" setup completed"));

  currentTubeSetup++;
  waitingForDropButton = false;

  if (currentTubeSetup >= totalTubesNeeded)
  {

    setupMode = false;
    Serial.println(F("Tube setup completed! System ready for automatic dispensing."));

    tft.fillScreen(ST77XX_BLACK);
    drawHeader();
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(50, 100);
    tft.print(F("SETUP"));
    tft.setCursor(50, 130);
    tft.print(F("COMPLETE!"));

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(20, 170);
    tft.print(F("System ready for"));
    tft.setCursor(20, 185);
    tft.print(F("automatic dispensing"));

    delay(3000);
  }
  else
  {
    waitingForDropButton = false;
  }
  requestTFTUpdate();
}

void showMainMenu()
{
  tft.fillScreen(ST77XX_BLACK);
  drawHeader();

  if (!setupMode && triggerSetupAfterBT && filestat && groupedCount > 0)
  {
    startTubeSetupMode();
    triggerSetupAfterBT = false;
  }

  if (setupMode)
  {
    showTubeSetupScreen();
    return;
  }

  if (checkMedicationTime() && !showNotification)
  {
    showNotification = true;
    notificationStartTime = millis();
    lastCountdownValue = -1; // Reset countdown tracking for fresh draw
  }

  if (showNotification)
  {
    drawNotification();
    return;
  }

  int contentY = 40;

  if (!filestat || groupedCount == 0)
  {
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(50, contentY + 50);
    tft.print(F("NO SCHEDULE DATA"));

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(50, contentY + 80);
    tft.print(F("Please load medication"));
    tft.setCursor(50, contentY + 95);
    tft.print(F("schedule via app"));
    return;
  }

  int nextMedIndex = findNextMedication();

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, contentY + 5);
  tft.print(F("MEDICATION SCHEDULE"));

  int cardY = contentY + 25;
  int cardsShown = 0;

  if (nextMedIndex != -1)
  {
    drawGroupedMedicationCard(10, cardY, 300, 75, groupedSchedules[nextMedIndex], true);
    cardY += 85;
    cardsShown++;
  }

  for (int i = 0; i < groupedCount && cardsShown < 3; i++)
  {
    if (i != nextMedIndex)
    {
      drawGroupedMedicationCard(10, cardY, 300, 75, groupedSchedules[i], false);
      cardY += 85;
      cardsShown++;
    }
  }

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 260);
  tft.print(F("Total schedules: "));
  tft.print(groupedCount);
  tft.print(F(" ("));
  tft.print(scheduleCount);
  tft.print(F(" doses)"));
}

bool checkJsonFile()
{
  File f = SD.open("data.json", FILE_READ);
  if (!f)
  {
    Serial.println(F("Cannot find data.json"));
    return false;
  }

  String jsonStr;
  while (f.available())
    jsonStr += (char)f.read();
  f.close();

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, jsonStr);

  if (err)
  {
    Serial.print(F("JSON syntax error: "));
    Serial.println(err.c_str());
    return false;
  }

  Serial.println(F("JSON is valid!"));
  return true;
}

bool initSD()
{
  // Ensure all CS pins are HIGH before anything
  digitalWrite(SD_CS, HIGH);
  digitalWrite(TFT_CS, HIGH);
  delay(100); // Let lines settle

  // Send dummy clocks with all CS high (required by SD spec)
  SPI.setClockDivider(SPI_CLOCK_DIV64); // Very slow for SD init
  for (int i = 0; i < 20; i++)
  {
    SPI.transfer(0xFF);
  }
  delay(50);

  // Now try SD init at slow speed
  SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz for SD

  for (int i = 0; i < 5; i++) // Increased retry count to 5
  {
    Serial.print(F("SD init attempt "));
    Serial.println(i + 1);

    // Ensure TFT is deselected
    digitalWrite(TFT_CS, HIGH);
    delay(10);

    if (SD.begin(SD_CS, SPI_HALF_SPEED)) // Use SPI_HALF_SPEED for more reliable init
    {
      Serial.println(F("SD initialized successfully."));
      digitalWrite(SD_CS, HIGH);
      delay(50);
      return true;
    }

    Serial.println(F("SD init failed, retrying..."));
    digitalWrite(SD_CS, HIGH);
    delay(300); // Longer delay between retries
  }
  return false;
}

void setup()
{
  Serial.begin(9600);
  Serial1.begin(115200);

  pinMode(53, OUTPUT); // Mega SS pin must be OUTPUT
  digitalWrite(53, HIGH);

  pinMode(SD_CS, OUTPUT);
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(DROP_BTN, INPUT_PULLUP);

  digitalWrite(SD_CS, HIGH);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);

  delay(100);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setDataMode(SPI_MODE0);
  delay(50);

  Serial.println(F("Initializing TFT first..."));
  digitalWrite(TFT_RST, LOW);
  delay(50);
  digitalWrite(TFT_RST, HIGH);
  delay(150);
  
  selectTFT();
  tft.init(240, 280);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(50, 120);
  tft.println(F("Loading..."));
  deselectAll();
  delay(100);
  Serial.println(F("TFT first init done."));

  // Send dummy clocks with all CS high
  SPI.setClockDivider(SPI_CLOCK_DIV64); // Slow for SD init
  for (int i = 0; i < 20; i++)
  {
    SPI.transfer(0xFF);
  }
  delay(50);

  Serial.println(F("Initializing SD card..."));
  if (!initSD())
  {
    Serial.println(F("Cannot initialize SD card!"));
    selectTFT();
    tft.fillScreen(ST77XX_RED);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(50, 100);
    tft.println(F("SD CARD ERROR!"));
    deselectAll();
    while (1);
  }

  digitalWrite(SD_CS, HIGH);
  delay(50);
  Serial.println(F("SD card ready."));

  Serial.println(F("Reinitializing TFT..."));
  SPI.setClockDivider(SPI_CLOCK_DIV4); // Back to fast speed for TFT
  
  // Full TFT reset sequence
  digitalWrite(TFT_RST, LOW);
  delay(50);
  digitalWrite(TFT_RST, HIGH);
  delay(150);

  // Flush any garbage on SPI bus
  for (int i = 0; i < 10; i++)
  {
    SPI.transfer(0xFF);
  }
  delay(10);

  selectTFT();
  tft.init(240, 280);
  tft.setRotation(1);
  deselectAll();
  delay(50);
  Serial.println(F("TFT reinitialized."));

  // 3) Show intro animation
  selectTFT();
  animatedIntro();
  deselectAll();

  // 4) Load schedule data from SD
  selectSD();
  filestat = loadScheduleData();
  deselectAll();

  // 5) RTC init (I2C, not SPI - no conflict)
  if (!rtc.begin())
    Serial.println(F("RTC not found!"));
  rtc.adjust(DateTime(2025, 8, 15, 6, 59, 30));

  // 6) Show main menu
  selectTFT();
  showMainMenu();
  deselectAll();

  // Servos and motors setup
  servo1.attach(A0);
  servo2.attach(A1);
  servo3.attach(A2);
  servo4.attach(A3);

  servo1.write(91);
  servo2.write(91);
  servo3.write(90);
  servo4.write(90);

  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(MOTOR_3, OUTPUT);
  pinMode(MOTOR_4, OUTPUT);
  pinMode(Sensor_PIN, INPUT);
  delay(200);

  Serial.println(F("Setup complete!"));
}

void loop()
{
  rtctime = rtc.now();

  // Event 1: Minute changed - update header time
  if (rtctime.minute() != lastDisplayedMinute)
  {
    lastDisplayedMinute = rtctime.minute();
    requestTFTUpdate();
    Serial.print(F("Time changed to "));
    Serial.print(rtctime.hour());
    Serial.print(F(":"));
    Serial.println(rtctime.minute());
  }

  // Event 2: Check for medication time (notification trigger)
  if (checkMedicationTime() && !showNotification)
  {
    showNotification = true;
    notificationStartTime = millis();
    lastCountdownValue = -1; // Reset countdown tracking for fresh draw
    requestTFTUpdate();
    Serial.println(F("Medication time - notification triggered"));
  }

  // Event 3: Notification state changed
  if (showNotification != lastNotificationState)
  {
    lastNotificationState = showNotification;
    lastCountdownValue = -1; // Reset on state change
    requestTFTUpdate();
  }

  if (showNotification && (millis() - lastCountdownUpdate > 10000))
  {
    requestTFTUpdate();
  }

  // Event 4: Filestat changed
  if (filestat != lastFilestat)
  {
    lastFilestat = filestat;
    requestTFTUpdate();
  }

  // Event 5: Schedule count changed
  if (groupedCount != lastGroupedCount)
  {
    lastGroupedCount = groupedCount;
    requestTFTUpdate();
  }

  if (digitalRead(DROP_BTN) == LOW)
  {
    delay(50);
    if (digitalRead(DROP_BTN) == LOW)
    {
      if (setupMode)
      {
        selectTFT();
        handleTubeSetupButton();
        deselectAll();
      }
      else if (showNotification)
      {
        handleDispensing();
      }
      delay(500);
    }
  }

  static int byteCounter = 0;
  static char tempBuffer[TEMP_BUFFER_SIZE + 1] = "";
  static int bufferPos = 0;

  while (Serial1.available())
  {
    char c = Serial1.read();
    Serial.print(c);
    if (bufferPos < TEMP_BUFFER_SIZE)
    {
      tempBuffer[bufferPos++] = c;
      tempBuffer[bufferPos] = '\0';
    }

    byteCounter++;
    lastByteTime = millis();

    if (!receiving)
    {
      char *startPos = strstr(tempBuffer, "#START#");
      if (startPos != nullptr)
      {
        receiving = true;
        receiveStartTime = millis();

        int startOffset = startPos - tempBuffer + 7;
        int remainingLen = bufferPos - startOffset;
        if (remainingLen > 0)
        {
          memmove(tempBuffer, tempBuffer + startOffset, remainingLen);
          bufferPos = remainingLen;
          tempBuffer[bufferPos] = '\0';
        }
        else
        {
          bufferPos = 0;
          tempBuffer[0] = '\0';
        }

        if (!startStreamingSave())
        {
          Serial.println(F("Failed to start streaming save"));
          receiving = false;
          bufferPos = 0;
          continue;
        }
        Serial.println(F("Started receiving JSON data..."));
      }
      else if (bufferPos >= TEMP_BUFFER_SIZE - 8)
      {
        memmove(tempBuffer, tempBuffer + TEMP_BUFFER_SIZE - 16, 16);
        bufferPos = 16;
        tempBuffer[bufferPos] = '\0';
      }
    }
    else
    {
      char *endPos = strstr(tempBuffer, "#END#");
      if (endPos != nullptr)
      {
        int finalLen = endPos - tempBuffer;
        if (finalLen > 0)
        {
          tempBuffer[finalLen] = '\0';
          String finalChunk = String(tempBuffer);
          writeStreamingChunk(finalChunk);
        }

        bool saved = finishStreamingSave();
        Serial.println(F("\nReceived complete JSON!"));
        Serial1.write('A');

        if (saved)
        {
          delay(2000);
          bool loaded = false;
          for (int attempt = 1; attempt <= 3; attempt++)
          {
            loaded = loadScheduleData();
            if (loaded)
            {
              Serial.print(F("Schedule loaded successfully after BT transfer (try "));
              Serial.print(attempt);
              Serial.println(F(")."));
              currentTubeSetup = 0;
              setupMode = false;
              triggerSetupAfterBT = true;
              break;
            }
            else
            {
              Serial.print(F("Schedule load failed after BT transfer (try "));
              Serial.print(attempt);
              Serial.println(F("). Retrying..."));
              delay(500);
            }
          }
          filestat = loaded;
          requestTFTUpdate();
          delay(2000);
        }
        else
        {
          filestat = false;
          Serial.println(F("Failed to save JSON to SD."));
          requestTFTUpdate();
        }

        int endOffset = (endPos - tempBuffer) + 5;
        int remainingLen = bufferPos - endOffset;
        if (remainingLen > 0)
        {
          memmove(tempBuffer, tempBuffer + endOffset, remainingLen);
          bufferPos = remainingLen;
          tempBuffer[bufferPos] = '\0';
        }
        else
        {
          bufferPos = 0;
          tempBuffer[0] = '\0';
        }

        receiving = false;
        Serial.println(F("Complete"));
        delay(300);
      }
      else
      {
        if (bufferPos >= TEMP_BUFFER_SIZE - 8)
        {
          String chunkToWrite = String(tempBuffer).substring(0, TEMP_BUFFER_SIZE / 2);
          writeStreamingChunk(chunkToWrite);

          int keepLen = bufferPos - (TEMP_BUFFER_SIZE / 2);
          memmove(tempBuffer, tempBuffer + (TEMP_BUFFER_SIZE / 2), keepLen);
          bufferPos = keepLen;
          tempBuffer[bufferPos] = '\0';
        }
      }
    }

    if (byteCounter >= 32)
    {
      byteCounter = 0;
    }
  }

  if (receiving)
  {
    if (millis() - lastByteTime > 5000)
    {
      Serial.println(F("Timeout: no new data, aborting streaming save."));
      if (streamingActive)
      {
        endStreamingSave();
      }
      bufferPos = 0;
      tempBuffer[0] = '\0';
      receiving = false;
    }
    else if (millis() - receiveStartTime > 20000)
    {
      Serial.println(F("Timeout: transmission too long, aborting streaming save."));
      if (streamingActive)
      {
        endStreamingSave();
      }
      bufferPos = 0;
      tempBuffer[0] = '\0';
      receiving = false;
    }
  }

  if (!receiving && tftNeedsUpdate)
  {
    selectTFT();
    showMainMenu();
    deselectAll();
    tftNeedsUpdate = false;
  }
}
