#include <Arduino.h>
#include <array>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BMP280 bmp;

// Winbond SPI
uint8_t SPI_CS = PA4;
uint8_t SPI_SCK = PA5;
uint8_t SPI_MISO = PA6;
uint8_t SPI_MOSI = PA7;

// On-Board LEDs
uint8_t LED_PC13 = PC13;

// I2C
// uint8_t SDA = PB7;
// uint8_t SCL = PB6;

// UART1
uint8_t UART1_TX = PA9;
uint8_t UART1_RX = PA10;

// LoRa UART
uint8_t UART2_TX = PA2;
uint8_t UART2_RX = PA3;

// SIM868 UART
uint8_t UART3_TX = PB10;
uint8_t UART3_RX = PB11;

// UART definition
HardwareSerial MySerial1(UART1_RX, UART1_TX);
HardwareSerial MySerial2(UART2_RX, UART2_TX);
HardwareSerial MySerial3(UART3_RX, UART3_TX);

// On-Board buttons
uint8_t STM_LT = PB3;
uint8_t STM_DN = PA15;
uint8_t STM_OK = PA12;
uint8_t STM_RT = PA11;
uint8_t STM_UP = PA8;

// On-Board switches
uint8_t STM_SW1 = PC14;
uint8_t STM_SW2 = PC15;
uint8_t STM_SW3 = PA0;
uint8_t STM_SW4 = PA1;
uint8_t STM_SW5 = PB0;
uint8_t STM_SW6 = PB1;

// SIM868 GPIO
uint8_t SIM_PWRK = PB15;

// On-Board button pins list
uint8_t btns[] = {
    STM_LT,
    STM_DN,
    STM_OK,
    STM_RT,
    STM_UP,
};

// On-Board switch pins list
uint8_t switches[] = {
    STM_SW1,
    STM_SW2,
    STM_SW3,
    STM_SW4,
    STM_SW5,
    STM_SW6,
};

const int btnsSize = sizeof(btns) / sizeof(uint8_t);
uint8_t btnsState[btnsSize] = {0};

const int switchesSize = sizeof(switches) / sizeof(uint8_t);
uint8_t switchesState[switchesSize] = {0};

int bmp_delay = 500;
int bmpTimer = millis();
bool bmpStarted = false;
float temperature;
float pressure;
float altitude;

int gps_delay = 800;
int gpsTimer = millis();
bool gpsStarted = false;

void setup();
int checkBtns();
void loop();
void io_tick();
String io_state();
void info_blink(int led, int count);

////////////////////////////////////////////////////////
void info_blink(int led, int count = -1)
{
  for (int i = 0; i < count; i++)
  {
    digitalWrite(led, LOW);
    delay(50);
    digitalWrite(led, HIGH);
    delay(50);
  }
}

void io_tick()
{
  for (int i = 0; i < btnsSize; i++)
  {
    btnsState[i] = digitalRead(btns[i]);
  }

  for (int i = 0; i < switchesSize; i++)
  {
    switchesState[i] = digitalRead(switches[i]);
  }
}

String io_state()
{
  String s = "SW:";
  for (int i = 0; i < switchesSize; i++)
  {
    s += String(switchesState[i]);
  }
  s += " BTN:";
  for (int i = 0; i < btnsSize; i++)
  {
    s += String(btnsState[i]);
  }
  return s;
}

std::array<String, 2> parse_UTS_date_time(String UTS_date_time, int offset = 0)
{
  int year, month, day, hour, minute, second;
  year = UTS_date_time.substring(0, 4).toInt();
  month = UTS_date_time.substring(4, 6).toInt();
  day = UTS_date_time.substring(6, 8).toInt();
  hour = UTS_date_time.substring(8, 10).toInt();
  minute = UTS_date_time.substring(10, 12).toInt();
  second = UTS_date_time.substring(12, 14).toInt();

  char date[11]; // Массив для строки DD.MM.YYYY (10 символов + '\0')
  sprintf(date, "%02d.%02d.%04d", day, month, year);

  char time[9]; // Массив для строки DD.MM.YYYY (10 символов + '\0')
  sprintf(time, "%02d:%02d:%02d", hour, minute, second);

  return {date, time};
}

std::array<String, 21> parse_CGNSINF(String CGNSINF, bool &parse_ok)
{
  std::array<String, 21> values = {}; // Инициализация пустого массива
  for (int i = 0; i < 21; i++)
  {
    values[i] = "";
  }

  // trimming off the prefix
  CGNSINF = CGNSINF.substring(CGNSINF.indexOf(' ', 0) + 1, CGNSINF.length());

  // counting commas
  int commas = 0;
  for (int i = 0; i < CGNSINF.length(); i++)
  {
    if (CGNSINF.charAt(i) == ',')
      commas++;
  }
  // MySerial1.println("CGNSINF = " + CGNSINF);
  // MySerial1.println("commas = " + String(commas));
  // MySerial1.println("length = " + String(CGNSINF.length()));
  // MySerial1.println("");
  if (commas != 20)
    return values;

  // +UGNSINF: 1,0,19800106002005.462,,,,0.00,0.0,0,,,,,,0,0,0,,,,

  // +UGNSINF: 1,0,20250524084650.300,,,,0.00,0.0,0,,,,,,3,0,,,29,,
  // +UGNSINF: 1,0,20250524084653.304,,,,3.06,283.3,0,,,,,,3,0,,,29,
  // +UGNSINF: 1,1,20250524085106.000,56.452375,84.961512,157.071,0.
  int idx = 0;
  int i = 0;
  String value;
  do
  {
    int start = CGNSINF.indexOf(',', idx);
    if (start == -1)
      break;

    value = CGNSINF.substring(idx, start);
    values[i] = value;

    idx = start + 1;
    i++;
  } while (idx < CGNSINF.length());

  // String GNSS_run = values[0];
  // String GNSS_fix = values[1];
  // String UTC_date_time = values[2];
  // String lattitude = values[3];
  // String lontitude  = values[4];
  // String MSL_altitude = values[5];
  // String speed_over_ground = values[6];
  // String course_over_ground = values[7];
  // String fix_mode   = values[8];
  // String reserved1  = values[9];
  // String HDOP = values[10];
  // String PDOP = values[11];
  // String VDOP   = values[12];
  // String reserved2  = values[13];
  // String GPS_satelites_in_view = values[14];
  // String GNSS_satelites_used = values[15];
  // String GLONASS_satelites_in_view = values[16];
  // String reserved3  = values[17];
  // String C_N0_max = values[18];
  // String HPA_2 = values[19];
  // String VPA_2 = values[20];

  parse_ok = true;
  return values;
}

void setup()
{
  // led init
  pinMode(LED_PC13, OUTPUT);
  digitalWrite(LED_PC13, HIGH);

  // buttons init
  for (int i = 0; i < btnsSize; i++)
  {
    // all buttons are externally pulled to 3V3
    // active LOW
    pinMode(btns[i], INPUT);
  }

  // SIM init
  pinMode(SIM_PWRK, INPUT_PULLUP);

  // switches init
  for (int i = 0; i < switchesSize; i++)
  {
    // switches do not pulled to anything
    // need software pullup
    // active LOW
    pinMode(switches[i], INPUT_PULLUP);
  };

  MySerial1.begin(115200);
  MySerial2.begin(115200);
  MySerial3.begin(115200);
  delay(500);

  // info_blink(LED_PC13, 5);

  // ===============================================
  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    MySerial1.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

  display.setTextSize(1);
  display.setRotation(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.cp437(true);
  display.clearDisplay();

  display.println("OLED: OK");
  display.display();

  // ===============================================
  // LoRa
  display.print("LoRa: ");
  MySerial1.write("Checking LoRa ...\n");
  display.display();
  info_blink(LED_PC13, 1);
  MySerial2.write("AT+DEVTYPE=?");
  delay(100);

  String c = "FAIL";
  while (MySerial2.available() > 0)
  {
    c = MySerial2.readString();
  }
  c.replace("\r\n", "");
  c.replace("DEVTYPE=", "");

  MySerial1.println(c);
  display.println(c);
  display.display();
  info_blink(LED_PC13, 3);
  delay(500);

  // ===============================================
  // SIM868
  display.print("SIM868: ");
  display.display();
  int xpos = display.getCursorX();
  int ypos = display.getCursorY();

  MySerial1.write("\n\nBooting SIM868 ...\n");
  display.setCursor(xpos, ypos);
  display.print("BOOTING...");
  display.display();
  // pressing SIM868 PWRK pin to boot it
  pinMode(SIM_PWRK, OUTPUT);
  digitalWrite(SIM_PWRK, LOW);
  digitalWrite(LED_PC13, HIGH);
  delay(1000);
  pinMode(SIM_PWRK, INPUT);
  digitalWrite(LED_PC13, LOW);
  delay(2000);

  // ===============================================
  // ===============================================
  // SIM868
  MySerial1.write("Checking SIM868 ...\n");
  info_blink(LED_PC13, 1);
  display.fillRect(xpos, ypos, 128, 8, SSD1306_BLACK);
  display.setCursor(xpos, ypos);
  display.print("AT? ...");
  display.display();
  MySerial3.write("AT\n");
  delay(100);

  c = "AT=FAIL";
  while (MySerial3.available() > 0)
  {
    c = MySerial3.readString();
  }
  int ok_ind = c.indexOf("OK");
  MySerial1.write("ANS: ");
  MySerial1.println(c);

  // c.replace("\r\n", "");
  // c.replace("AT", "");
  if (ok_ind != -1)
  {
    c = "AT=OK";
  }

  MySerial1.println(c);
  display.fillRect(xpos, ypos, 128, 8, SSD1306_BLACK);
  display.setCursor(xpos, ypos);
  display.print(c);
  display.display();

  // ===============================================
  // ===============================================
  // GPS
  MySerial1.write("Enabling GPS ...\n");
  info_blink(LED_PC13, 1);
  display.fillRect(xpos, ypos, 128, 8, SSD1306_BLACK);
  display.setCursor(xpos, ypos);
  display.print("PWR? ...");
  display.display();
  MySerial3.write("AT+CGNSPWR=1\n");
  delay(500);
  c = "PWR=FAIL";
  while (MySerial3.available() > 0)
  {
    c = MySerial3.readString();
  }
  MySerial1.write("ANS: ");
  MySerial1.println(c);
  ok_ind = c.indexOf("CGNSPWR");
  if (ok_ind != -1)
  {
    c = "PWR=OK";
  }
  MySerial1.println(c);
  display.fillRect(xpos, ypos, 128, 8, SSD1306_BLACK);
  display.setCursor(xpos, ypos);
  display.print(c);
  display.display();

  // ===============================================
  // ===============================================
  // GPS version
  MySerial1.write("Checking GPS version ...\n");
  info_blink(LED_PC13, 1);
  display.fillRect(xpos, ypos, 128, 8, SSD1306_BLACK);
  display.setCursor(xpos, ypos);
  display.print("VER? ...");
  display.display();
  MySerial3.write("AT+CGNSVER\n");
  delay(500);
  c = "VER=FAIL";
  while (MySerial3.available() > 0)
  {
    c = MySerial3.readString();
  }
  MySerial1.write("ANS: ");
  MySerial1.println(c);
  ok_ind = c.indexOf("AXN");
  if (ok_ind != -1)
  {
    c = "VER=OK";
  }
  MySerial1.println(c);
  display.fillRect(xpos, ypos, 128, 8, SSD1306_BLACK);
  display.setCursor(xpos, ypos);
  display.print(c);
  display.display();

  // ===============================================
  // ===============================================
  // Enabling NMEA
  // MySerial1.write("Enabling NMEA ...\n");
  // info_blink(LED_PC13, 1);
  // display.fillRect(xpos, ypos, 128, 8, SSD1306_BLACK);
  // display.setCursor(xpos, ypos);
  // display.print("NMEA? ...");
  // display.display();
  // MySerial3.write("AT+CGNSURC=1\n");
  // delay(100);
  // c = "NMEA=OK";
  // MySerial1.println(c);
  // display.fillRect(xpos, ypos, 128, 8, SSD1306_BLACK);
  // display.setCursor(xpos, ypos);
  // display.println(c);
  // display.display();

  info_blink(LED_PC13, 3);
  delay(500);

  // ===============================================
  // BMP280
  display.print("BMP280: ");
  MySerial1.write("\n\nChecking BMP280 ...\n");
  display.display();
  info_blink(LED_PC13, 1);
  delay(100);

  unsigned status;
  c = "FAIL";
  status = bmp.begin(0x76);
  if (!status)
  {
    MySerial1.println(c);
    MySerial1.println(bmp.sensorID());
    display.println(c);
    display.display();
  }
  else
  {
    MySerial1.println(bmp.sensorID());
    display.println("OK");
    display.display();
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    bmpStarted = true;
  }

  // ===============================================
  // W25Q128 / W25Q32
  // auto detect W25Q128 or W25Q32 by probing sectors
  MySerial1.write("\n\nChecking W25Q32 ...\n");
  // flash.begin();
  display.print("W25Q32: ");
  // uint8_t b1, b2, b3;
  // uint32_t JEDEC = flash.getJEDECID();
  // // uint16_t ManID = flash.getManID();
  // b1 = (JEDEC >> 16);
  // b2 = (JEDEC >> 8);
  // b3 = (JEDEC >> 0);
  display.printf("TODO");
  // display.printf("W25Q32: %02xh|%02xh|%02xh", b1, b2, b3);
  MySerial1.printf("W25Q32: need to be implemented\n");
  // MySerial1.printf("Manufacturer ID: %02xh\nMemory Type: %02xh\nCapacity: %02xh", b1, b2, b3);

  // AT24C256
  // display.print("AT24C256: ");
  // display.display();
  // info_blink(LED_PC13, 1);
  // delay(100);

  // uint8_t data_w = random(0, 256);
  // uint8_t data_r = 0x00;

  // // Write byte (Address 0x10)
  // myEEPROM.write(0x10, data_w);

  // display.print("W-OK ");
  // MySerial1.println("Write OK");
  // display.display();
  // info_blink(LED_PC13, 2);

  // // Read byte (Address 0x10)
  // data_r = myEEPROM.read(0x10);

  // if (data_r == data_w)
  // {
  //   MySerial1.println("Read OK");
  //   display.println("R-OK");
  // }
  // else
  // {
  //   MySerial1.println("Read FAIL");
  //   display.println("R-FAIL");

  //   display.print(data_w);
  //   display.print(" != ");
  //   display.println(data_r);
  // }
  // display.display();
  // info_blink(LED_PC13, 2);
  // delay(500);

  MySerial1.println("\nALL DONE\n");
  MySerial3.flush();
}

void loop()
{
  info_blink(LED_PC13, 1);
  io_tick();

  if ((bmpStarted) && (millis() - bmpTimer > bmp_delay))
  {
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure() / 133.3;
    altitude = bmp.readAltitude(1031);
    bmpTimer = millis();
  }

  // BMP info
  if (bmpStarted)
  {
    display.fillRect(0, 0, 128, 8, SSD1306_BLACK);
    display.setCursor(0, 0);
    display.print(temperature);

    display.print(" ");
    display.print(pressure);

    display.print(" ");
    display.print(altitude);
    display.display();

    // MySerial1.printf("Temperature: %f C, Pressure: %f mmHg, Altitude: %f m\n", temperature, pressure, altitude);
  }

  // io status
  display.fillRect(0, 8, 128, 8, SSD1306_BLACK);
  display.setCursor(0, 8);
  display.println(io_state());
  display.display();

  // GPS status
  if (millis() - gpsTimer > gps_delay)
  {
    MySerial3.println("AT+CGNSINF");
    delay(5);
    if (MySerial3.available())
    {
      String cgnsinf;
      do
      {
        cgnsinf = MySerial3.readStringUntil('\n');
      } while (cgnsinf.indexOf('+CGNSINF:') == -1);

      // t = MySerial3.readStringUntil('\n');
      // cgnsinf = MySerial3.readStringUntil('\n');
      // t = MySerial3.readStringUntil('\n');
      // t = MySerial3.readStringUntil('\n');
      MySerial1.println("CGNSINF: " + cgnsinf);

      bool parse_ok = false;
      std::array<String, 21> parsed_cgnsinf = parse_CGNSINF(cgnsinf, parse_ok);
      MySerial1.println("parse_ok = " + String(parse_ok));

      if (parse_ok)
      {
        std::array<String, 2> date_time = parse_UTS_date_time(parsed_cgnsinf[2]);

        // clear display
        display.fillRect(0, 16, 128, 64, SSD1306_BLACK);
        display.setCursor(0, 24);
        display.printf("RUN: %2d     FIX: %2d\n", parsed_cgnsinf[0].toInt(), parsed_cgnsinf[1].toInt());
        display.printf("GPS: %2d     GLO: %2d\n", parsed_cgnsinf[14].toInt(), parsed_cgnsinf[16].toInt());
        display.printf("%s  %s\n", date_time[1].c_str(), date_time[0].c_str());
        display.printf("LAT: %s \nLON: %s\n", parsed_cgnsinf[3].c_str(), parsed_cgnsinf[4].c_str());
        display.display();
      }
    }
    gpsTimer = millis();
  }
}
