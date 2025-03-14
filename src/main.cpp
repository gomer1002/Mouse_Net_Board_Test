#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <serialEEPROM.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BMP280 bmp;

/*
 * Atmel AT24C256 EEPROM Memory
 * Device Address 0x50 (A0 = GND, A1 = GND)
 * 256K bit memory = 32768 Bytes
 * 64-Byte Page Write Buffer
 */
serialEEPROM myEEPROM(0x50, 32768, 64);

// On-Board LEDs
uint8_t LED_PC13 = PC13;

// LoRa LEDs
uint8_t LED_LED1 = PB8;
uint8_t LED_LED2 = PB9;

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

HardwareSerial MySerial1(UART1_RX, UART1_TX);
HardwareSerial MySerial2(UART2_RX, UART2_TX);
HardwareSerial MySerial3(UART3_RX, UART3_TX);

// On-Board buttons
uint8_t STM_BTN1 = PB0;
uint8_t LORA_PA0 = PB1;
uint8_t LORA_RST = PA6;

// uint8_t STM_SW1 = PC13;
uint8_t STM_SW2 = PC14;
uint8_t STM_SW3 = PC15;
uint8_t STM_SW4 = PA0;
uint8_t STM_SW5 = PA1;
uint8_t STM_SW6 = PA7;

// SIM868 GPIO
uint8_t SIM_SLEEP = PA4;
uint8_t SIM_PWRK = PA5;

// On-Board button pins list
uint8_t btns[] = {
    STM_BTN1,
    LORA_PA0,
    LORA_RST,
};

uint8_t switches[] = {
    // STM_SW1,
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
float temperature;
float pressure;
float altitude;

void setup();
int checkBtns();
void loop();
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

void setup()
{
  // led init
  pinMode(LED_PC13, OUTPUT);
  digitalWrite(LED_PC13, HIGH);

  // LoRa leds init
  pinMode(LED_LED1, INPUT);
  pinMode(LED_LED2, INPUT);

  // buttons init
  pinMode(STM_BTN1, INPUT);
  pinMode(LORA_PA0, INPUT);
  pinMode(LORA_RST, INPUT);

  // SIM init
  pinMode(SIM_SLEEP, INPUT);
  // digitalWrite(SIM_SLEEP, HIGH);
  pinMode(SIM_PWRK, OUTPUT);

  // switches init
  for (int i = 0; i < switchesSize; i++)
  {
    pinMode(switches[i], INPUT_PULLUP);
  }

  // buttons init
  pinMode(btns[0], INPUT_PULLUP);
  pinMode(btns[1], INPUT);
  pinMode(btns[2], INPUT_PULLUP);

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
  digitalWrite(SIM_PWRK, HIGH);
  digitalWrite(LED_PC13, LOW);
  delay(100);
  digitalWrite(SIM_PWRK, LOW);
  digitalWrite(LED_PC13, HIGH);
  delay(1000);
  digitalWrite(SIM_PWRK, HIGH);
  digitalWrite(LED_PC13, LOW);
  delay(3000);

  display.setCursor(xpos, ypos);
  display.fillRect(xpos, ypos, 128, ypos + 8, SSD1306_BLACK);
  display.print("CHECKING...");
  display.display();
  MySerial1.write("Checking SIM868 ...\n");
  info_blink(LED_PC13, 1);
  MySerial3.write("AT\n");
  delay(100);

  c = "FAIL";
  while (MySerial3.available() > 0)
  {
    c = MySerial3.readString();
  }
  int ok_ind = c.indexOf("OK");
  // c.replace("\r\n", "");
  // c.replace("AT", "");
  if (ok_ind != -1)
  {
    c = "OK";
  }

  MySerial1.println(c);
  display.fillRect(xpos, ypos, 128, ypos + 8, SSD1306_BLACK);
  display.setCursor(xpos, ypos);
  display.println(c);
  display.display();
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
    display.println(bmp.sensorID());
    display.display();
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }

  // ===============================================
  // AT24C256
  display.print("AT24C256: ");
  MySerial1.write("\n\nChecking AT24C256 ...\n");
  display.display();
  info_blink(LED_PC13, 1);
  delay(100);

  uint8_t data_w = 0xab;
  uint8_t data_r = 0x00;

  // Write byte (Address 0x10)
  myEEPROM.write(0x10, data_w);

  display.print("W-OK ");
  MySerial1.println("Write OK");
  display.display();
  info_blink(LED_PC13, 2);

  // Read byte (Address 0x10)
  data_r = myEEPROM.read(0x10);

  if (data_r == data_w)
  {
    MySerial1.println("Read OK");
    display.println("R-OK");
  }
  else
  {
    MySerial1.println("Read FAIL");
    display.println("R-FAIL");

    display.print(data_w);
    display.print(" != ");
    display.println(data_r);
  }
  display.display();
  info_blink(LED_PC13, 2);
  delay(500);

  MySerial1.println("\nALL DONE\n");
}

void loop()
{
  info_blink(LED_PC13, 1);
  io_tick();

  if (millis() - bmpTimer > bmp_delay)
  {
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure() / 133.3;
    altitude = bmp.readAltitude(1031);

    MySerial1.print(F("Temperature = "));
    MySerial1.print(temperature);
    MySerial1.println(" C");

    MySerial1.print(F("Pressure = "));
    MySerial1.print(pressure);
    MySerial1.println(" mmHg");

    MySerial1.print(F("Approx altitude = "));
    MySerial1.print(altitude);
    MySerial1.println(" m");

    MySerial1.println();

    bmpTimer = millis();
  }

  display.fillRect(0, 40, 128, 64, SSD1306_BLACK);
  display.setCursor(0, 40);
  display.print(temperature);

  display.print(" ");
  display.print(pressure);

  display.print(" ");
  display.print(altitude);

  display.println();

  display.println(io_state());
  display.display();

  delay(50);
}
