// Board configuration
#ifndef USE_RF95
#define USE_RF95
#endif

#ifndef USING_SX1276
#define USING_SX1276
#endif

// Core includes first
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Project configuration
#include "utilities.h"

// Libraries
#include <WiFi.h>
#include <esp_adc_cal.h>
#include <RadioLib.h>
#include <AceButton.h>
#include <TFT_eSPI.h>
#include <TinyGPSPlus.h>

// Board support
#include "LoRaBoards.h"

using namespace ace_button;

// Display setup
TFT_eSPI tft = TFT_eSPI();

// GPS Setup
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

#if defined(USE_RF95) || defined(USING_SX1276)
// Radio module configuration
#define CONFIG_RADIO_FREQ           915.0
#define CONFIG_RADIO_OUTPUT_POWER   30
#define CONFIG_RADIO_BW            125.0

// Use SX1276 module with correct pin mapping
SX1276 radio = new Module(
    RADIO_CS_PIN,      // CS Pin 
    RADIO_DIO0_PIN,    // DIO0 Pin
    RADIO_RST_PIN,     // Reset Pin 
    RADIO_DIO1_PIN     // DIO1 Pin
);
#else
#error "Please define either USE_RF95 or USING_SX1276"
#endif

// Forward declarations
void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState);
void setFlag(void);
void setupDisplay();
void updateDisplay();

// Variables for radio operation
int state = RADIOLIB_ERR_NONE;
bool transmittedFlag = false;
uint32_t transmissionCounter = 0;
uint32_t recvCounter = 0;
float radioRSSI = 0;
bool isRadioOnline = false;
uint32_t displayUpdateInterval = 0;
uint32_t lastRssiUpdate = 0;

// GPS Variables
float gpsLat = 0.0;
float gpsLon = 0.0;
int gpsSats = 0;

// Button setup
AceButton button;

// Interrupt callback
void setFlag(void) {
    transmittedFlag = true;
}

void setupDisplay() {
    pinMode(TFT_MOSI, OUTPUT);
    pinMode(TFT_SCLK, OUTPUT);
    pinMode(TFT_CS, OUTPUT);
    pinMode(TFT_DC, OUTPUT);
    pinMode(TFT_RST, OUTPUT);
    pinMode(TFT_BL, OUTPUT);
    
    digitalWrite(TFT_CS, HIGH);
    digitalWrite(TFT_BL, HIGH);
    
    SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
    
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(3);
    tft.drawString("T-BEAM", 10, 10);
    tft.setTextSize(2);
    tft.drawString("Initializing...", 10, 50);
}

void updateDisplay() {
    static bool firstRun = true;
    static int lastTX = -1;
    static int lastRX = -1;
    static float lastRSSI = -999;
    static int lastSats = -1;
    static float lastLat = -999;
    static float lastLon = -999;
    static uint32_t lastStatusUpdate = 0;
    
    if (firstRun) {
        tft.fillRect(0, 80, tft.width(), tft.height() - 80, TFT_BLACK);
        tft.drawFastVLine(tft.width()/2 + 10, 85, 150, TFT_DARKGREY);
        firstRun = false;
    }

    int leftX = 10;
    int leftValX = 130;
    int rightX = tft.width()/2 + 30;
    int rightValX = rightX + 110;

    tft.setTextSize(2);
    
    // Left Column - Radio info
    if (lastTX != transmissionCounter || lastRX != recvCounter) {
        // Radio Status
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawString("Radio:", leftX, 90);
        tft.fillRect(leftValX, 90, 140, 20, TFT_BLACK);
        tft.setTextColor(isRadioOnline ? TFT_GREEN : TFT_RED, TFT_BLACK);
        tft.drawString(isRadioOnline ? "ONLINE" : "OFFLINE", leftValX, 90);

        // TX Count
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.drawString("TX Count:", leftX, 120);
        tft.fillRect(leftValX, 120, 140, 20, TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString(String(transmissionCounter), leftValX, 120);
        
        // RX Count
        tft.setTextColor(TFT_CYAN, TFT_BLACK);
        tft.drawString("RX Count:", leftX, 150);
        tft.fillRect(leftValX, 150, 140, 20, TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString(String(recvCounter), leftValX, 150);
        
        lastTX = transmissionCounter;
        lastRX = recvCounter;
    }

    // RSSI
    if (abs(lastRSSI - radioRSSI) > 0.5) {
        tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
        tft.drawString("RSSI:", leftX, 180);
        tft.fillRect(leftValX, 180, 140, 20, TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString(String(radioRSSI, 2) + " dBm", leftValX, 180);
        lastRSSI = radioRSSI;
    }

    // Right Column - GPS info
    if (lastSats != gpsSats) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawString("GPS Sats:", rightX, 90);
        tft.fillRect(rightValX, 90, 50, 20, TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString(String(gpsSats), rightValX, 90);
        lastSats = gpsSats;
    }

    if (abs(lastLat - gpsLat) > 0.000001) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawString("Lat:", rightX, 120);
        tft.fillRect(rightX, 150, 160, 20, TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString(String(gpsLat, 6), rightX + 10, 150);
        lastLat = gpsLat;
    }

    if (abs(lastLon - gpsLon) > 0.000001) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawString("Lon:", rightX, 180);
        tft.fillRect(rightX, 210, 160, 20, TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString(String(gpsLon, 6), rightX + 10, 210);
        lastLon = gpsLon;
    }

    // Bottom status bar
    if (millis() - lastStatusUpdate > 1000) {
        tft.fillRect(0, 240, tft.width(), 30, TFT_BLACK);
        tft.drawFastHLine(10, 240, tft.width() - 20, TFT_DARKGREY);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.drawString("Click = TX       Long Press = Sleep", leftX, 260);
        lastStatusUpdate = millis();
    }
}

void handleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {
    switch (eventType) {
    case AceButton::kEventClicked:
        {
            Serial.println("Start transmit");
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.drawString("Transmitting...", 10, 260);
            
            if (isRadioOnline) {
                state = radio.transmit((uint8_t *)&transmissionCounter, 4);
                if (state != RADIOLIB_ERR_NONE) {
                    Serial.println(F("[Radio] transmit packet failed!"));
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString("TX Failed!", 10, 260);
                } else {
                    transmissionCounter++;
                    radioRSSI = radio.getRSSI();
                }
            }
        }
        break;

    case AceButton::kEventLongPressed:
        {
            Serial.println("Long pressed - entering sleep");
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setTextSize(2);
            tft.drawString("Entering Sleep Mode", 10, 100);
            tft.drawString("Press button to wake", 10, 140);
            delay(2000);
            
            if (isRadioOnline) {
                radio.sleep();
            }
            gpsSerial.end();
            tft.fillScreen(TFT_BLACK);
            digitalWrite(TFT_BL, LOW);
            
            esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);
            esp_sleep_enable_ext1_wakeup(_BV(BUTTON_PIN), ESP_EXT1_WAKEUP_ALL_LOW);
            esp_sleep_enable_timer_wakeup(30 * 1000 * 1000);
            
            Serial.println("Going to sleep...");
            delay(100);
            esp_deep_sleep_start();
        }
        break;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println("Starting T-BEAM initialization...");
    
    // Initialize Wire for I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(250);

    // Initialize SPI for radio with explicit pin assignments
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    delay(250);

    // Initialize display first
    Serial.println("Setting up display...");
    setupDisplay();
    delay(250);

    // Initialize GPS
    Serial.println("Setting up GPS...");
    gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(250);

    // Initialize radio
    Serial.println("Setting up Radio...");
    tft.drawString("Initializing Radio...", 10, 80);

    pinMode(RADIO_MISO_PIN, INPUT);
    pinMode(RADIO_MOSI_PIN, OUTPUT);
    pinMode(RADIO_SCLK_PIN, OUTPUT);
    pinMode(RADIO_CS_PIN, OUTPUT);
    pinMode(RADIO_RST_PIN, OUTPUT);
    pinMode(RADIO_BUSY_PIN, INPUT);
    pinMode(RADIO_DIO1_PIN, INPUT);

    digitalWrite(RADIO_CS_PIN, HIGH);
    delay(100);

    // Hard reset of radio
    digitalWrite(RADIO_RST_PIN, LOW);
    delay(100);
    digitalWrite(RADIO_RST_PIN, HIGH);
    delay(100);

    // Initialize radio with debug output
    Serial.println("\nRadio Setup Details:");
    Serial.println("===================");
    Serial.println("Pin Configuration:");
    Serial.printf("MISO Pin: %d, State: %d\n", RADIO_MISO_PIN, digitalRead(RADIO_MISO_PIN));
    Serial.printf("MOSI Pin: %d, State: %d\n", RADIO_MOSI_PIN, digitalRead(RADIO_MOSI_PIN));
    Serial.printf("SCK Pin:  %d, State: %d\n", RADIO_SCLK_PIN, digitalRead(RADIO_SCLK_PIN));
    Serial.printf("CS Pin:   %d, State: %d\n", RADIO_CS_PIN, digitalRead(RADIO_CS_PIN));
    Serial.printf("RST Pin:  %d, State: %d\n", RADIO_RST_PIN, digitalRead(RADIO_RST_PIN));
    Serial.printf("BUSY Pin: %d, State: %d\n", RADIO_BUSY_PIN, digitalRead(RADIO_BUSY_PIN));
    Serial.printf("DIO1 Pin: %d, State: %d\n", RADIO_DIO1_PIN, digitalRead(RADIO_DIO1_PIN));

    // Begin radio
    Serial.println("Attempting radio initialization...");
    state = radio.begin();
    Serial.printf("Radio init state: %d\n", state);
    
    // Translate error code
    switch(state) {
        case RADIOLIB_ERR_NONE:
            Serial.println("Radio initialization successful!");
            break;
        case RADIOLIB_ERR_CHIP_NOT_FOUND:
            Serial.println("Error: Radio chip not found!");
            break;
        case RADIOLIB_ERR_SPI_WRITE_FAILED:
            Serial.println("Error: SPI write failed!");
            break;
        case RADIOLIB_ERR_INVALID_SYNC_WORD:
            Serial.println("Error: Invalid sync word!");
            break;
        default:
            Serial.printf("Error: Unknown error code %d\n", state);
            break;
    }

    if (state == RADIOLIB_ERR_NONE) {
        isRadioOnline = true;
        Serial.println(F("[Radio] success!"));
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawString("Radio Online!", 10, 110);
        
        delay(100);
        
        // Configure radio parameters
        state = radio.setFrequency(CONFIG_RADIO_FREQ);
        Serial.println("Set Frequency: " + String(state));
        delay(50);
        
        state = radio.setBandwidth(CONFIG_RADIO_BW);
        Serial.println("Set Bandwidth: " + String(state));
        delay(50);
        
        state = radio.setSpreadingFactor(12);
        Serial.println("Set SF: " + String(state));
        delay(50);
        
        state = radio.setCodingRate(6);
        Serial.println("Set CR: " + String(state));
        delay(50);

        state = radio.setSyncWord(0xAB);
        Serial.println("Set Sync: " + String(state));
        delay(50);
        
        state = radio.setOutputPower(CONFIG_RADIO_OUTPUT_POWER);
        Serial.println("Set Power: " + String(state));
        delay(50);
        
        state = radio.setPreambleLength(16);
        Serial.println("Set Preamble: " + String(state));
        delay(50);

        radio.setCRC(false);
        delay(50);

        // Configure DIO pins for interrupt
        radio.setDio0Action(setFlag, RISING);
        delay(50);

#ifdef USING_DIO2_AS_RF_SWITCH
        state = radio.setDio2AsRfSwitch();
        Serial.println("Set DIO2 as RF Switch: " + String(state));
        delay(50);
#endif
        
        Serial.println("Starting receive mode...");
        state = radio.startReceive();
        Serial.println("Receive state: " + String(state));
    } else {
        Serial.println(F("[Radio] Failed to initialize!"));
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString("Radio Failed!", 10, 110);
    }

    // Button setup
    Serial.println("Setting up button...");
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    button.init(BUTTON_PIN);
    ButtonConfig *buttonConfig = button.getButtonConfig();
    buttonConfig->setEventHandler(handleEvent);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);

    // Show main screen
    Serial.println("Setup complete - showing main screen");
    delay(1000);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(3);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("T-BEAM", 10, 20);  // Moved down from 10
    tft.drawString("RADIO", 10, 55);   // Moved down from 40
    updateDisplay();
}

void loop() {
    static uint32_t lastGPSUpdate = 0;
    static uint32_t lastRSSIUpdate = 0;
    uint32_t currentMillis = millis();
    
    // Check button state
    button.check();
    
    // Process GPS data as it arrives
    while (gpsSerial.available()) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isValid()) {
                gpsLat = gps.location.lat();
                gpsLon = gps.location.lng();
            }
            gpsSats = gps.satellites.value();
        }
    }
    
    // Update RSSI periodically if radio is online
    if (isRadioOnline && (currentMillis - lastRSSIUpdate > 2000)) {
        radioRSSI = radio.getRSSI();
        lastRSSIUpdate = currentMillis;
    }
    
    // Update display
    if (currentMillis - displayUpdateInterval > 500) {
        updateDisplay();
        displayUpdateInterval = currentMillis;
    }
    
    // Handle incoming radio packets
    if (transmittedFlag) {
        transmittedFlag = false;
        
        if (state == RADIOLIB_ERR_NONE && isRadioOnline) {
            uint8_t data[4];
            state = radio.readData(data, 4);
            
            if (state == RADIOLIB_ERR_NONE) {
                Serial.println(F("[Radio] Received packet!"));
                radioRSSI = radio.getRSSI();
                recvCounter++;
                
                tft.setTextColor(TFT_GREEN, TFT_BLACK);
                tft.drawString("Packet Received!", 10, 260);
            }
            
            // Restart receive mode
            radio.startReceive();
        }
    }
    
    delay(10); // Small delay to prevent busy-waiting
}