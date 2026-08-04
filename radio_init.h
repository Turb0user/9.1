#ifndef RADIO_INIT_H
#define RADIO_INIT_H

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <math.h>

#define PIN_NSS     PA4
#define PIN_SCK     PA5
#define PIN_MOSI    PA7
#define PIN_MISO    PA6
#define PIN_BUSY    PA1
#define PIN_DIO1    PB0
#define PIN_NRST    PA2
#define PIN_TXEN    PB10
#define PIN_RXEN    PA0
#define PIN_PTT     PA3
#define PIN_LED     PC13

// ==================== НАСТРОЙКИ ====================
#define RADIO_BITRATE_KBPS 9.6
#define RADIO_FREQ_MHZ      433.900
#define RADIO_POWER_DBM     22
#define RADIO_DEV_FACTOR    0.50f

// ==================== ФУНКЦИИ РАСЧЕТА ====================
inline float calcDeviation(float bitrate) {
    return bitrate * RADIO_DEV_FACTOR;
}

inline float calcBandwidth(float bitrate, float deviation) {
    return bitrate + 2.0f * deviation;
}

inline float nearestBandwidth(float bw) {
    const float table[] = {
        4.8, 5.8, 7.3, 9.7,
        11.7, 14.6, 19.5, 23.4,
        29.3, 39.0, 46.9, 58.6,
        78.2, 93.8, 117.3, 156.2,
        187.2, 234.3, 312.0, 373.6,
        467.0
    };
    float best = table[0];
    for (int i = 1; i < sizeof(table)/sizeof(table[0]); i++) {
        if (fabs(table[i] - bw) < fabs(best - bw))
            best = table[i];
    }
    return best;
}

// ==================== АВТОМАТИЧЕСКИЙ РАСЧЕТ ====================
#define RADIO_PREAMBLE_LENGTH 16  
#define RADIO_TX_WAIT_US(packetSize) ((((packetSize * 8) / RADIO_BITRATE_KBPS) * 1000) * 2)

extern LLCC68 radio;
extern bool radioReady;
extern float currentRssi;
extern float lastPacketRssi;
extern uint32_t rssiReadCount;
extern int packetSize;

// Внешняя функция обработки прерывания (определена в .ino)
extern void onDio1Interrupt(void);

inline bool waitBusy(uint32_t timeout) {
    uint32_t start = millis();
    while (digitalRead(PIN_BUSY)) {
        if (millis() - start > timeout) return false;
        delayMicroseconds(1);
    }
    return true;
}

inline float readRSSI_SPI() {
    uint8_t buf;
    if (!waitBusy(100)) return currentRssi;
    
    digitalWrite(PIN_NSS, LOW);
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    SPI.transfer(0x1D);
    buf = SPI.transfer(0x00);
    buf = SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(PIN_NSS, HIGH);
    
    float rssi = -((float)buf) / 2.0;
    if (rssi < -130 || rssi > -10) return currentRssi;
    
    rssiReadCount++;
    if (rssiReadCount < 5) {
        currentRssi = rssi;
    } else {
        currentRssi = currentRssi * 0.7 + rssi * 0.3;
    }
    return currentRssi;
}

inline void setTxenRxen(bool txen, bool rxen) {
    digitalWrite(PIN_TXEN, txen ? HIGH : LOW);
    digitalWrite(PIN_RXEN, rxen ? HIGH : LOW);
    delayMicroseconds(50);
}

inline void initRadioHardware() {
    float deviation = calcDeviation(RADIO_BITRATE_KBPS);
    float bandwidth = nearestBandwidth(calcBandwidth(RADIO_BITRATE_KBPS, deviation));
    
    Serial.println(F("\n=== GFSK RADIO INIT ==="));
    Serial.print(F("  Bitrate: ")); Serial.print(RADIO_BITRATE_KBPS); Serial.println(F(" kbps"));
    Serial.print(F("  Deviation: ")); Serial.print(deviation); Serial.println(F(" kHz"));
    Serial.print(F("  Bandwidth: ")); Serial.print(bandwidth); Serial.println(F(" kHz"));
    Serial.print(F("  Preamble: ")); Serial.println(RADIO_PREAMBLE_LENGTH);
    
    pinMode(PIN_TXEN, OUTPUT);
    pinMode(PIN_RXEN, OUTPUT);
    setTxenRxen(false, true);
    
    pinMode(PIN_NSS, OUTPUT);
    digitalWrite(PIN_NSS, HIGH);
    pinMode(PIN_BUSY, INPUT);
    pinMode(PIN_DIO1, INPUT);
    
    SPI.begin();
    int state = radio.beginFSK();
    
    if (state != RADIOLIB_ERR_NONE) {
        radioReady = false;
        Serial.println(F("Radio Initialization FAILED!"));
        return;
    }
    
    radio.calibrateImage(RADIO_FREQ_MHZ);
    radio.setDio2AsRfSwitch(true);
    radio.setRegulatorDCDC();
    radio.setCurrentLimit(140.0);
    radio.setFrequency(RADIO_FREQ_MHZ);
    radio.setBitRate(RADIO_BITRATE_KBPS);
    radio.setFrequencyDeviation(deviation);
    radio.setRxBandwidth(bandwidth);
    radio.setDataShaping(RADIOLIB_SHAPING_0_5);
    radio.setPreambleLength(RADIO_PREAMBLE_LENGTH);
    
    uint8_t syncWord[] = {0x12, 0xAD, 0x2B};
    radio.setSyncWord(syncWord, sizeof(syncWord));
    radio.setOutputPower(RADIO_POWER_DBM);
    radio.setCRC(2);
    
    // ============================================================
    // 🔧 КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ:
    // Устанавливаем обработчик прерывания ДО startReceive()!
    // ============================================================
    radio.setDio1Action(onDio1Interrupt);
    Serial.println(F("[RADIO] DIO1 Interrupt handler set!"));
    
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        radioReady = true;
        Serial.println(F("=== GFSK INIT SUCCESS ===\n"));
    } else {
        Serial.print(F("startReceive FAILED: "));
        Serial.println(state);
    }
    
    delay(100);
    currentRssi = readRSSI_SPI();
}

#endif
