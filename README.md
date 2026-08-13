#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <HardwareTimer.h>
#include "codec2.h"
#include "audio.h"
#include "radio_init.h"

// --- ПРОТОТИПЫ ---
void onDio1Interrupt(void); 
void rxPwmISR(void);
void handleVoiceTransmit(void);
void processIncomingPacketFromISR(void); 
void printDiagnostics(void);
void initHardwareAdcTimerDriven(void);

// --- ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ---
HardwareTimer *pwmTimer = NULL;
struct CODEC2 *c2 = NULL;
int samplesPerFrame = 0;
int bytesPerFrame = 0;
int packetSize = 0;

int16_t*  audioBuffer = NULL;    
uint8_t*  txPacketBuffer = NULL; 
uint8_t*  rxPacketBuffer = NULL; 

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

volatile uint16_t adcBuffer[ADC_BUFFER_SIZE];
volatile int adcIndex = 0;
volatile bool dmaReady = false;
volatile uint32_t dmaIrqCount = 0;

volatile bool txBufferReadyHalf = false;
volatile bool txBufferReadyFull = false;

volatile bool radioActionDone = false; 
volatile bool radioActionInProgress = false; 

Module* radioModule = new Module(PIN_NSS, PIN_DIO1, PIN_NRST, PIN_BUSY);
LLCC68 radio = radioModule; 

bool radioReady = false;
bool isTransmitting = false;

float currentRssi = -120.0;
float lastPacketRssi = -120.0;
uint32_t rssiReadCount = 0;

volatile uint32_t framesEncoded = 0;
volatile uint32_t packetsSent = 0;
volatile uint32_t adcReadCount = 0;
uint32_t lastDebugPrint = 0;

volatile uint32_t rxPacketsReceived = 0;
volatile uint32_t rxPacketsFailed = 0;
volatile uint32_t lastPacketTime = 0;
volatile uint32_t rxIntervalMax = 0;
volatile uint32_t rxIntervalMin = 99999;
volatile uint32_t bufferEmptyEvents = 0;

// ==================== RX БУФЕР ====================
#define RX_RING_SIZE 2560
#define RX_START_THRESHOLD 640

int16_t rxRingBuffer[RX_RING_SIZE];
volatile int rxWriteIdx = 0;
volatile int rxReadIdx = 0;
volatile int rxBufferedSamples = 0;
HardwareTimer *rxPwmTimer8k = NULL;

static int32_t lastLpfPwm = 50;
volatile bool rxAllowPlayback = false; 

// ==================== ВЕКТОР ПРЕРЫВАНИЯ DMA ====================
extern "C" void DMA2_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_adc1);
}

extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1 && isTransmitting) {
        txBufferReadyHalf = true;
        dmaIrqCount++;
    }
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        if (isTransmitting) {
            txBufferReadyFull = true;
        }
        dmaIrqCount++;
    }
}

// ==================== ПРЕРЫВАНИЕ DIO1 ====================
void onDio1Interrupt(void) {
    radioActionDone = true; 
}

// ==================== ШИМ ВЫВОД ====================
void rxPwmISR(void) {
    if (!isTransmitting) {
        if (rxBufferedSamples > 1280) {
            rxReadIdx = rxWriteIdx;
            rxBufferedSamples = 0;
            rxAllowPlayback = false;
        }

        if (!rxAllowPlayback && rxBufferedSamples >= RX_START_THRESHOLD) {
            rxAllowPlayback = true;
        }

        if (rxAllowPlayback && rxBufferedSamples > 0) {
            int32_t sample = rxRingBuffer[rxReadIdx];
            rxReadIdx = (rxReadIdx + 1) % RX_RING_SIZE;
            rxBufferedSamples--;

            int32_t pwm = ((sample + 32768) * 100) / 65535;
            if (pwm > 100) pwm = 100;
            if (pwm < 0)   pwm = 0;
            
            lastLpfPwm = pwm;
            audioOutWrite((uint8_t)pwm);
        } else {
            if (rxAllowPlayback) {
                bufferEmptyEvents++; 
            }
            rxAllowPlayback = false; 
            lastLpfPwm = (lastLpfPwm * 15 + 50) / 16; 
            audioOutWrite((uint8_t)lastLpfPwm);
        }
    }
}

// ==================== ИНИЦИАЛИЗАЦИЯ АЦП ====================
void initHardwareAdcTimerDriven() {
    Serial.println(F("\n[HAL ADC] Configuring Hardware 8kHz Timer Trigger..."));
    
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_RCC_ADC1_CLK_ENABLE();
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE; 
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; 
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;       
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_9; 
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, ADC_BUFFER_SIZE);

    HardwareTimer *adcTimer = new HardwareTimer(TIM3);
    adcTimer->setOverflow(8000, HERTZ_FORMAT);
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(adcTimer->getHandle(), &sMasterConfig);
    adcTimer->resume();
    
    Serial.println(F("[HAL ADC] 8kHz Hardware Sync OK."));
}

// ==================== ПЕРЕДАЧА ====================
void handleVoiceTransmit() {
    bool processFrame = false;
    uint16_t* sourcePtr = NULL;

    if (txBufferReadyHalf) {
        txBufferReadyHalf = false;
        sourcePtr = (uint16_t*)&adcBuffer; 
        processFrame = true;
    }
    else if (txBufferReadyFull) {
        txBufferReadyFull = false;
        sourcePtr = (uint16_t*)&adcBuffer[ADC_BUFFER_SIZE / 2];
        processFrame = true;
    }

    if (processFrame && sourcePtr != NULL) {
        static int32_t dcOffset = 1800; 
        static int txFrameCounter = 0;
        
        for (int i = 0; i < samplesPerFrame; i++) {
            uint16_t rawAdc = sourcePtr[i];
            
            dcOffset = (dcOffset * 15 + rawAdc) / 16; 

            int32_t sample32 = ((int32_t)rawAdc - dcOffset) * 12; 
            
            if (sample32 > 32767)  sample32 = 32767;
            if (sample32 < -32768) sample32 = -32768;
            
            audioBuffer[i] = (int16_t)sample32;
            adcReadCount++;
        }

        framesEncoded++;

        uint8_t* dest = txPacketBuffer + (txFrameCounter * bytesPerFrame);
        codec2_encode(c2, dest, audioBuffer);
        txFrameCounter++;

        if (txFrameCounter >= FRAMES_PER_PACKET) {
            txFrameCounter = 0;
            
            uint32_t waitStart = micros();
            // 🔧 ИЗМЕНЕНО: 1000 → RADIO_TX_WAIT_US(packetSize)
            while (radioActionInProgress && (micros() - waitStart < RADIO_TX_WAIT_US(packetSize))) {}
            
            if (!radioActionInProgress) {
                 radioActionInProgress = true;
                 radio.standby();
                 delayMicroseconds(100);
                 radio.finishTransmit(); 

                int state = radio.startTransmit(txPacketBuffer, packetSize);
                if (state == RADIOLIB_ERR_NONE) {
                    packetsSent++;
                } else {
                    radioActionInProgress = false;
                }
            }
        }
    }
}

// ==================== ПРИЕМ ====================
void processIncomingPacketFromISR() {
    uint32_t now = millis();
    uint32_t interval = now - lastPacketTime;
    lastPacketTime = now;
    
    if (rxPacketsReceived > 0) {
        if (interval > rxIntervalMax) rxIntervalMax = interval;
        if (interval < rxIntervalMin) rxIntervalMin = interval;
    }

    int16_t irqStatus = radio.getIrqFlags();
    int state = radio.readData(rxPacketBuffer, packetSize);

    if (state == RADIOLIB_ERR_NONE) {
        rxPacketsReceived++;
        lastPacketRssi = radio.getRSSI();
        
        for (int f = 0; f < FRAMES_PER_PACKET; f++) {
            uint8_t* framePtr = rxPacketBuffer + (f * bytesPerFrame);
            codec2_decode(c2, audioBuffer, framePtr);

            for (int i = 0; i < samplesPerFrame; i++) {
                if (rxBufferedSamples < RX_RING_SIZE) {
                    rxRingBuffer[rxWriteIdx] = audioBuffer[i];
                    rxWriteIdx = (rxWriteIdx + 1) % RX_RING_SIZE;
                    rxBufferedSamples++;
                }
            }
        }

        if (rxPacketsReceived % 10 == 0) { 
            Serial.print(F("📥 [RX] Packets:")); Serial.print(rxPacketsReceived);
            Serial.print(F(" | Interval:")); Serial.print(interval); Serial.print(F("ms"));
            Serial.print(F(" | RSSI:")); Serial.println(lastPacketRssi, 1);
        }

    } else {
        rxPacketsFailed++;
        Serial.print(F("❌ [RX_ERR] State: ")); Serial.print(state); 
        Serial.print(F(" | IrqFlags: 0x")); Serial.println(irqStatus, HEX);
    }
    
    setTxenRxen(false, true);
    radio.startReceive();
}

// ==================== ДИАГНОСТИКА ====================
void printDiagnostics() {
    Serial.println(F("\n=== EVENT-DRIVEN STREAM DIAGNOSTICS ==="));
    Serial.print(F("  Stream Frames    : ")); Serial.println(framesEncoded);
    Serial.print(F("  Packets Sent     : ")); Serial.println(packetsSent);
    Serial.print(F("  Processed ADC    : ")); Serial.println(adcReadCount);
    Serial.print(F("  DMA IRQ Count    : ")); Serial.println(dmaIrqCount);
    Serial.print(F("  Frame Size(B)    : ")); Serial.println(packetSize);
    Serial.println(F("--- RX STATS ---"));
    Serial.print(F("  Good RX Packets  : ")); Serial.println(rxPacketsReceived);
    Serial.print(F("  Bad RX Packets   : ")); Serial.println(rxPacketsFailed);
    Serial.print(F("  Buffer Starvations: ")); Serial.println(bufferEmptyEvents);
    Serial.println(F("=======================================\n"));
}

// ==================== SETUP ====================
void setup() {
    pinMode(PIN_LED, OUTPUT);
    for(int i = 0; i < 5; i++) {
        digitalWrite(PIN_LED, HIGH);
        delay(60);
        digitalWrite(PIN_LED, LOW);
        delay(60);
    }

    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n\n========================================"));
    Serial.println(F("=== SYSTEM START ==="));
    Serial.println(F("========================================\n"));

    pinMode(PIN_PTT, INPUT_PULLUP);
    digitalWrite(PIN_LED, HIGH);

    initAudioCodec();
    initHardwareAdcTimerDriven();
    initRadioHardware();

    if (radioReady) {
        radio.setDio1Action(onDio1Interrupt);
        Serial.println(F("[RADIO] DIO1 Interrupt Attached!"));
    }

    rxPwmTimer8k = new HardwareTimer(TIM2);
    rxPwmTimer8k->setOverflow(8000, HERTZ_FORMAT);
    rxPwmTimer8k->attachInterrupt(rxPwmISR);
    rxPwmTimer8k->resume();

    Serial.println(F("\n========================================"));
    Serial.println(F("=== SYSTEM READY ==="));
    Serial.println(F("========================================\n"));
}

// ==================== LOOP ====================
void loop() {
    bool pttPressed = (digitalRead(PIN_PTT) == LOW);

    if (pttPressed && !isTransmitting) {
        isTransmitting = true;
        digitalWrite(PIN_LED, LOW); 
        
        framesEncoded = 0;
        packetsSent = 0;
        adcReadCount = 0;
        dmaIrqCount = 0;
        
        txBufferReadyHalf = false;
        txBufferReadyFull = false;

        rxWriteIdx = 0;
        rxReadIdx = 0;
        rxBufferedSamples = 0;
        rxAllowPlayback = false;

        rxPacketsReceived = 0;
        rxPacketsFailed = 0;
        rxIntervalMax = 0;
        rxIntervalMin = 99999;
        bufferEmptyEvents = 0;

        radioActionDone = false; 
        radioActionInProgress = false; 

        Serial.println(F("🎤 [TX ON]"));
        setTxenRxen(true, false);
        delayMicroseconds(50);
    } 
    else if (!pttPressed && isTransmitting) {
        isTransmitting = false;
        digitalWrite(PIN_LED, HIGH); 
        
        lastLpfPwm = 50; 
        audioOutWrite(50); 
        rxAllowPlayback = false;

        radio.standby();
        radio.finishTransmit();
        
        radioActionDone = false;
        radioActionInProgress = false;

        setTxenRxen(false, true);
        radio.startReceive();

        Serial.println(F("🔇 [TX OFF]"));
        printDiagnostics();
    }

    if (radioActionDone) {
        radioActionDone = false; 
        
        if (isTransmitting) {
            radioActionInProgress = false; 
        } else {
            processIncomingPacketFromISR(); 
        }
    }

    if (isTransmitting) {
        handleVoiceTransmit(); 

        if (millis() - lastDebugPrint > 1000) {
            lastDebugPrint = millis();
            Serial.print(F("📡 Frames:")); Serial.print(framesEncoded);
            Serial.print(F(" | Pushes:")); Serial.print(packetsSent);
            Serial.print(F(" | DMA IRQs:")); Serial.print(dmaIrqCount);
            Serial.print(F(" | ADC: ["));
            for(int i = 0; i < 4; i++) {
                Serial.print(adcBuffer[i]);
                if(i < 3) Serial.print(F(", "));
            }
            Serial.println(F("]"));
        }
    } else {
        static uint32_t lastRssiRead = 0;
        if (millis() - lastRssiRead > 150) {
            lastRssiRead = millis();
            readRSSI_SPI();
        }
    }
}

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
#define RADIO_BITRATE_KBPS  38.4
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


#ifndef AUDIO_H
#define AUDIO_H

#include <Arduino.h>
#include <HardwareTimer.h>
#include "codec2.h"
#include "stm32f4xx_hal.h"

// ==================== ПИНЫ АУДИО ====================
#define MIC_PIN         PB1
#define AUDIO_OUT_PIN   PA8

// 🔧 ВЫБЕРИТЕ РЕЖИМ: 1300, 2400 или 3200
#define CODEC2_MODE CODEC2_MODE_1300

// ЖЕСТКАЯ ФИКСАЦИЯ: 2 кадра = 40 мс звука
#define FRAMES_PER_PACKET 2

// ==================== БУФЕРЫ ПЕРИФЕРИИ ====================
#define ADC_BUFFER_SIZE 640

// ==================== РАЗМЕРЫ КАДРОВ ДЛЯ CODEC2 ====================
// ВНИМАНИЕ! Это реальные размеры сжатых кадров в байтах
// Они НЕ равны bits_per_frame / 8 из-за битовой упаковки
#define CODEC2_BYTES_PER_FRAME_1300  7
#define CODEC2_BYTES_PER_FRAME_2400  12
#define CODEC2_BYTES_PER_FRAME_3200  16

inline int getCodec2BytesPerFrame(int mode) {
    switch(mode) {
        case CODEC2_MODE_1300: return CODEC2_BYTES_PER_FRAME_1300;
        case CODEC2_MODE_2400: return CODEC2_BYTES_PER_FRAME_2400;
        case CODEC2_MODE_3200: return CODEC2_BYTES_PER_FRAME_3200;
        default: return 0;
    }
}

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (extern) ====================
extern HardwareTimer *pwmTimer;
extern struct CODEC2 *c2;
extern int samplesPerFrame;
extern int bytesPerFrame;
extern int packetSize;           

extern int16_t*  audioBuffer;    
extern uint8_t*  txPacketBuffer; 
extern uint8_t*  rxPacketBuffer; 

extern volatile uint16_t adcBuffer[ADC_BUFFER_SIZE];
extern volatile int adcIndex;
extern volatile bool dmaReady;
extern volatile uint32_t dmaIrqCount;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

// ==================== PWM ВЫВОД ЗВУКА ====================
inline void initAudioOut() {
    Serial.println(F("    [PWM] Start..."));
    pinMode(AUDIO_OUT_PIN, OUTPUT);
    pwmTimer = new HardwareTimer(TIM1);
    pwmTimer->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, AUDIO_OUT_PIN);
    pwmTimer->setOverflow(200000, HERTZ_FORMAT);
    pwmTimer->setCaptureCompare(1, 0, PERCENT_COMPARE_FORMAT);
    pwmTimer->resume();
    Serial.println(F("    [PWM] OK"));
}

inline void audioOutWrite(uint8_t pwmValue) {
    if (pwmTimer == NULL) return;
    if (pwmValue > 100) pwmValue = 100;
    pwmTimer->setCaptureCompare(1, pwmValue, PERCENT_COMPARE_FORMAT);
}

// ==================== ИНИЦИАЛИЗАЦИЯ КОДЕКА ====================
inline void initAudioCodec() {
    Serial.println(F("\n[initAudioCodec] START"));
    
    Serial.print(F("  Codec2 create..."));
    c2 = codec2_create(CODEC2_MODE);
    if (!c2) {
        Serial.println(F(" FAIL"));
        return;
    }
    Serial.println(F(" OK"));
    
    samplesPerFrame = codec2_samples_per_frame(c2);
    
    // 🔧 ИСПРАВЛЕНО: используем таблицу размеров, а не bits_per_frame
    bytesPerFrame = getCodec2BytesPerFrame(CODEC2_MODE);
    packetSize = bytesPerFrame * FRAMES_PER_PACKET;

    Serial.print(F("    samplesPerFrame: ")); Serial.println(samplesPerFrame);
    Serial.print(F("    bytesPerFrame: ")); Serial.println(bytesPerFrame);
    Serial.print(F("    packetSize: ")); Serial.println(packetSize);

    Serial.print(F("  malloc buffers..."));
    audioBuffer = (int16_t*)malloc(samplesPerFrame * sizeof(int16_t));
    txPacketBuffer = (uint8_t*)malloc(packetSize);
    rxPacketBuffer = (uint8_t*)malloc(packetSize);
    if (audioBuffer && txPacketBuffer && rxPacketBuffer) {
        Serial.println(F(" OK"));
    } else {
        Serial.println(F(" FAIL"));
        return;
    }
    
    Serial.print(F("  Init PWM..."));
    initAudioOut();
    
    Serial.println(F("[initAudioCodec] END"));
}

#endif
