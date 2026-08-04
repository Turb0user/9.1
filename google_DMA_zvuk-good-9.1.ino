#include <Arduino.h>#include <SPI.h>#include <RadioLib.h>#include <HardwareTimer.h>#include <string.h>  // для memcpy#include "codec2.h"#include "audio.h"#include "radio_init.h"

// --- ПРОТОТИПЫ ---void onDio1Interrupt(void);void rxPwmISR(void);void handleVoiceTransmit(void);void processIncomingPacketFromISR(void);void printDiagnostics(void);void initHardwareAdcTimerDriven(void);

// --- ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ---HardwareTimer *pwmTimer = NULL;struct CODEC2 *c2 = NULL;int samplesPerFrame = 0;int bytesPerFrame = 0;int packetSize = 0;

int16_t*  audioBuffer = NULL;uint8_t*  txPacketBuffer = NULL;uint8_t*  rxPacketBuffer = NULL;

// Буферы для безопасной работы с прерываниямиuint8_t tempPacketBuffer[64];volatile bool packetReady = false;volatile bool needStartReceive = false;volatile bool rxErrorFlag = false;volatile int lastRxErrorState = 0;volatile uint32_t rxReadTimeout = 0;

ADC_HandleTypeDef hadc1;DMA_HandleTypeDef hdma_adc1;

volatile uint16_t adcBuffer[ADC_BUFFER_SIZE];volatile int adcIndex = 0;volatile bool dmaReady = false;volatile uint32_t dmaIrqCount = 0;

volatile bool txBufferReadyHalf = false;volatile bool txBufferReadyFull = false;

volatile bool radioActionDone = false;volatile bool radioActionInProgress = false;

Module* radioModule = new Module(PIN_NSS, PIN_DIO1, PIN_NRST, PIN_BUSY);LLCC68 radio = radioModule;

bool radioReady = false;bool isTransmitting = false;

float currentRssi = -120.0;float lastPacketRssi = -120.0;uint32_t rssiReadCount = 0;

volatile uint32_t framesEncoded = 0;volatile uint32_t packetsSent = 0;volatile uint32_t adcReadCount = 0;uint32_t lastDebugPrint = 0;

volatile uint32_t rxPacketsReceived = 0;volatile uint32_t rxPacketsFailed = 0;volatile uint32_t lastPacketTime = 0;volatile uint32_t rxIntervalMax = 0;volatile uint32_t rxIntervalMin = 99999;volatile uint32_t bufferEmptyEvents = 0;

// ==================== RX БУФЕР ====================#define RX_RING_SIZE 3840#define RX_START_THRESHOLD 640#define RX_RESET_THRESHOLD (RX_RING_SIZE / 2)

int16_t rxRingBuffer[RX_RING_SIZE];volatile int rxWriteIdx = 0;volatile int rxReadIdx = 0;volatile int rxBufferedSamples = 0;HardwareTimer *rxPwmTimer8k = NULL;

static int32_t lastLpfPwm = 50;volatile bool rxAllowPlayback = false;

// ==================== ВЕКТОР ПРЕРЫВАНИЯ DMA ====================extern "C" void DMA2_Stream0_IRQHandler(void) {HAL_DMA_IRQHandler(&hdma_adc1);}

extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {if (hadc->Instance == ADC1 && isTransmitting) {txBufferReadyHalf = true;dmaIrqCount++;}}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {if (hadc->Instance == ADC1) {if (isTransmitting) {txBufferReadyFull = true;}dmaIrqCount++;}}

// ==================== ПРЕРЫВАНИЕ DIO1 ====================void onDio1Interrupt(void) {static uint32_t counter = 0;counter++;if (counter % 10 == 0) {Serial.print(F("⚡ DIO1 IRQ: "));Serial.println(counter);}radioActionDone = true;// Сбрасываем флаг при завершении приемаif (isTransmitting) {radioActionInProgress = false;}}

// ==================== ШИМ ВЫВОД ====================void rxPwmISR(void) {if (!isTransmitting) {if (rxBufferedSamples > RX_RESET_THRESHOLD) {rxReadIdx = rxWriteIdx;rxBufferedSamples = 0;rxAllowPlayback = false;}

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

// ==================== ИНИЦИАЛИЗАЦИЯ АЦП ====================void initHardwareAdcTimerDriven() {Serial.println(F("\n[HAL ADC] Configuring Hardware 8kHz Timer Trigger..."));

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

// ==================== ПЕРЕДАЧА ====================void handleVoiceTransmit() {bool processFrame = false;uint16_t* sourcePtr = NULL;

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
        
        if (!radioActionInProgress) {
            radioActionInProgress = true;
            radio.standby();
            radio.finishTransmit(); 

            int state = radio.startTransmit(txPacketBuffer, packetSize);
            if (state == RADIOLIB_ERR_NONE) {
                packetsSent++;
            }
            // 🔧 ПРИНУДИТЕЛЬНЫЙ СБРОС ФЛАГА — БЕЗ ТАЙМЕРОВ И ЗАДЕРЖЕК!
            radioActionInProgress = false;
        }
    }
}

}

// ==================== БЫСТРАЯ ЧАСТЬ (ПРЕРЫВАНИЕ) ====================void processIncomingPacketFromISR() {// Если мы передаем — игнорируем приемif (isTransmitting) {return;}

// 🔧 ЗАЩИТА ОТ НЕПРАВИЛЬНОГО ПАКЕТА: проверяем размер
if (packetSize == 0 || packetSize > 64) {
    rxErrorFlag = true;
    lastRxErrorState = -999;
    needStartReceive = true;
    return;
}

int state = radio.readData(tempPacketBuffer, packetSize);

if (state == RADIOLIB_ERR_NONE) {
    lastPacketRssi = radio.getRSSI();
    memcpy((void*)rxPacketBuffer, (void*)tempPacketBuffer, packetSize);
    packetReady = true;
    needStartReceive = true;
} else {
    rxPacketsFailed++;
    rxErrorFlag = true;
    lastRxErrorState = state;
    needStartReceive = false;
    
    // При ошибке — сбрасываем чип в безопасное состояние
    radio.standby();
    delayMicroseconds(100);
    needStartReceive = true;
}

setTxenRxen(false, true);

}

// ==================== ДИАГНОСТИКА ====================void printDiagnostics() {Serial.println(F("\n=== EVENT-DRIVEN STREAM DIAGNOSTICS ==="));Serial.print(F("  packetSize: ")); Serial.println(packetSize);Serial.print(F("  bytesPerFrame: ")); Serial.println(bytesPerFrame);Serial.print(F("  FRAMES_PER_PACKET: ")); Serial.println(FRAMES_PER_PACKET);Serial.print(F("  Stream Frames    : ")); Serial.println(framesEncoded);Serial.print(F("  Packets Sent     : ")); Serial.println(packetsSent);Serial.print(F("  Processed ADC    : ")); Serial.println(adcReadCount);Serial.print(F("  DMA IRQ Count    : ")); Serial.println(dmaIrqCount);Serial.println(F("--- RX STATS ---"));Serial.print(F("  Good RX Packets  : ")); Serial.println(rxPacketsReceived);Serial.print(F("  Bad RX Packets   : ")); Serial.println(rxPacketsFailed);Serial.print(F("  Buffer Starvations: ")); Serial.println(bufferEmptyEvents);Serial.println(F("=======================================\n"));}

// ==================== SETUP ====================void setup() {pinMode(PIN_LED, OUTPUT);for(int i = 0; i < 5; i++) {digitalWrite(PIN_LED, HIGH);delay(60);digitalWrite(PIN_LED, LOW);delay(60);}

Serial.begin(115200);
delay(500);

Serial.println(F("\n\n========================================"));
Serial.println(F("=== SYSTEM START ==="));
Serial.println(F("========================================\n"));

pinMode(PIN_PTT, INPUT_PULLUP);
digitalWrite(PIN_LED, HIGH);

initAudioCodec();

Serial.print(F("📦 packetSize: ")); Serial.println(packetSize);
Serial.print(F("📦 bytesPerFrame: ")); Serial.println(bytesPerFrame);
Serial.print(F("📦 FRAMES_PER_PACKET: ")); Serial.println(FRAMES_PER_PACKET);

initHardwareAdcTimerDriven();
initRadioHardware();

if (radioReady) {
    Serial.println(F("[RADIO] Radio ready!"));
}

rxPwmTimer8k = new HardwareTimer(TIM2);
rxPwmTimer8k->setOverflow(8000, HERTZ_FORMAT);
rxPwmTimer8k->attachInterrupt(rxPwmISR);
rxPwmTimer8k->resume();

Serial.println(F("\n========================================"));
Serial.println(F("=== SYSTEM READY ==="));
Serial.println(F("========================================\n"));

}

// ==================== LOOP ====================void loop() {bool pttPressed = (digitalRead(PIN_PTT) == LOW);

// --- ОБРАБОТКА PTT ---
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
    packetReady = false;
    needStartReceive = false;
    rxErrorFlag = false;

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
    packetReady = false;
    needStartReceive = false;

    setTxenRxen(false, true);
    radio.startReceive();

    Serial.println(F("🔇 [TX OFF]"));
    printDiagnostics();
}

// --- ОБРАБОТКА ПРЕРЫВАНИЯ DIO1 ---
if (radioActionDone) {
    radioActionDone = false; 
    processIncomingPacketFromISR();
}

// --- ПЕРЕЗАПУСК ПРИЕМА ---
if (needStartReceive && !isTransmitting) {
    needStartReceive = false;
    
    uint32_t start = millis();
    while (digitalRead(PIN_BUSY) && (millis() - start < 50)) {
        delayMicroseconds(100);
    }
    
    if (!digitalRead(PIN_BUSY)) {
        int state = radio.startReceive();
        if (state != RADIOLIB_ERR_NONE) {
            needStartReceive = true;
        }
    } else {
        radio.standby();
        delayMicroseconds(100);
        needStartReceive = true;
    }
}

// --- ОБРАБОТКА ПАКЕТА ---
if (packetReady && !isTransmitting) {
    packetReady = false;
    
    uint32_t now = millis();
    uint32_t interval = now - lastPacketTime;
    lastPacketTime = now;
    
    if (rxPacketsReceived > 0) {
        if (interval > rxIntervalMax) rxIntervalMax = interval;
        if (interval < rxIntervalMin) rxIntervalMin = interval;
    }
    
    rxPacketsReceived++;
    
    __disable_irq();
    for (int f = 0; f < FRAMES_PER_PACKET; f++) {
        uint8_t* framePtr = rxPacketBuffer + (f * bytesPerFrame);
        codec2_decode(c2, audioBuffer, framePtr);
        
        for (int i = 0; i < samplesPerFrame; i++) {
            if (rxBufferedSamples < RX_RING_SIZE) {
                rxRingBuffer[rxWriteIdx] = audioBuffer[i];
                rxWriteIdx = (rxWriteIdx + 1) % RX_RING_SIZE;
                rxBufferedSamples++;
            } else {
                rxRingBuffer[rxWriteIdx] = audioBuffer[i];
                rxWriteIdx = (rxWriteIdx + 1) % RX_RING_SIZE;
                rxReadIdx = (rxReadIdx + 1) % RX_RING_SIZE;
            }
        }
    }
    __enable_irq();
    
    if (rxPacketsReceived % 10 == 0) {
        Serial.print(F("📥 [RX] Packets:"));
        Serial.print(rxPacketsReceived);
        Serial.print(F(" | Interval:"));
        Serial.print(interval);
        Serial.print(F("ms | RSSI:"));
        Serial.println(lastPacketRssi, 1);
    }
}

// --- ОБРАБОТКА ОШИБОК ---
if (rxErrorFlag && !isTransmitting) {
    rxErrorFlag = false;
    Serial.print(F("❌ [RX_ERR] State: "));
    Serial.println(lastRxErrorState);
}

// --- ПЕРЕДАЧА ---
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
} 
// --- RSSI ---
else {
    static uint32_t lastRssiRead = 0;
    if (millis() - lastRssiRead > 150) {
        lastRssiRead = millis();
        readRSSI_SPI();
    }
}

}
