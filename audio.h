#ifndef AUDIO_H
#define AUDIO_H

#include <Arduino.h>
#include <HardwareTimer.h>
#include "codec2.h"
#include "stm32f4xx_hal.h"

// ==================== ПИНЫ АУДИО ====================
#define MIC_PIN         PB1
#define AUDIO_OUT_PIN   PA8

// ==================== ★★★ ВЫБЕРИТЕ РЕЖИМ ★★★ ====================
// Раскомментируйте нужный режим:
#define CODEC2_MODE CODEC2_MODE_1300
//#define CODEC2_MODE CODEC2_MODE_700
//#define CODEC2_MODE CODEC2_MODE_2400
//#define CODEC2_MODE CODEC2_MODE_3200
// ================================================================

// ==================== ПАРАМЕТРЫ ДЛЯ ВСЕХ РЕЖИМОВ ====================
#if CODEC2_MODE == CODEC2_MODE_700
    #define SAMPLES_PER_FRAME   320
    #define BYTES_PER_FRAME     14
    #define FRAMES_PER_PACKET   1
    #define ADC_BUFFER_SIZE     1280

#elif CODEC2_MODE == CODEC2_MODE_1300
    #define SAMPLES_PER_FRAME   160   // ← ИСПРАВЛЕНО: 160, а не 320!
    #define BYTES_PER_FRAME     7
    #define FRAMES_PER_PACKET   2
    #define ADC_BUFFER_SIZE     640

#elif CODEC2_MODE == CODEC2_MODE_2400
    #define SAMPLES_PER_FRAME   160
    #define BYTES_PER_FRAME     12
    #define FRAMES_PER_PACKET   2
    #define ADC_BUFFER_SIZE     640

#elif CODEC2_MODE == CODEC2_MODE_3200
    #define SAMPLES_PER_FRAME   160
    #define BYTES_PER_FRAME     16
    #define FRAMES_PER_PACKET   2
    #define ADC_BUFFER_SIZE     640

#else
    // Если выбран неподдерживаемый режим — ошибка компиляции
    #error "Unsupported CODEC2_MODE! Check your selection."
#endif

// ==================== РАЗМЕР ПАКЕТА ====================
#define PACKET_SIZE (BYTES_PER_FRAME * FRAMES_PER_PACKET)

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
    
    Serial.print(F("  Codec2 create (mode "));
    Serial.print(CODEC2_MODE);
    Serial.print(F(")..."));
    
    c2 = codec2_create(CODEC2_MODE);
    if (!c2) {
        Serial.println(F(" FAIL"));
        return;
    }
    Serial.println(F(" OK"));
    
    // Используем константы из макросов
    samplesPerFrame = SAMPLES_PER_FRAME;
    bytesPerFrame = BYTES_PER_FRAME;
    packetSize = PACKET_SIZE;

    Serial.println(F("\n=== CODEC2 CONFIG ==="));
    Serial.print(F("  Mode: ")); Serial.println(CODEC2_MODE);
    Serial.print(F("  samplesPerFrame: ")); Serial.println(samplesPerFrame);
    Serial.print(F("  bytesPerFrame: ")); Serial.println(bytesPerFrame);
    Serial.print(F("  FRAMES_PER_PACKET: ")); Serial.println(FRAMES_PER_PACKET);
    Serial.print(F("  packetSize: ")); Serial.println(packetSize);
    Serial.print(F("  ADC_BUFFER_SIZE: ")); Serial.println(ADC_BUFFER_SIZE);
    Serial.println(F("====================\n"));

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
