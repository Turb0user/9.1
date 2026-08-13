Да. В этом варианте причина зависания на приёме, скорее всего, не радио. У тебя уже был рабочий приём на тех же пинах и с теми же параметрами FSK. После добавления Codec2 + аудиотаймера появляется участок, который может надолго блокировать loop().

Главное подозрение — codec2_decode() выполняется прямо в обработчике приёма, пока TIM3 продолжает работать на 8 кГц, плюс одновременно меняется FIFO. Для диагностики сначала надо разделить радио и Codec2.

1. Сначала замени весь блок RX на этот
if (radioActionFlag) {
    radioActionFlag = false;

    if (!isTransmittingMode) {

        Serial.println(F("[RX-1] IRQ"));

        uint8_t rawBuffer[PACKET_LENGTH];

        Serial.println(F("[RX-2] readData..."));

        int state = radio.readData(rawBuffer, PACKET_LENGTH);

        Serial.print(F("[RX-3] readData state="));
        Serial.println(state);

        if (state == RADIOLIB_ERR_NONE) {

            Serial.println(F("[RX-4] PACKET OK"));

            Serial.print(F("[RX-DATA] "));
            for (int i = 0; i < PACKET_LENGTH; i++) {
                if (rawBuffer[i] < 0x10) Serial.print('0');
                Serial.print(rawBuffer[i], HEX);
                Serial.print(' ');
            }
            Serial.println();

            Serial.println(F("[RX-5] before codec"));

            int16_t pcm_out_buffer[CODEC2_SAMPLES_PER_FRM];

            for (int f = 0; f < 2; f++) {

                Serial.print(F("[RX-DEC] frame "));
                Serial.println(f);

                codec2_decode(
                    c2,
                    pcm_out_buffer,
                    rawBuffer + f * CODEC2_FRAME_BYTES
                );

                Serial.println(F("[RX-DEC] decoded"));

                for (int i = 0; i < CODEC2_SAMPLES_PER_FRM; i++) {
                    audioOut_put(pcm_out_buffer[i]);
                }

                Serial.println(F("[RX-DEC] FIFO done"));
            }

            Serial.println(F("[RX-6] codec complete"));
        }

        Serial.println(F("[RX-7] restart RX"));

        StartRX_Async();

        Serial.println(F("[RX-8] RX restarted"));
    }
    else {
        txReadyForNext = true;
    }
}

Это сразу покажет, где именно зависает.

2. Но я вижу ещё одну серьёзную проблему

У тебя сейчас:

MyTim = new HardwareTimer(TIM3);
MyTim->setOverflow(8000, HERTZ_FORMAT);
MyTim->attachInterrupt(Timer_8kHz_ISR);
MyTim->setInterruptPriority(4, 0);
MyTim->resume();

И одновременно:

codec2_decode(...)

Codec2 выполняет довольно тяжёлую обработку. А каждые 125 мкс его прерывает:

Timer_8kHz_ISR()

В ISR выполняется:

audioOut_get(pcm_out);

и запись:

TIM1->CCR1 = pwm_val;

Само по себе это нормально, но для диагностики Codec2 обязательно отключим TIM3 на время декодирования.

Сделай так:

Перед декодированием:

MyTim->pause();

После декодирования:

MyTim->resume();

То есть:

if (state == RADIOLIB_ERR_NONE) {

    Serial.println(F("[RX-4] PACKET OK"));

    int16_t pcm_out_buffer[CODEC2_SAMPLES_PER_FRM];

    // Останавливаем аудио ISR
    MyTim->pause();

    Serial.println(F("[RX-5] codec start"));

    for (int f = 0; f < 2; f++) {

        Serial.print(F("[RX-DEC] frame "));
        Serial.println(f);

        codec2_decode(
            c2,
            pcm_out_buffer,
            rawBuffer + f * CODEC2_FRAME_BYTES
        );

        Serial.println(F("[RX-DEC] decoded"));

        for (int i = 0; i < CODEC2_SAMPLES_PER_FRM; i++) {
            audioOut_put(pcm_out_buffer[i]);
        }

        Serial.println(F("[RX-DEC] FIFO done"));
    }

    Serial.println(F("[RX-6] codec complete"));

    // Возвращаем аудиотаймер
    MyTim->resume();
}

Но есть важный нюанс: если остановить TIM3, FIFO перестаёт выдаваться в PWM. После resume() он начнёт воспроизводиться.

3. Ещё лучше — вообще не декодировать два кадра одним вызовом RX

Сейчас у тебя:

IRQ
 ↓
readData
 ↓
codec2_decode frame 0
 ↓
codec2_decode frame 1
 ↓
640 samples → FIFO
 ↓
startReceive

Это потенциально длинный блок.

Я бы сделал очередь:

RADIO IRQ
   ↓
readData
   ↓
копируем 14 байт
   ↓
сразу startReceive()
   ↓
loop()
   ↓
Codec2 decode
   ↓
FIFO

То есть радио должно как можно быстрее вернуться в RX.

4. Самое важное исправление

Добавь отдельный буфер пакета:

volatile bool codecPacketReady = false;
uint8_t codecPacket[PACKET_LENGTH];

И RX:

if (radioActionFlag) {
    radioActionFlag = false;

    if (!isTransmittingMode) {

        Serial.println(F("[RX-1] IRQ"));

        int state = radio.readData(codecPacket, PACKET_LENGTH);

        Serial.print(F("[RX-2] state="));
        Serial.println(state);

        if (state == RADIOLIB_ERR_NONE) {
            stat_rx_packet_cnt++;
            codecPacketReady = true;

            Serial.println(F("[RX-3] packet copied"));
        }

        Serial.println(F("[RX-4] restart"));

        StartRX_Async();

        Serial.println(F("[RX-5] RX ready"));
    }
    else {
        txReadyForNext = true;
    }
}

А Codec2 вынести ниже, отдельно:

if (codecPacketReady && !isTransmittingMode) {

    codecPacketReady = false;

    Serial.println(F("[CODEC] START"));

    int16_t pcm_out_buffer[CODEC2_SAMPLES_PER_FRM];

    MyTim->pause();

    for (int f = 0; f < 2; f++) {

        Serial.print(F("[CODEC] frame="));
        Serial.println(f);

        codec2_decode(
            c2,
            pcm_out_buffer,
            codecPacket + f * CODEC2_FRAME_BYTES
        );

        Serial.println(F("[CODEC] decoded"));

        for (int i = 0; i < CODEC2_SAMPLES_PER_FRM; i++) {
            audioOut_put(pcm_out_buffer[i]);
        }
    }

    MyTim->resume();

    Serial.println(F("[CODEC] DONE"));
}

Это уже гораздо правильнее.

5. Но есть ещё одна проблема с твоим FIFO

Ты используешь:

inline void audioOut_put(int16_t sample)

а ISR одновременно делает:

audioOut_get(...)

На STM32F401 запись 32-битного head/tail атомарна, поэтому обычно это работает. Но во время массовой загрузки 640 сэмплов лучше временно запретить IRQ:

for (int i = 0; i < CODEC2_SAMPLES_PER_FRM; i++) {
    audioOut_put(pcm_out_buffer[i]);
}

не обязательно.

Не надо делать __disable_irq() вокруг всего codec2_decode() — это уже может сломать радио.

6. Очень важный тест

Перед Codec2 вообще сделай такой RX:

if (state == RADIOLIB_ERR_NONE) {

    Serial.println(F("[RX] PACKET OK"));

    for (int i = 0; i < PACKET_LENGTH; i++) {
        Serial.print(rawBuffer[i], HEX);
        Serial.print(' ');
    }

    Serial.println();
}

Без:

codec2_decode()

и без:

audioOut_put()

Если при этом:

[RX-1] IRQ
[RX-2] state=0
[RX-3] packet copied
[RX-4] restart
[RX-5] RX ready

идёт стабильно — радио полностью исправно.

Тогда проблема точно находится в связке:

Codec2 + TIM3 + PWM/FIFO
7. И ещё: у тебя сейчас неправильная логика PTT-фильтра

Вот это:

if (currentRawPtt != stablePttState) {
    if ((millis() - lastPttChangeTime) > 50) {
        stablePttState = currentRawPtt;
    }
} else {
    lastPttChangeTime = millis();
}

не является нормальным debounce.

Лучше:

static bool lastRawPtt = false;

bool rawPtt = (digitalRead(PIN_PTT) == LOW);

if (rawPtt != lastRawPtt) {
    lastPttChangeTime = millis();
    lastRawPtt = rawPtt;
}

if ((millis() - lastPttChangeTime) >= 50) {
    stablePttState = rawPtt;
}

Иначе при ВЧ/наводках у тебя реально могут происходить быстрые:

TX
RX
TX
RX
TX
RX

что видно в твоих предыдущих логах.

Что я бы сделал сейчас

Не менять радио вообще. Оно у тебя уже доказано рабочим.

Сначала поставь архитектуру:

                    ┌──────────────┐
DIO1 ──IRQ─────────►│ readData()   │
                    │ 14 bytes     │
                    └──────┬───────┘
                           │
                           ▼
                    codecPacket[14]
                           │
                    startReceive()
                           │
                           ▼
                     радио снова RX

loop()
  │
  └── codecPacketReady
          │
          ▼
     codec2_decode()
          │
          ▼
      640 samples
          │
          ▼
       Audio FIFO
          │
          ▼
       TIM3 8 kHz
          │
          ▼
       TIM1 PWM PA8

Главное: readData() должен завершиться → startReceive() → только потом заниматься Codec2.

И первым делом выведи лог именно с [RX-1] ... [RX-8]. По последней напечатанной строке я смогу точно сказать, зависает ли у тебя readData(), codec2_decode(), StartRX_Async() или аудиотаймер.
