Да. Здесь уже видно несколько конкретных проблем. Радио само по себе, судя по предыдущему тесту, работает. Зависание появляется именно после IRQ приёма, на цепочке readData() → codec2_decode() → FIFO → StartRX().

Самая подозрительная часть — сейчас у тебя одновременно работают:

TIM3 ISR 8000 Гц;
DIO1 IRQ радио;
codec2_decode() из loop();
Serial.print() из loop();
прямое управление ADC;
прямое управление TIM1.

И есть одна особенно неприятная ошибка в обработке радио.

1. Сначала убираем возможное зависание внутри readData()

Сейчас:

if (radioActionFlag) {
    radioActionFlag = false;

    if (!isTransmittingMode) {
        uint8_t rawBuffer[PACKET_LENGTH];
        int state = radio.readData(rawBuffer, PACKET_LENGTH);

        if (state == RADIOLIB_ERR_NONE) {
            ...
        }

        StartRX_Async();
    }
}

Сделай диагностику до и после каждого этапа:

if (radioActionFlag) {

    radioActionFlag = false;

    if (!isTransmittingMode) {

        Serial.print("[IRQ]");

        uint8_t rawBuffer[PACKET_LENGTH];

        Serial.print("[RD]");
        int state = radio.readData(rawBuffer, PACKET_LENGTH);
        Serial.print("[RD=");
        Serial.print(state);
        Serial.print("]");

        if (state == RADIOLIB_ERR_NONE) {

            stat_rx_packet_cnt++;

            Serial.print("[OK]");

            if (c2 != NULL) {

                int16_t pcm_out_buffer[CODEC2_SAMPLES_PER_FRM];

                Serial.print("[DEC]");

                for (int f = 0; f < 2; f++) {

                    codec2_decode(
                        c2,
                        pcm_out_buffer,
                        rawBuffer + f * CODEC2_FRAME_BYTES
                    );

                    Serial.print(".");

                    for (int i = 0; i < CODEC2_SAMPLES_PER_FRM; i++) {
                        audioOut_put(pcm_out_buffer[i]);
                    }
                }

                Serial.print("[DECOK]");
            }
        }

        Serial.print("[RX]");

        StartRX_Async();

        Serial.print("[RXOK]");
    }
    else {
        txReadyForNext = true;
    }
}

Особенно важно, где остановится вывод.

Например:

[IRQ][RD]

значит проблема в radio.readData().

[IRQ][RD=0][OK][DEC].

значит зависание происходит внутри первого codec2_decode().

[IRQ][RD=0][OK][DEC]..[DECOK][RX]

значит проблема в StartRX_Async().

2. Но я вижу более серьёзную проблему с таймером

У тебя ISR:

void Timer_8kHz_ISR(void) {

    int16_t pcm_out = 0;

    if (audioOut_get(pcm_out)) {
        ...
        TIM1->CCR1 = pwm_val;
    } else {
        TIM1->CCR1 = 128;
        stat_underrun_flag = true;
    }

    if (isTransmittingMode) {
        ...
        audioIn_put(tone_pcm);
        sineIdx++;
        ADC1->CR2 |= ADC_CR2_SWSTART;
    }
}

То есть во время приёма ADC вообще не нужен, но PWM ISR всё равно работает постоянно.

Это само по себе допустимо, но сейчас ISR обращается к FIFO в то время, когда основной поток выполняет:

codec2_decode()

и:

audioOut_put()

На STM32 это может работать, но я бы сейчас максимально упростил систему.

3. На время диагностики полностью отключи ADC

Вот это:

ADC1->CR2 |= ADC_CR2_SWSTART;

вообще убери из ISR.

И пока сделай ISR только таким:

void Timer_8kHz_ISR(void) {

    int16_t pcm_out;

    if (audioOut_get(pcm_out)) {

        int32_t pwm_val =
            ((int32_t)pcm_out + 32768) >> 8;

        if (pwm_val < 0)
            pwm_val = 0;

        if (pwm_val > 255)
            pwm_val = 255;

        TIM1->CCR1 = pwm_val;

    } else {

        TIM1->CCR1 = 128;
        stat_underrun_flag = true;
    }
}

Во время RX ADC нам вообще не нужен.

4. Ещё одна проблема — audioOut_put() вызывается из основного потока

Сейчас:

inline void audioOut_put(int16_t sample) {

    uint32_t next_tail =
        (audioOutFIFO.tail + 1) & AUDIO_FIFO_MASK;

    if (next_tail == audioOutFIFO.head) {
        audioOutFIFO.head =
            (audioOutFIFO.head + 1) & AUDIO_FIFO_MASK;
    }

    audioOutFIFO.buffer[audioOutFIFO.tail] = sample;
    audioOutFIFO.tail = next_tail;
}

А audioOut_get() одновременно вызывается из ISR.

Это SPSC FIFO, и в целом схема правильная, но во время диагностики я бы сделал ещё проще: декодируем кадр во временный буфер, затем одним критическим участком переносим данные.

Например:

for (int f = 0; f < 2; f++) {

    codec2_decode(
        c2,
        pcm_out_buffer,
        rawBuffer + f * CODEC2_FRAME_BYTES
    );

    noInterrupts();

    for (int i = 0; i < CODEC2_SAMPLES_PER_FRM; i++) {
        audioOut_put(pcm_out_buffer[i]);
    }

    interrupts();
}

Это не идеально с точки зрения realtime, но для диагностики позволит исключить гонку FIFO.

5. Очень важный момент: твой AUDIO_FIFO_SIZE = 2048

Для Codec2:

320 samples / frame
2 frames / packet
640 samples / packet

При 8000 Гц:

640 / 8000 = 80 ms

То есть один радиопакет даёт 80 мс звука.

А FIFO:

2048 / 8000 = 256 мс

То есть примерно 256 мс буфера.

Это нормально.

6. Но у тебя codec2_decode() может выполняться дольше, чем кажется

И при этом TIM3 продолжает генерировать:

8000 interrupts/sec

Причём каждое прерывание вызывает:

audioOut_get()

и ещё обращается к:

TIM1->CCR1

Если приоритет:

MyTim->setInterruptPriority(4, 0);

то это хорошо — он ниже радио.

Но я бы вообще на время теста сделал:

MyTim->setInterruptPriority(10, 0);

или оставил стандартный низкий приоритет.

7. Ещё одна очень важная вещь: не печатай диагностику слишком часто

Вот это:

Serial.print(F("[I][R:OK]"));

на каждом пакете при скорости 38.4 кбит/с — плохая идея.

У тебя пакет 14 байт.

Приблизительно один пакет может приходить каждые десятки миллисекунд, а Serial на 115200 тоже имеет конечную пропускную способность.

Для теста вообще убери:

Serial.print()

из каждого пакета.

Оставь только счётчики.

Например:

volatile uint32_t rxIrqCount = 0;
volatile uint32_t rxOkCount = 0;
volatile uint32_t rxErrCount = 0;

IRQ:

void Radio_IRQ_Handler() {
    radioActionFlag = true;
    rxIrqCount++;
}

Приём:

if (state == RADIOLIB_ERR_NONE) {
    rxOkCount++;
}
else {
    rxErrCount++;
}

И раз в секунду:

Serial.print("IRQ=");
Serial.print(rxIrqCount);

Serial.print(" OK=");
Serial.print(rxOkCount);

Serial.print(" ERR=");
Serial.print(rxErrCount);

Serial.print(" FIFO=");
Serial.println(audioOut_available());
8. Я бы сейчас вообще заменил весь RX-блок

Вот этот:

if (radioActionFlag) {
    ...
}

на такой:

if (radioActionFlag) {

    radioActionFlag = false;

    if (isTransmittingMode) {
        txReadyForNext = true;
    }
    else {

        uint8_t rawBuffer[PACKET_LENGTH];

        int state = radio.readData(
            rawBuffer,
            PACKET_LENGTH
        );

        if (state == RADIOLIB_ERR_NONE) {

            stat_rx_packet_cnt++;

            if (c2 != NULL) {

                int16_t pcm_out_buffer[CODEC2_SAMPLES_PER_FRM];

                for (int f = 0; f < 2; f++) {

                    codec2_decode(
                        c2,
                        pcm_out_buffer,
                        rawBuffer +
                        f * CODEC2_FRAME_BYTES
                    );

                    noInterrupts();

                    for (int i = 0;
                         i < CODEC2_SAMPLES_PER_FRM;
                         i++) {

                        audioOut_put(
                            pcm_out_buffer[i]
                        );
                    }

                    interrupts();
                }
            }
        }

        // ОБЯЗАТЕЛЬНО возвращаемся в RX
        StartRX_Async();
    }
}
9. И самое главное — исправить StartRX_Async()

Сейчас:

void StartRX_Async() {
    digitalWrite(PIN_TXEN, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_RXEN, HIGH);
    delayMicroseconds(10);
    radio.startReceive();
}

Сделай:

void StartRX_Async() {

    digitalWrite(PIN_TXEN, LOW);
    digitalWrite(PIN_RXEN, HIGH);

    delayMicroseconds(100);

    int state = radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("RX START ERROR=");
        Serial.println(state);
    }
}

Это важно: сейчас ты вообще не проверяешь результат startReceive().

10. И TX тоже обязательно проверять

Сейчас:

radio.startTransmit(outPacket, PACKET_LENGTH);

Лучше:

int state = radio.startTransmit(
    outPacket,
    PACKET_LENGTH
);

if (state != RADIOLIB_ERR_NONE) {

    Serial.print("TX START ERROR=");
    Serial.println(state);

    txReadyForNext = true;
}
11. Есть ещё ошибка в логике PTT

У тебя:

bool stablePttState = HIGH;

а потом:

bool currentRawPtt =
    (digitalRead(PIN_PTT) == LOW);

То есть currentRawPtt — это:

true  = кнопка нажата
false = кнопка отпущена

а stablePttState первоначально:

HIGH = true

Получается после запуска программа считает PTT нажатым, пока debounce не установит правильное состояние.

Правильно:

bool stablePttState = false;

И:

uint32_t lastPttChangeTime = 0;
12. И ещё одна критичная вещь с PTT

У тебя:

if (currentRawPtt != stablePttState) {

    if ((millis() - lastPttChangeTime) > 50) {
        stablePttState = currentRawPtt;
    }

} else {
    lastPttChangeTime = millis();
}

Это работает не так, как обычно ожидается от debounce.

Сделай нормально:

static bool lastRawPtt = false;
static uint32_t pttTimer = 0;

bool rawPtt =
    (digitalRead(PIN_PTT) == LOW);

if (rawPtt != lastRawPtt) {
    lastRawPtt = rawPtt;
    pttTimer = millis();
}

if ((millis() - pttTimer) >= 50) {

    if (stablePttState != rawPtt) {

        stablePttState = rawPtt;

        if (stablePttState) {

            // PTT ON
            __disable_irq();

            audioInFIFO.head = 0;
            audioInFIFO.tail = 0;

            audioOutFIFO.head = 0;
            audioOutFIFO.tail = 0;

            __enable_irq();

            isTransmittingMode = true;
            txReadyForNext = true;

            digitalWrite(PIN_LED, LOW);

            SwitchToTX_Hardware();

        } else {

            // PTT OFF
            isTransmittingMode = false;

            digitalWrite(PIN_LED, HIGH);

            StartRX_Async();
        }
    }
}
Что я считаю наиболее вероятным

По твоему описанию я бы расставил подозрения так:

№1 — взаимодействие codec2_decode() + 8 кГц ISR + FIFO.

№2 — повторный startReceive() после IRQ при ещё неустоявшемся состоянии LLCC68.

№3 — Serial.print() на каждый пакет создаёт ощущение зависания/забивает обработку.

№4 — неправильная работа PTT/debounce.

№5 — ADC стартует в каждом звуковом ISR, хотя на RX вообще не нужен.

И ещё: сам PWM пока не выглядит причиной зависания. Он может давать грязный звук, но он не должен намертво останавливать приём. Поэтому сейчас я бы не трогал TIM1, а сначала добился стабильного:

IRQ → readData → codec2_decode → FIFO → startReceive

После этого отдельно будем чинить качество PWM-аудио.

Если при твоём текущем коде вывод останавливается именно на [I][R:OK], это особенно важно: тогда радио пакет уже принято, и надо проверять именно codec2_decode()/FIFO, а не RF.
