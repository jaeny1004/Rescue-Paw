#include <Wire.h>
#include <math.h>
#include <RescuePaw_TinyML_v2_inferencing.h>

// =====================================================
// RescuePaw V2
// 자동 착지 감지 → 시계열 정렬 → TinyML → 진동
// =====================================================


// ==============================
// 핀
// ==============================

const int FSR_PIN = 1;       // A0 / GPIO1
const int MOTOR_PIN = 8;     // D5 / GPIO8

const uint8_t MPU_ADDR = 0x68;


// ==============================
// 데이터 설정
// ==============================

// 학습과 동일하게 50Hz
const int SAMPLE_INTERVAL_MS = 20;

// 착지 전 0.4초
const int PRE_SAMPLES = 20;

// Edge Impulse 모델:
// 2초 × 50Hz = 100 samples
const int TOTAL_SAMPLES =
    EI_CLASSIFIER_RAW_SAMPLE_COUNT;


// ==============================
// 센서 데이터 구조체
// ==============================

struct SensorSample {

    int fsr;

    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;
};


// 착지 직전 링버퍼
SensorSample preBuffer[PRE_SAMPLES];

// AI에 넣을 최종 2초 데이터
SensorSample alignedBuffer[TOTAL_SAMPLES];

// Edge Impulse 입력 배열
static float modelInput[
    EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
];


// =====================================================
// IMU 읽기
// =====================================================

bool readIMU(
    int16_t &ax,
    int16_t &ay,
    int16_t &az,
    int16_t &gx,
    int16_t &gy,
    int16_t &gz
) {

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    Wire.requestFrom(
        MPU_ADDR,
        (uint8_t)14
    );

    if (Wire.available() != 14) {
        return false;
    }

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();

    // 온도값 2바이트 버림
    Wire.read();
    Wire.read();

    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();

    return true;
}


// =====================================================
// 센서 한 번 읽기
// =====================================================

bool readSensor(SensorSample &s) {

    s.fsr = analogRead(FSR_PIN);

    return readIMU(
        s.ax,
        s.ay,
        s.az,
        s.gx,
        s.gy,
        s.gz
    );
}


// =====================================================
// 가속도 크기
// =====================================================

float accelMagnitude(const SensorSample &s) {

    return sqrtf(
        (float)s.ax * s.ax +
        (float)s.ay * s.ay +
        (float)s.az * s.az
    );
}


// =====================================================
// 50Hz 샘플 한 번
// =====================================================

bool read50Hz(SensorSample &s) {

    unsigned long start =
        micros();

    if (!readSensor(s)) {
        return false;
    }

    long elapsed =
        (long)(micros() - start);

    long remain =
        SAMPLE_INTERVAL_MS * 1000L
        - elapsed;

    if (remain > 0) {
        delayMicroseconds(remain);
    }

    return true;
}


// =====================================================
// Setup
// =====================================================

void setup() {

    Serial.begin(115200);

    pinMode(MOTOR_PIN, OUTPUT);

    // 처음엔 반드시 진동 OFF
    digitalWrite(
        MOTOR_PIN,
        LOW
    );


    // --------------------------
    // IMU
    // --------------------------

    Wire.begin(
        11,     // SDA
        12      // SCL
    );


    // MPU 깨우기
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    delay(100);


    // Accelerometer ±16g
    // 학습 데이터와 동일
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x18);
    Wire.endTransmission();


    // Gyroscope ±2000 dps
    // 학습 데이터와 동일
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x18);
    Wire.endTransmission();

    delay(500);


    // --------------------------
    // 모델 규격 확인
    // --------------------------

    Serial.println();
    Serial.println("==============================");
    Serial.println(" RescuePaw V2 TinyML");
    Serial.println("==============================");

    Serial.print("Frequency : ");
    Serial.print(EI_CLASSIFIER_FREQUENCY);
    Serial.println(" Hz");

    Serial.print("Samples   : ");
    Serial.println(
        EI_CLASSIFIER_RAW_SAMPLE_COUNT
    );

    Serial.print("Channels  : ");
    Serial.println(
        EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME
    );

    Serial.print("Inputs    : ");
    Serial.println(
        EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
    );


    // 우리가 학습한 모델 규격인지 체크
    if (
        EI_CLASSIFIER_RAW_SAMPLE_COUNT != 100 ||
        EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 7
    ) {

        Serial.println();
        Serial.println("!!! MODEL SIZE ERROR !!!");
        Serial.println(
            "Expected: 100 samples x 7 channels"
        );

        while (1) {
            delay(1000);
        }
    }


    Serial.println();
    Serial.println(
        "S 입력 -> 착지 감지 시작"
    );

    Serial.println("==============================");
}


// =====================================================
// Loop
// =====================================================

void loop() {

    if (Serial.available() > 0) {

        char c =
            Serial.read();

        if (
            c == 'S' ||
            c == 's'
        ) {

            runRescuePaw();
        }
    }
}


// =====================================================
// RescuePaw 한 번 실행
// =====================================================

void runRescuePaw() {

    // 센서 측정 중에는 진동 금지
    digitalWrite(
        MOTOR_PIN,
        LOW
    );


    Serial.println();
    Serial.println("==============================");
    Serial.println("PREPARE");
    Serial.println("==============================");

    delay(1000);


    // =================================================
    // 1. 평상시 가속도 기준값 측정
    // =================================================

    Serial.println("CALIBRATING...");

    const int CALIBRATION_SAMPLES = 25;

    float accelSum = 0.0f;


    for (
        int i = 0;
        i < CALIBRATION_SAMPLES;
        i++
    ) {

        SensorSample s;

        if (!read50Hz(s)) {

            Serial.println(
                "IMU READ ERROR"
            );

            return;
        }

        accelSum +=
            accelMagnitude(s);
    }


    float baselineAccel =
        accelSum /
        CALIBRATION_SAMPLES;


    // 학습 데이터 수집 코드와 동일
    float triggerThreshold =
        baselineAccel * 1.60f;


    if (
        triggerThreshold
        < baselineAccel + 1200.0f
    ) {

        triggerThreshold =
            baselineAccel + 1200.0f;
    }


    Serial.print("Baseline  : ");
    Serial.println(
        baselineAccel
    );

    Serial.print("Threshold : ");
    Serial.println(
        triggerThreshold
    );


    // =================================================
    // 2. 착지 전 0.4초 확보
    // =================================================

    SensorSample preBufferLocal[
        PRE_SAMPLES
    ];

    int preIndex = 0;


    for (
        int i = 0;
        i < PRE_SAMPLES;
        i++
    ) {

        if (!read50Hz(
            preBufferLocal[i]
        )) {

            Serial.println(
                "IMU READ ERROR"
            );

            return;
        }
    }


    preIndex = 0;


    // =================================================
    // 3. 착지 대기
    // =================================================

    Serial.println();
    Serial.println("==============================");
    Serial.println("ARMED");
    Serial.println(
        "이제 떨어뜨리세요."
    );
    Serial.println("==============================");


    SensorSample triggerSample;


    while (true) {

        SensorSample current;

        if (!read50Hz(current)) {

            Serial.println(
                "IMU READ ERROR"
            );

            return;
        }


        float mag =
            accelMagnitude(current);


        // ---------------------------------
        // 착지 감지
        // ---------------------------------

        if (
            mag >= triggerThreshold
        ) {

            triggerSample =
                current;

            break;
        }


        // 아직 착지 전이면
        // 링버퍼 계속 갱신
        preBufferLocal[preIndex] =
            current;

        preIndex++;

        if (
            preIndex >= PRE_SAMPLES
        ) {

            preIndex = 0;
        }
    }


    // =================================================
    // 4. 착지 전 데이터 정렬
    // =================================================

    for (
        int i = 0;
        i < PRE_SAMPLES;
        i++
    ) {

        int sourceIndex =
            (preIndex + i)
            % PRE_SAMPLES;

        alignedBuffer[i] =
            preBufferLocal[
                sourceIndex
            ];
    }


    // =================================================
    // 5. 착지 순간 = index 20
    // =================================================

    alignedBuffer[
        PRE_SAMPLES
    ] = triggerSample;


    // =================================================
    // 6. 착지 후 나머지 데이터
    // =================================================

    for (
        int i =
            PRE_SAMPLES + 1;

        i < TOTAL_SAMPLES;

        i++
    ) {

        if (!read50Hz(
            alignedBuffer[i]
        )) {

            Serial.println(
                "IMU READ ERROR"
            );

            return;
        }
    }


    // 여기까지 끝나야 출력
    // sampling timing에 영향을 안 줌
    Serial.println();
    Serial.println(
        "IMPACT DETECTED"
    );

    Serial.println(
        "DATA ALIGNED"
    );


    // =================================================
    // 7. AI 입력 배열 구성
    //
    // 학습 CSV 순서와 반드시 동일:
    //
    // fsr, ax, ay, az, gx, gy, gz
    // =================================================

    for (
        int sample = 0;
        sample < TOTAL_SAMPLES;
        sample++
    ) {

        int idx =
            sample *
            EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;


        modelInput[idx + 0] =
            (float)alignedBuffer[sample].fsr;

        modelInput[idx + 1] =
            (float)alignedBuffer[sample].ax;

        modelInput[idx + 2] =
            (float)alignedBuffer[sample].ay;

        modelInput[idx + 3] =
            (float)alignedBuffer[sample].az;

        modelInput[idx + 4] =
            (float)alignedBuffer[sample].gx;

        modelInput[idx + 5] =
            (float)alignedBuffer[sample].gy;

        modelInput[idx + 6] =
            (float)alignedBuffer[sample].gz;
    }


    // =================================================
    // 8. Edge Impulse signal 생성
    // =================================================

    signal_t signal;


    int signalError =
        numpy::signal_from_buffer(
            modelInput,
            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE,
            &signal
        );


    if (
        signalError != 0
    ) {

        Serial.print(
            "Signal error: "
        );

        Serial.println(
            signalError
        );

        return;
    }


    // =================================================
    // 9. TinyML 추론
    // =================================================

    ei_impulse_result_t result = { 0 };


    EI_IMPULSE_ERROR res =
        run_classifier(
            &signal,
            &result,
            false
        );


    if (
        res != EI_IMPULSE_OK
    ) {

        Serial.print(
            "Inference error: "
        );

        Serial.println(
            res
        );

        return;
    }


    // =================================================
    // 10. 결과 확인
    // =================================================

    float safeScore = 0.0f;
    float unsafeScore = 0.0f;


    Serial.println();
    Serial.println("==============================");
    Serial.println("AI RESULT");
    Serial.println("==============================");


    for (
        size_t i = 0;
        i < EI_CLASSIFIER_LABEL_COUNT;
        i++
    ) {

        const char *label =
            result.classification[i].label;

        float score =
            result.classification[i].value;


        Serial.print(label);
        Serial.print(" : ");
        Serial.println(
            score,
            4
        );


        if (
            strcmp(
                label,
                "safe"
            ) == 0
        ) {

            safeScore =
                score;
        }


        if (
            strcmp(
                label,
                "unsafe"
            ) == 0
        ) {

            unsafeScore =
                score;
        }
    }


    // =================================================
    // 11. 실제 ESP32-S3 추론시간
    // =================================================

    Serial.println();

    Serial.print("DSP time : ");
    Serial.print(
        result.timing.dsp
    );
    Serial.println(" ms");


    Serial.print(
        "Classification time : "
    );

    Serial.print(
        result.timing.classification
    );

    Serial.println(" ms");


    // =================================================
    // 12. SAFE / UNSAFE
    // =================================================

    Serial.println();


    // 일단 Model Testing과 동일하게
    // 가장 높은 확률 클래스 사용

    if (
        unsafeScore > safeScore
    ) {

        Serial.println(
            ">>> UNSAFE <<<"
        );

        Serial.println(
            "VIBRATION ON"
        );


        // 센서 수집이 완전히 끝난 뒤에만 진동
        digitalWrite(
            MOTOR_PIN,
            HIGH
        );

        delay(800);

        digitalWrite(
            MOTOR_PIN,
            LOW
        );


        Serial.println(
            "VIBRATION OFF"
        );
    }

    else {

        Serial.println(
            ">>> SAFE <<<"
        );

        digitalWrite(
            MOTOR_PIN,
            LOW
        );
    }


    Serial.println();
    Serial.println("==============================");
    Serial.println(
        "다음 실험: S 입력"
    );
    Serial.println("==============================");
}