#include <Wire.h>
#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

// 모터 핀 및 채널 설정
constexpr int MOTOR_A = 23, MOTOR_B = 19, MOTOR_C = 18, MOTOR_D = 26;
constexpr int CHANNEL_A = 0, CHANNEL_B = 1, CHANNEL_C = 2, CHANNEL_D = 3;
constexpr int MOTOR_FREQ = 5000;              // PWM 주파수 (5kHz)
constexpr int MOTOR_RESOLUTION = 10;          // PWM 해상도 (0~1023)

// 자세 제어용 전역 변수
volatile int throttle = 0;                    // 스로틀 (출력 세기)
volatile double tAngleX = 0.0, tAngleY = 0.0, tAngleZ = 0.0;  // 목표 자세
volatile double currentAngX = 0.0, currentAngY = 0.0, currentAngZ = 0.0;  // 현재 자세

// FreeRTOS 태스크 핸들
TaskHandle_t TaskDroneControlHandle;
TaskHandle_t TaskAutoAviHandle;

// 디스플레이는 현재 사용하지 않음 (주석 처리)
/*
void display_setup() {
    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(TFT_DARKGREY);
    tft.setCursor(0, 30);
    tft.setTextFont(2);
    tft.setTextColor(TFT_PINK, TFT_BLACK);
    tft.setTextSize(1.5);
    tft.println("Scientia Drone");
    tft.setTextFont(4);
    tft.setTextColor(TFT_BLUE);
    tft.println("GWNU_CS");
    delay(2000);
    tft.fillScreen(TFT_BLACK);
}
*/

// IMU(MPU6050) 초기화 및 모터 PWM 설정
void drone_setup() {
    Wire.begin();
    Wire.setClock(400000); // 고속 I2C (400kHz)
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);       // 슬립 해제 레지스터
    Wire.write(0x00);       // 전원 온
    Wire.endTransmission(true);

    // 모터 핀을 PWM 채널에 연결
    ledcAttachPin(MOTOR_A, CHANNEL_A);
    ledcAttachPin(MOTOR_B, CHANNEL_B);
    ledcAttachPin(MOTOR_C, CHANNEL_C);
    ledcAttachPin(MOTOR_D, CHANNEL_D);

    // 각 PWM 채널 초기 설정
    ledcSetup(CHANNEL_A, MOTOR_FREQ, MOTOR_RESOLUTION);
    ledcSetup(CHANNEL_B, MOTOR_FREQ, MOTOR_RESOLUTION);
    ledcSetup(CHANNEL_C, MOTOR_FREQ, MOTOR_RESOLUTION);
    ledcSetup(CHANNEL_D, MOTOR_FREQ, MOTOR_RESOLUTION);

    // 초기 모터 정지
    ledcWrite(CHANNEL_A, 0);
    ledcWrite(CHANNEL_B, 0);
    ledcWrite(CHANNEL_C, 0);
    ledcWrite(CHANNEL_D, 0);
}

// 드론 자세 제어 태스크 (200Hz 루프 + 예측 보정 포함)
void TaskDroneControl(void* pvParameters) {
    // 센서 오프셋 계산용 누적 변수
    static int32_t AcXSum = 0, AcYSum = 0, AcZSum = 0;
    static int32_t GyXSum = 0, GyYSum = 0, GyZSum = 0;
    static double AcXOff = 0, AcYOff = 0, AcZOff = 0;
    static double GyXOff = 0, GyYOff = 0, GyZOff = 0;
    static int cnt_sample = 1000;  // 1000개 샘플 평균

    unsigned long lastMicros = micros();  // 루프 주기 체크
    static double angX_g = 0, angY_g = 0, angZ_g = 0;
    static double angX_c = 0, angY_c = 0;

    // PID 상수 및 필터 설정
    constexpr double Kp = 1.6;
    constexpr double Ki = 0.05;
    constexpr double Kd = 6.0;
    constexpr double ALPHA = 0.95;
    constexpr double PREDICT_TIME = 0.015; // 예측 보정 시간 (15ms)

    static double iSumX = 0, iSumY = 0, iSumZ = 0;

    for (;;) {
        unsigned long nowMicros = micros();
        if (nowMicros - lastMicros < 5000) continue;  // 200Hz 주기 (5ms)
        double dt = (nowMicros - lastMicros) / 1e6;
        lastMicros = nowMicros;

        // MPU6050 데이터 요청
        Wire.beginTransmission(0x68);
        Wire.write(0x3B); // 가속도 시작 주소
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)0x68, (size_t)14, (bool)true);

        // 가속도 및 자이로 데이터 수신
        int16_t AcX = (Wire.read() << 8) | Wire.read();
        int16_t AcY = (Wire.read() << 8) | Wire.read();
        int16_t AcZ = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // 온도 무시
        int16_t GyX = (Wire.read() << 8) | Wire.read();
        int16_t GyY = (Wire.read() << 8) | Wire.read();
        int16_t GyZ = (Wire.read() << 8) | Wire.read();

        // 초기 1000 샘플 동안 오프셋 계산
        if (cnt_sample > 0) {
            AcXSum += AcX; AcYSum += AcY; AcZSum += AcZ;
            GyXSum += GyX; GyYSum += GyY; GyZSum += GyZ;
            cnt_sample--;
            if (cnt_sample == 0) {
                AcXOff = AcXSum / 1000.0;
                AcYOff = AcYSum / 1000.0;
                AcZOff = AcZSum / 1000.0;
                GyXOff = GyXSum / 1000.0;
                GyYOff = GyYSum / 1000.0;
                GyZOff = GyZSum / 1000.0;
            }
            delay(1);
            continue;
        }

        // 오프셋 제거 및 보정
        double dAcX = AcX - AcXOff;
        double dAcY = AcY - AcYOff;
        double dAcZ = AcZ - AcZOff + 16384;
        double dGyX = GyX - GyXOff;
        double dGyY = GyY - GyYOff;
        double dGyZ = GyZ - GyZOff;

        constexpr double GYRO_SCALE = 131.0;
        double rateX = dGyX / GYRO_SCALE;
        double rateY = dGyY / GYRO_SCALE;
        double rateZ = dGyZ / GYRO_SCALE;

        // 자이로 적분
        angX_g += rateX * dt;
        angY_g += rateY * dt;
        angZ_g += rateZ * dt;

        // 가속도 기반 각도 계산
        constexpr double R2D = 180.0 / 3.14159;
        double magYZ = sqrt(dAcY * dAcY + dAcZ * dAcZ);
        double magXZ = sqrt(dAcX * dAcX + dAcZ * dAcZ);
        double angY_a = atan(-dAcX / magYZ) * R2D;
        double angX_a = atan(dAcY / magXZ) * R2D;

        // 보완 필터로 자이로 + 가속도 융합
        angX_c = ALPHA * (angX_c + rateX * dt) + (1 - ALPHA) * angX_a;
        angY_c = ALPHA * (angY_c + rateY * dt) + (1 - ALPHA) * angY_a;
        double angZ_c = angZ_g;

        if (throttle == 0) angX_c = angY_c = angZ_c = 0;

        // 현재 자세 갱신
        currentAngX = angX_c;
        currentAngY = angY_c;
        currentAngZ = angZ_c;

        // ✅ 예측 보정: 기울어질 각도를 미리 계산
        double predX = angX_c + rateX * PREDICT_TIME;
        double predY = angY_c + rateY * PREDICT_TIME;
        double predZ = angZ_c + rateZ * PREDICT_TIME;

        // PID 오차 계산
        double errX = tAngleX - predX;
        double errY = tAngleY - predY;
        double errZ = tAngleZ - predZ;

        iSumX += Ki * errX * dt;
        iSumY += Ki * errY * dt;
        iSumZ += Ki * errZ * dt;

        double corrX = Kp * errX + Kd * (-rateX) + iSumX;
        double corrY = Kp * errY + Kd * (-rateY) + iSumY;
        double corrZ = Kp * errZ + Kd * (-rateZ) + iSumZ;

        // 모터 출력 제한
        corrX = constrain(corrX, -300, 300);
        corrY = constrain(corrY, -300, 300);
        corrZ = constrain(corrZ, -300, 300);

        if (throttle == 0) iSumX = iSumY = iSumZ = 0;

        // 모터별 출력 계산
        int outA = constrain(int(throttle + corrX - corrY + corrZ), 0, 1000);
        int outB = constrain(int(throttle - corrX - corrY - corrZ), 0, 1000);
        int outC = constrain(int(throttle - corrX + corrY + corrZ), 0, 1000);
        int outD = constrain(int(throttle + corrX + corrY - corrZ), 0, 1000);

        ledcWrite(CHANNEL_A, outA);
        ledcWrite(CHANNEL_B, outB);
        ledcWrite(CHANNEL_C, outC);
        ledcWrite(CHANNEL_D, outD);

        // 착륙 감지 시 자동 종료
        if (throttle < 100 && abs(rateX) < 0.5 && abs(rateY) < 0.5 && abs(rateZ) < 0.5) {
            ledcWrite(CHANNEL_A, 0);
            ledcWrite(CHANNEL_B, 0);
            ledcWrite(CHANNEL_C, 0);
            ledcWrite(CHANNEL_D, 0);
            throttle = 0;
            vTaskDelete(NULL);
        }
    }
}

// 자동 이륙/착륙 시나리오 태스크
void TaskAutoAviMain(void* pvParameters) {
    pinMode(0, INPUT_PULLUP);
    while (digitalRead(0) == HIGH) delay(10);  // 버튼 누를 때까지 대기

    // 이륙 시 현재 자세를 목표로 설정
    tAngleX = currentAngX;
    tAngleY = currentAngY;
    tAngleZ = 0.0;

    // 점진적 이륙
    for (int t = 0; t <= 475; ++t) {
        throttle++;
        delay(5);
    }

    delay(60000);  // 60초간 호버링 유지

    // 천천히 하강
    for (int t = 475; t >= 400; --t) {
        throttle--;
        delay(6);
    }

    delay(2000);  // 중간 호버링

    // 완전 착륙
    for (int t = 400; t >= 0; --t) {
        throttle--;
        delay(100);
    }

    throttle = 0;
    vTaskDelete(TaskAutoAviHandle);
}

// Arduino 기본 setup 함수
void setup() {
    // display_setup(); // 현재 비활성화
    drone_setup();

    // 자세 제어 태스크 실행
    xTaskCreatePinnedToCore(TaskDroneControl, "TaskDroneControl", 10000, nullptr, 2, &TaskDroneControlHandle, 0);

    // 자동 이륙/착륙 태스크 실행
    xTaskCreatePinnedToCore(TaskAutoAviMain, "TaskAutoAvi", 10000, nullptr, 1, &TaskAutoAviHandle, 1);
}

// loop()는 FreeRTOS 구조에서는 사용하지 않음
void loop() {}
