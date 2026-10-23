import serial
import time
import os
import re

# ==============================
# 설정
# ==============================

PORT = "COM5"       # 포트가 바뀌었으면 수정
BAUD = 115200

SAVE_DIR = "dataset_v2"

os.makedirs(SAVE_DIR, exist_ok=True)


# ==============================
# Label 선택
# ==============================

while True:
    label = input("Label 입력 (safe / unsafe): ").strip().lower()

    if label in ["safe", "unsafe"]:
        break

    print("safe 또는 unsafe만 입력하세요.")


# ==============================
# 다음 파일 번호 찾기
# safe.01.csv
# safe.02.csv ...
# ==============================

def get_next_number(label):

    max_num = 0

    pattern = re.compile(
        rf"^{re.escape(label)}\.(\d+)\.csv$"
    )

    for filename in os.listdir(SAVE_DIR):

        match = pattern.match(filename)

        if match:
            num = int(match.group(1))
            max_num = max(max_num, num)

    return max_num + 1


trial_number = get_next_number(label)


# ==============================
# Serial 연결
# ==============================

print()
print(f"{PORT} 연결 중...")

ser = serial.Serial(
    PORT,
    BAUD,
    timeout=1
)

time.sleep(2)

# ESP32 부팅 메시지 정리
ser.reset_input_buffer()

print("연결 완료")
print()
print("==============================")
print(f"현재 Label : {label}")
print(f"다음 Trial : {trial_number:02d}")
print("==============================")
print()
print("Enter = 새 실험")
print("q + Enter = 종료")


# ==============================
# 반복 실험
# ==============================

while True:

    command = input("\n>>> ").strip().lower()

    if command == "q":
        break


    # ----------------------------------
    # Arduino에 실험 시작 명령
    # ----------------------------------

    ser.reset_input_buffer()

    ser.write(b"S")
    ser.flush()

    print()
    print("------------------------------")
    print(f"{label.upper()} Trial {trial_number:02d}")
    print("------------------------------")


    csv_lines = []

    recording = False


    # ----------------------------------
    # Arduino 응답 기다림
    # ----------------------------------

    while True:

        try:
            raw = ser.readline()

            if not raw:
                continue

            line = raw.decode(
                "utf-8",
                errors="ignore"
            ).strip()

        except Exception as e:
            print("Serial read error:", e)
            continue


        if not line:
            continue


        # Arduino 진행 상태를 화면에도 출력
        if not recording:
            print(line)


        # ----------------------------------
        # CSV 시작
        # ----------------------------------

        if line == "START":

            recording = True

            print()
            print("CSV 수신 중...")

            continue


        # ----------------------------------
        # CSV 종료
        # ----------------------------------

        if line == "END":

            break


        # ----------------------------------
        # START~END 데이터만 저장
        # ----------------------------------

        if recording:

            csv_lines.append(line)


    # ==============================
    # CSV 저장
    # ==============================

    filename = (
        f"{label}.{trial_number:02d}.csv"
    )

    filepath = os.path.join(
        SAVE_DIR,
        filename
    )


    with open(
        filepath,
        "w",
        encoding="utf-8",
        newline=""
    ) as f:

        for line in csv_lines:
            f.write(line + "\n")


    print()
    print("==============================")
    print("저장 완료")
    print(filepath)
    print(f"CSV lines : {len(csv_lines)}")
    print("==============================")


    trial_number += 1


ser.close()

print("종료")