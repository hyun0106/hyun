import serial
import time
import math
import atexit

# ============================================================
# 1. 통신 설정
# ============================================================

port_L = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

baudrate_L = 460800
baudrate_Ardu = 460800

ser_L = serial.Serial(port_L, baudrate_L, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, baudrate_Ardu, timeout=1)


# ============================================================
# 2. 파라미터
# ============================================================

MIN_DIST = 80.0

# RC카 크기: 20cm x 20cm, 라이다 정중앙
CAR_HALF_WIDTH = 100.0
SAFETY_MARGIN = 30.0
PATH_HALF_WIDTH = CAR_HALF_WIDTH + SAFETY_MARGIN  # 130mm

# 정면 경로 검사
PATH_CHECK_DIST = 650.0
PATH_DANGER_DIST = 520.0
PATH_BLOCK_POINTS = 3

# 넓은 전방 열린 길 판단
LOOKAHEAD_DIST = 700.0
OPEN_SIDE_WIDTH = 260.0
CENTER_WIDTH = PATH_HALF_WIDTH
CENTER_BLOCK_DIST = 560.0
CENTER_BLOCK_POINTS = 3

# 좌우 여유공간 검사
SIDE_CHECK_DIST = 260.0
SIDE_MAX_SCORE_DIST = 320.0

# 진짜 막힘 판단
STUCK_FRONT_DIST = 150.0
STUCK_POINTS = 8

# 속도
NORMAL_SPEED = 0.52
AVOID_SPEED = 0.35
BACK_SPEED = 0.40
ESCAPE_SPEED = 0.30

# 조향
AVOID_STEER = 0.55
ESCAPE_STEER = 0.60

# 조향 방향 보정
STEER_SIGN = -1

# 조향 smoothing
SMOOTH = 0.20
prev_steer = 0.0

# 상태
MODE_NORMAL = 0
MODE_BACK = 1
MODE_ESCAPE = 2

mode = MODE_NORMAL

back_count = 0
escape_count = 0
escape_dir = 0

BACK_CYCLES = 5
ESCAPE_CYCLES = 9

# 탈출 방향 판단용
last_escape_dir = 0
escape_fail_count = 0

prev_center_min = 0.0
prev_path_cnt = 999

MAX_ESCAPE_FAIL = 2
IMPROVE_DIST = 80.0


# ============================================================
# 3. 종료 처리
# ============================================================

def cleanup():
    try:
        ser_Ardu.write(b"S\n")
        ser_L.write(bytes([0xA5, 0x25]))  # STOP
        time.sleep(0.1)
        ser_L.close()
        ser_Ardu.close()
    except Exception:
        pass


atexit.register(cleanup)


# ============================================================
# 4. 보조 함수
# ============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def send_forward(steer, speed):
    global prev_steer

    steer = clamp(steer, -0.60, 0.60)
    speed = clamp(speed, 0.0, 1.0)

    steer = SMOOTH * prev_steer + (1.0 - SMOOTH) * steer
    steer = clamp(steer, -0.60, 0.60)

    prev_steer = steer

    ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())


def send_backward(speed=BACK_SPEED):
    global prev_steer

    prev_steer = 0.0
    speed = clamp(speed, 0.0, 1.0)

    ser_Ardu.write(f"B {speed:.2f}\n".encode())


def normalize_angle(angle):
    if angle > 180:
        angle -= 360
    return angle


def polar_to_xy(angle, distance):
    """
    x > 0 : RC카 앞쪽
    y > 0 : 오른쪽
    y < 0 : 왼쪽
    """
    a = normalize_angle(angle)
    theta = math.radians(a)

    x = distance * math.cos(theta)
    y = distance * math.sin(theta)

    return x, y


def choose_wider_dir(left_score, right_score):
    """
    왼쪽 선택  -> -1
    오른쪽 선택 ->  1
    """
    if left_score > right_score:
        return -1
    else:
        return 1


def choose_open_dir(left_open_score, right_open_score, left_score, right_score):
    """
    넓은 전방 열린 길 우선 판단.
    열린 길 점수 차이가 애매하면 기존 좌우 점수 사용.
    """
    diff = abs(left_open_score - right_open_score)

    if diff > 300.0:
        if left_open_score > right_open_score:
            return -1
        else:
            return 1

    return choose_wider_dir(left_score, right_score)


# ============================================================
# 5. LiDAR 시작
# ============================================================

ser_L.write(bytes([0xA5, 0x40]))  # RESET
time.sleep(1.0)

ser_L.write(bytes([0xA5, 0x20]))  # SCAN
time.sleep(0.05)

try:
    ser_L.read(7)  # response descriptor
except Exception:
    pass

print("=" * 60)
print("넓은 전방 판단 + 탈출 방향 개선 판단 LiDAR 주행 시작")
print("RC카 크기: 20cm x 20cm")
print("라이다 위치: 정중앙")
print("구조: NORMAL → BACK → ESCAPE")
print(f"PATH_HALF_WIDTH = {PATH_HALF_WIDTH:.0f} mm")
print(f"PATH_CHECK_DIST = {PATH_CHECK_DIST:.0f} mm")
print(f"PATH_DANGER_DIST = {PATH_DANGER_DIST:.0f} mm")
print(f"LOOKAHEAD_DIST = {LOOKAHEAD_DIST:.0f} mm")
print(f"OPEN_SIDE_WIDTH = {OPEN_SIDE_WIDTH:.0f} mm")
print("=" * 60)


# ============================================================
# 6. 스캔 누적 변수
# ============================================================

scan_buf = []

path_cnt = 0
path_min = 9999.0

front_close_cnt = 0
front_min = 9999.0

left_score = 0.0
right_score = 0.0
left_cnt = 0
right_cnt = 0

left_open_score = 0.0
right_open_score = 0.0
center_block_cnt = 0
center_min = 9999.0


# ============================================================
# 7. 메인 루프
# ============================================================

while True:
    data = ser_L.read(5)

    if len(data) != 5:
        continue

    # --------------------------------------------------------
    # 패킷 검증
    # --------------------------------------------------------
    s_flag = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1

    if s_inv_flag != (1 - s_flag):
        continue

    check_bit = data[1] & 0x01

    if check_bit != 1:
        continue

    quality = data[0] >> 2

    if quality == 0:
        continue

    # --------------------------------------------------------
    # 각도, 거리 계산
    # --------------------------------------------------------
    angle_q6 = ((data[1] >> 1) | (data[2] << 7))
    angle = angle_q6 / 64.0

    distance_q2 = data[3] | (data[4] << 8)
    distance = distance_q2 / 4.0

    if distance < MIN_DIST:
        continue

    scan_buf.append((angle, distance))

    x, y = polar_to_xy(angle, distance)

    # --------------------------------------------------------
    # 1) 정면 차폭 경로 검사
    # --------------------------------------------------------
    if 0 < x < PATH_CHECK_DIST and abs(y) < PATH_HALF_WIDTH:
        path_cnt += 1
        path_min = min(path_min, x)

    # --------------------------------------------------------
    # 2) 진짜 막힘 판단
    # --------------------------------------------------------
    if 0 < x < STUCK_FRONT_DIST and abs(y) < PATH_HALF_WIDTH:
        front_close_cnt += 1
        front_min = min(front_min, x)

    # --------------------------------------------------------
    # 3) 좌우 여유공간 점수 계산
    # --------------------------------------------------------
    if 0 < x < SIDE_CHECK_DIST:
        score = min(distance, SIDE_MAX_SCORE_DIST)

        if y < -PATH_HALF_WIDTH:
            left_score += score
            left_cnt += 1

        elif y > PATH_HALF_WIDTH:
            right_score += score
            right_cnt += 1

    # --------------------------------------------------------
    # 4) 넓은 전방 열린 길 판단
    # --------------------------------------------------------
    if 0 < x < LOOKAHEAD_DIST:

        if abs(y) < CENTER_WIDTH:
            center_block_cnt += 1
            center_min = min(center_min, x)

        elif -OPEN_SIDE_WIDTH < y < -CENTER_WIDTH:
            left_open_score += min(x, LOOKAHEAD_DIST)

        elif CENTER_WIDTH < y < OPEN_SIDE_WIDTH:
            right_open_score += min(x, LOOKAHEAD_DIST)

    # --------------------------------------------------------
    # 한 바퀴 스캔 완료 시 판단
    # --------------------------------------------------------
    if s_flag == 1 and len(scan_buf) > 15:

        path_blocked = (
            path_cnt >= PATH_BLOCK_POINTS and
            path_min < PATH_DANGER_DIST
        )

        center_blocked_ahead = (
            center_block_cnt >= CENTER_BLOCK_POINTS and
            center_min < CENTER_BLOCK_DIST
        )

        stuck = (
            front_close_cnt >= STUCK_POINTS and
            front_min < STUCK_FRONT_DIST
        )

        wider_dir = choose_wider_dir(left_score, right_score)

        open_dir = choose_open_dir(
            left_open_score,
            right_open_score,
            left_score,
            right_score
        )

        # ====================================================
        # 상태 기반 판단
        # ====================================================

        if mode == MODE_BACK:
            send_backward(BACK_SPEED)
            back_count -= 1

            print(
                f"BACK remain={back_count} "
                f"escape_dir={escape_dir}"
            )

            if back_count <= 0:
                mode = MODE_ESCAPE
                escape_count = ESCAPE_CYCLES

        elif mode == MODE_ESCAPE:
            real_steer = STEER_SIGN * ESCAPE_STEER * escape_dir
            send_forward(real_steer, ESCAPE_SPEED)
            escape_count -= 1

            print(
                f"ESCAPE dir={escape_dir} "
                f"real_steer={real_steer:.2f} "
                f"remain={escape_count}"
            )

            if escape_count <= 0:
                mode = MODE_NORMAL

        else:
            # ------------------------------------------------
            # NORMAL 상태
            # ------------------------------------------------

            if stuck:
                escape_improving = (
                    center_min > prev_center_min + IMPROVE_DIST or
                    path_cnt < prev_path_cnt
                )

                if last_escape_dir == 0:
                    escape_dir = open_dir

                elif escape_improving:
                    escape_dir = last_escape_dir
                    escape_fail_count = 0

                else:
                    escape_fail_count += 1

                    if escape_fail_count >= MAX_ESCAPE_FAIL:
                        escape_dir = -last_escape_dir
                        escape_fail_count = 0
                    else:
                        escape_dir = last_escape_dir

                last_escape_dir = escape_dir
                prev_center_min = center_min
                prev_path_cnt = path_cnt

                mode = MODE_BACK
                back_count = BACK_CYCLES

                send_backward(BACK_SPEED)

                print(
                    f"STUCK → BACK "
                    f"front_min={front_min:.0f} "
                    f"front_cnt={front_close_cnt} "
                    f"center_min={center_min:.0f} "
                    f"path_cnt={path_cnt} "
                    f"Lopen={left_open_score:.0f} "
                    f"Ropen={right_open_score:.0f} "
                    f"escape_dir={escape_dir} "
                    f"fail_count={escape_fail_count}"
                )

            elif path_blocked or center_blocked_ahead:
                steer = AVOID_STEER * open_dir
                real_steer = STEER_SIGN * steer

                send_forward(real_steer, AVOID_SPEED)

                print(
                    f"OPEN_AVOID "
                    f"path_min={path_min:.0f} "
                    f"path_cnt={path_cnt} "
                    f"center_min={center_min:.0f} "
                    f"center_cnt={center_block_cnt} "
                    f"Lopen={left_open_score:.0f} "
                    f"Ropen={right_open_score:.0f} "
                    f"open_dir={open_dir} "
                    f"real_steer={real_steer:.2f}"
                )

            else:
                send_forward(0.0, NORMAL_SPEED)

                print(
                    f"CLEAR → FORWARD "
                    f"Lopen={left_open_score:.0f} "
                    f"Ropen={right_open_score:.0f} "
                    f"Lscore={left_score:.0f} "
                    f"Rscore={right_score:.0f}"
                )

        # ----------------------------------------------------
        # 다음 스캔 초기화
        # ----------------------------------------------------
        scan_buf = []

        path_cnt = 0
        path_min = 9999.0

        front_close_cnt = 0
        front_min = 9999.0

        left_score = 0.0
        right_score = 0.0
        left_cnt = 0
        right_cnt = 0

        left_open_score = 0.0
        right_open_score = 0.0
        center_block_cnt = 0
        center_min = 9999.0
