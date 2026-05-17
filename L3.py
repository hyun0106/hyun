import serial
import time
import math
import atexit

# ============================================================
# 1. 통신 설정
# ============================================================

port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

baudrate_L    = 460800
baudrate_Ardu = 460800

ser_L    = serial.Serial(port_L, baudrate_L, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, baudrate_Ardu, timeout=1)


# ============================================================
# 2. 파라미터
# ============================================================

MIN_DIST = 80.0

# RC카 크기: 20cm x 20cm, 라이다 정중앙
CAR_HALF_WIDTH  = 100.0
SAFETY_MARGIN   = 15.0
PATH_HALF_WIDTH = CAR_HALF_WIDTH + SAFETY_MARGIN   # 115mm

# 정면 경로 검사
PATH_CHECK_DIST   = 480.0
PATH_DANGER_DIST  = 430.0
PATH_BLOCK_POINTS = 3

# 전방 열린 길 판단
LOOKAHEAD_DIST      = 580.0
OPEN_SIDE_WIDTH     = 300.0
CENTER_WIDTH        = PATH_HALF_WIDTH
CENTER_BLOCK_DIST   = 450.0
CENTER_BLOCK_POINTS = 3

# 좌우 여유공간 검사
SIDE_CHECK_DIST     = 280.0
SIDE_MAX_SCORE_DIST = 320.0

# 진짜 막힘 판단
STUCK_FRONT_DIST = 150.0
STUCK_POINTS     = 7

# 속도: 전체적으로 상향
NORMAL_SPEED    = 0.68
AVOID_SPEED_MIN = 0.38
AVOID_SPEED_MAX = 0.55
BACK_SPEED      = 0.42
ESCAPE_SPEED    = 0.42

# 조향: 강하게
AVOID_STEER  = 0.72
ESCAPE_STEER = 0.82

# 실제 조향 제한
MAX_STEER = 0.85

# 조향 방향 보정
STEER_SIGN = -1

# 조향 smoothing: 거의 즉각 반응
SMOOTH     = 0.06
prev_steer = 0.0

# 상태
MODE_NORMAL = 0
MODE_BACK   = 1
MODE_ESCAPE = 2

mode = MODE_NORMAL

back_count   = 0
escape_count = 0
escape_dir   = 0

# 후진은 길게, 전진 회전은 적당히
BACK_CYCLES   = 6
ESCAPE_CYCLES = 7

last_escape_dir   = 0
escape_fail_count = 0

prev_center_min = 0.0
prev_path_cnt   = 999

MAX_ESCAPE_FAIL = 3
IMPROVE_DIST    = 50.0


# ============================================================
# 3. Gap Finding 파라미터
# ============================================================

GAP_MAX_VALID_DIST = 1500.0
GAP_MIN_ANGLE_DIFF = 3.0
GAP_MIN_DIST_JUMP  = 100.0
GAP_MIN_PASS_WIDTH = CAR_HALF_WIDTH * 2 + 30   # 230mm
GAP_MIN_SCORE      = 180.0


# ============================================================
# 4. 종료 처리
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
# 5. 보조 함수
# ============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def send_forward(steer, speed):
    global prev_steer

    steer = clamp(steer, -MAX_STEER, MAX_STEER)
    speed = clamp(speed, 0.0, 1.0)

    steer = SMOOTH * prev_steer + (1.0 - SMOOTH) * steer
    steer = clamp(steer, -MAX_STEER, MAX_STEER)

    prev_steer = steer
    ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())


def send_forward_direct(steer, speed):
    """ESCAPE 전용. smoothing 없이 바로 조향."""
    global prev_steer

    steer = clamp(steer, -MAX_STEER, MAX_STEER)
    speed = clamp(speed, 0.0, 1.0)

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
    return -1 if left_score > right_score else 1


def choose_open_dir(left_open_score, right_open_score, left_score, right_score):
    diff = abs(left_open_score - right_open_score)

    if diff > 300.0:
        return -1 if left_open_score > right_open_score else 1

    return choose_wider_dir(left_score, right_score)


def calc_avoid_speed(path_min):
    """장애물이 가까울수록 느리게, 멀수록 빠르게."""
    ratio = clamp(path_min / PATH_DANGER_DIST, 0.0, 1.0)
    return AVOID_SPEED_MIN + (AVOID_SPEED_MAX - AVOID_SPEED_MIN) * ratio


# ============================================================
# 6. Gap Finding 함수
# ============================================================

def find_best_gap(scan_buf):
    front_points = []

    for angle, distance in scan_buf:
        a = normalize_angle(angle)

        if -90.0 < a < 90.0 and MIN_DIST < distance < GAP_MAX_VALID_DIST:
            front_points.append((a, distance))

    if len(front_points) < 5:
        return 0.0, 0.0

    front_points.sort(key=lambda p: p[0])

    gaps = []

    for i in range(len(front_points) - 1):
        a1, d1 = front_points[i]
        a2, d2 = front_points[i + 1]

        angle_diff = a2 - a1
        dist_jump  = abs(d2 - d1)

        if angle_diff < GAP_MIN_ANGLE_DIFF:
            continue

        if dist_jump < GAP_MIN_DIST_JUMP:
            continue

        theta = math.radians(angle_diff)

        physical_width = math.sqrt(
            d1 ** 2 + d2 ** 2 - 2.0 * d1 * d2 * math.cos(theta)
        )

        if physical_width < GAP_MIN_PASS_WIDTH:
            continue

        gap_angle     = (a1 + a2) / 2.0
        angle_penalty = abs(gap_angle) / 90.0
        score         = physical_width * (1.0 - 0.5 * angle_penalty)

        gaps.append((gap_angle, score))

    if not gaps:
        return 0.0, 0.0

    best = max(gaps, key=lambda g: g[1])
    return best[0], best[1]


# ============================================================
# 7. 조향 강도 계산
# ============================================================

def calc_avoid_steer(gap_angle, path_min, open_dir, gap_valid):
    """
    방향: gap이 유효하면 gap 방향, 없으면 open_dir.
    강도: 장애물 거리에 반비례.
    """

    if gap_valid:
        steer_dir = 1 if gap_angle >= 0 else -1
    else:
        steer_dir = open_dir

    intensity = 1.0 - clamp(path_min / PATH_DANGER_DIST, 0.0, 1.0)
    intensity = clamp(intensity + 0.55, 0.55, 1.0)

    steer = steer_dir * AVOID_STEER * intensity
    return clamp(steer, -MAX_STEER, MAX_STEER)


# ============================================================
# 8. LiDAR 시작
# ============================================================

ser_L.write(bytes([0xA5, 0x40]))   # RESET
time.sleep(1.0)

ser_L.write(bytes([0xA5, 0x20]))   # SCAN
time.sleep(0.05)

try:
    ser_L.read(7)   # response descriptor
except Exception:
    pass


print("=" * 60)
print("LiDAR 자율주행 시작 - 강한 회전/긴 후진 버전")
print(f"PATH_HALF_WIDTH    = {PATH_HALF_WIDTH:.0f} mm")
print(f"PATH_CHECK_DIST    = {PATH_CHECK_DIST:.0f} mm")
print(f"PATH_DANGER_DIST   = {PATH_DANGER_DIST:.0f} mm")
print(f"STUCK_FRONT_DIST   = {STUCK_FRONT_DIST:.0f} mm")
print(f"STUCK_POINTS       = {STUCK_POINTS}")
print(f"NORMAL_SPEED       = {NORMAL_SPEED:.2f}")
print(f"AVOID_SPEED        = {AVOID_SPEED_MIN:.2f} ~ {AVOID_SPEED_MAX:.2f}")
print(f"BACK_SPEED         = {BACK_SPEED:.2f}")
print(f"ESCAPE_SPEED       = {ESCAPE_SPEED:.2f}")
print(f"BACK_CYCLES        = {BACK_CYCLES}")
print(f"ESCAPE_CYCLES      = {ESCAPE_CYCLES}")
print(f"MAX_STEER          = {MAX_STEER:.2f}")
print("=" * 60)


# ============================================================
# 9. 스캔 누적 변수
# ============================================================

scan_buf = []

path_cnt = 0
path_min = 9999.0

front_close_cnt = 0
front_min       = 9999.0

left_score  = 0.0
right_score = 0.0
left_cnt    = 0
right_cnt   = 0

left_open_score  = 0.0
right_open_score = 0.0
center_block_cnt = 0
center_min       = 9999.0


# ============================================================
# 10. 메인 루프
# ============================================================

while True:
    data = ser_L.read(5)

    if len(data) != 5:
        continue

    # 패킷 검증
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1

    if s_inv_flag != (1 - s_flag):
        continue

    check_bit = data[1] & 0x01

    if check_bit != 1:
        continue

    quality = data[0] >> 2

    if quality == 0:
        continue

    # 각도, 거리 계산
    angle_q6 = (data[1] >> 1) | (data[2] << 7)
    angle = angle_q6 / 64.0

    distance_q2 = data[3] | (data[4] << 8)
    distance = distance_q2 / 4.0

    if distance < MIN_DIST:
        continue

    scan_buf.append((angle, distance))

    x, y = polar_to_xy(angle, distance)

    # 1) 정면 차폭 경로 검사
    if 0 < x < PATH_CHECK_DIST and abs(y) < PATH_HALF_WIDTH:
        path_cnt += 1
        path_min = min(path_min, x)

    # 2) 진짜 막힘 판단
    if 0 < x < STUCK_FRONT_DIST and abs(y) < PATH_HALF_WIDTH:
        front_close_cnt += 1
        front_min = min(front_min, x)

    # 3) 좌우 여유공간 점수 계산
    if 0 < x < SIDE_CHECK_DIST:
        score = min(distance, SIDE_MAX_SCORE_DIST)

        if y < -PATH_HALF_WIDTH:
            left_score += score
            left_cnt += 1

        elif y > PATH_HALF_WIDTH:
            right_score += score
            right_cnt += 1

    # 4) 넓은 전방 열린 길 판단
    if 0 < x < LOOKAHEAD_DIST:

        if abs(y) < CENTER_WIDTH:
            center_block_cnt += 1
            center_min = min(center_min, x)

        elif -OPEN_SIDE_WIDTH < y < -CENTER_WIDTH:
            left_open_score += min(x, LOOKAHEAD_DIST)

        elif CENTER_WIDTH < y < OPEN_SIDE_WIDTH:
            right_open_score += min(x, LOOKAHEAD_DIST)

    # 한 바퀴 스캔 완료 시 판단
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

        open_dir = choose_open_dir(
            left_open_score,
            right_open_score,
            left_score,
            right_score
        )

        gap_angle, gap_score = find_best_gap(scan_buf)
        gap_valid = gap_score >= GAP_MIN_SCORE

        # ====================================================
        # 상태 기반 판단
        # ====================================================

        if mode == MODE_BACK:
            send_backward(BACK_SPEED)
            back_count -= 1

            print(
                f"BACK "
                f"remain={back_count} "
                f"escape_dir={escape_dir} "
                f"front_min={front_min:.0f}"
            )

            if back_count <= 0:
                fresh_gap_angle, fresh_gap_score = find_best_gap(scan_buf)
                fresh_gap_valid = fresh_gap_score >= GAP_MIN_SCORE

                if fresh_gap_valid:
                    escape_dir = 1 if fresh_gap_angle >= 0 else -1

                mode = MODE_ESCAPE
                escape_count = ESCAPE_CYCLES

        elif mode == MODE_ESCAPE:
            real_steer = STEER_SIGN * ESCAPE_STEER * escape_dir

            send_forward_direct(real_steer, ESCAPE_SPEED)
            escape_count -= 1

            print(
                f"ESCAPE "
                f"dir={escape_dir} "
                f"real_steer={real_steer:.2f} "
                f"remain={escape_count}"
            )

            if escape_count <= 0:
                mode = MODE_NORMAL

        else:
            if stuck:
                escape_improving = (
                    center_min > prev_center_min + IMPROVE_DIST or
                    path_cnt < prev_path_cnt
                )

                if last_escape_dir == 0:
                    escape_dir = (1 if gap_angle >= 0 else -1) if gap_valid else open_dir

                elif escape_improving:
                    escape_dir = last_escape_dir
                    escape_fail_count = 0

                else:
                    escape_fail_count += 1

                    if escape_fail_count >= MAX_ESCAPE_FAIL:
                        escape_dir = (1 if gap_angle >= 0 else -1) if gap_valid else -last_escape_dir
                        escape_fail_count = 0
                    else:
                        escape_dir = last_escape_dir

                last_escape_dir = escape_dir
                prev_center_min = center_min
                prev_path_cnt   = path_cnt

                mode = MODE_BACK
                back_count = BACK_CYCLES

                send_backward(BACK_SPEED)

                print(
                    f"STUCK -> BACK "
                    f"front_min={front_min:.0f} "
                    f"front_cnt={front_close_cnt} "
                    f"gap_angle={gap_angle:.1f} "
                    f"gap_score={gap_score:.0f} "
                    f"gap_valid={gap_valid} "
                    f"escape_dir={escape_dir} "
                    f"fail_cnt={escape_fail_count}"
                )

            elif path_blocked or center_blocked_ahead:
                raw_steer = calc_avoid_steer(
                    gap_angle,
                    path_min,
                    open_dir,
                    gap_valid
                )

                real_steer = STEER_SIGN * raw_steer
                dynamic_speed = calc_avoid_speed(path_min)

                send_forward(real_steer, dynamic_speed)

                mode_label = "GAP_AVOID" if gap_valid else "OPEN_AVOID"

                print(
                    f"{mode_label} "
                    f"gap_angle={gap_angle:.1f} "
                    f"gap_score={gap_score:.0f} "
                    f"path_min={path_min:.0f} "
                    f"real_steer={real_steer:.2f} "
                    f"speed={dynamic_speed:.2f}"
                )

            else:
                send_forward(0.0, NORMAL_SPEED)

                print(
                    f"CLEAR -> FORWARD "
                    f"gap_angle={gap_angle:.1f} "
                    f"gap_score={gap_score:.0f} "
                    f"Lopen={left_open_score:.0f} "
                    f"Ropen={right_open_score:.0f}"
                )

        # 다음 스캔 초기화
        scan_buf = []

        path_cnt = 0
        path_min = 9999.0

        front_close_cnt = 0
        front_min = 9999.0

        left_score  = 0.0
        right_score = 0.0
        left_cnt    = 0
        right_cnt   = 0

        left_open_score  = 0.0
        right_open_score = 0.0
        center_block_cnt = 0
        center_min       = 9999.0
