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

ser_L    = serial.Serial(port_L,    baudrate_L,    timeout=1)
ser_Ardu = serial.Serial(port_Ardu, baudrate_Ardu, timeout=1)


# ============================================================
# 2. 파라미터
# ============================================================

MIN_DIST = 80.0

# RC카 크기: 20cm x 20cm, 라이다 정중앙
CAR_HALF_WIDTH = 100.0
SAFETY_MARGIN  = 20.0
PATH_HALF_WIDTH = CAR_HALF_WIDTH + SAFETY_MARGIN   # 120mm

# 정면 경로 검사
PATH_CHECK_DIST  = 550.0
PATH_DANGER_DIST = 520.0
PATH_BLOCK_POINTS = 3

# 넓은 전방 열린 길 판단
LOOKAHEAD_DIST    = 600.0
OPEN_SIDE_WIDTH   = 260.0
CENTER_WIDTH      = PATH_HALF_WIDTH
CENTER_BLOCK_DIST = 560.0
CENTER_BLOCK_POINTS = 3

# 좌우 여유공간 검사
SIDE_CHECK_DIST    = 260.0
SIDE_MAX_SCORE_DIST = 320.0

# 진짜 막힘 판단
STUCK_FRONT_DIST = 200.0
STUCK_POINTS     = 5

# 속도
NORMAL_SPEED   = 0.52
# [수정3] 회피 속도를 거리 비례로 동적 계산 (고정값 제거)
AVOID_SPEED_MAX = 0.30   # 장애물이 멀 때
AVOID_SPEED_MIN = 0.15   # 장애물이 매우 가까울 때
BACK_SPEED     = 0.35
ESCAPE_SPEED   = 0.22

# 조향
AVOID_STEER  = 0.60
ESCAPE_STEER = 0.60

# 조향 방향 보정
STEER_SIGN = -1

# 조향 smoothing
SMOOTH     = 0.20
prev_steer = 0.0

# 상태
MODE_NORMAL = 0
MODE_BACK   = 1
MODE_ESCAPE = 2

mode = MODE_NORMAL

back_count   = 0
escape_count = 0
escape_dir   = 0

BACK_CYCLES   = 4
ESCAPE_CYCLES = 5

last_escape_dir   = 0
escape_fail_count = 0

prev_center_min = 0.0
prev_path_cnt   = 999

MAX_ESCAPE_FAIL = 3
IMPROVE_DIST    = 50.0

# ============================================================
# Gap Finding 파라미터
# ============================================================

GAP_MAX_VALID_DIST   = 1500.0
GAP_MIN_ANGLE_DIFF   = 3.0
GAP_MIN_DIST_JUMP    = 150.0
GAP_WALL_FAR_THRESH  = 1200.0
GAP_WALL_NEAR_THRESH = 600.0
GAP_MIN_PASS_WIDTH   = CAR_HALF_WIDTH * 2 + 40   # 240mm
GAP_MIN_SCORE        = 200.0


# ============================================================
# 3. 종료 처리
# ============================================================

def cleanup():
    try:
        ser_Ardu.write(b"S\n")
        ser_L.write(bytes([0xA5, 0x25]))
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


def send_forward_direct(steer, speed):
    """ESCAPE 전용. smoothing 없이 바로 조향."""
    global prev_steer

    steer = clamp(steer, -0.60, 0.60)
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
    a     = normalize_angle(angle)
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


# ============================================================
# [수정3] 거리 비례 회피 속도 계산
# ============================================================

def calc_avoid_speed(path_min):
    """
    path_min이 가까울수록 느리게, 멀수록 빠르게.
    PATH_DANGER_DIST 기준으로 선형 보간.
    """
    ratio = clamp(path_min / PATH_DANGER_DIST, 0.0, 1.0)
    return AVOID_SPEED_MIN + (AVOID_SPEED_MAX - AVOID_SPEED_MIN) * ratio


# ============================================================
# Gap Finding 함수
# ============================================================

def find_best_gap(scan_buf):
    """
    전방 스캔 데이터에서 통과 가능한 최적 gap을 찾아
    (gap_angle, gap_score) 를 반환한다.

    gap_angle : 음수=왼쪽, 양수=오른쪽 (도 단위)
    gap_score : 클수록 좋은 gap. GAP_MIN_SCORE 미만이면 유효 gap 없음.

    필터 순서:
      1) 거리 상한          → 벽 끝 가짜 gap 제거
      2) 각도 간격          → 누락 포인트 노이즈 제거
      3) 거리 차이          → 미세 노이즈 제거
      4) 벽 끝 패턴 감지   → 벽 근처 오탐 제거
      5) 실제 통과 폭       → 차폭보다 좁은 gap 제거
    """
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
        far_dist   = max(d1, d2)
        near_dist  = min(d1, d2)

        # 필터 1: 각도 간격
        if angle_diff < GAP_MIN_ANGLE_DIFF:
            continue

        # 필터 2: 거리 차이
        if dist_jump < GAP_MIN_DIST_JUMP:
            continue

        # 필터 3: 벽 끝 패턴
        if far_dist > GAP_WALL_FAR_THRESH and near_dist < GAP_WALL_NEAR_THRESH:
            continue

        # 필터 4: 실제 통과 폭 (코사인 법칙)
        theta = math.radians(angle_diff)
        physical_width = math.sqrt(
            d1 ** 2 + d2 ** 2 - 2.0 * d1 * d2 * math.cos(theta)
        )
        if physical_width < GAP_MIN_PASS_WIDTH:
            continue

        # 점수: 통과 폭 클수록, 정면에 가까울수록 높음
        gap_angle     = (a1 + a2) / 2.0
        angle_penalty = abs(gap_angle) / 90.0
        score         = physical_width * (1.0 - 0.5 * angle_penalty)

        gaps.append((gap_angle, score))

    if not gaps:
        return 0.0, 0.0

    best = max(gaps, key=lambda g: g[1])
    return best[0], best[1]


# ============================================================
# [수정2] 거리 기반 조향 강도 계산
# ============================================================

def calc_avoid_steer(gap_angle, path_min, open_dir, gap_valid):
    """
    [수정2] gap_angle은 방향 결정에만 사용.
    조향 강도는 path_min(장애물까지 거리)에 반비례.

    path_min이 가까울수록 강하게 꺾고,
    멀수록 부드럽게 꺾는다.

    반환: real_steer (STEER_SIGN 적용 전)
    """
    if gap_valid:
        gap_dir = 1 if gap_angle >= 0 else -1
    else:
        gap_dir = open_dir

    # 거리 비례 강도: 가까울수록 1.0, 멀수록 0.4
    intensity = 1.0 - clamp(path_min / PATH_DANGER_DIST, 0.0, 1.0)
    intensity = clamp(intensity + 0.4, 0.4, 1.0)

    steer = gap_dir * AVOID_STEER * intensity
    return clamp(steer, -0.60, 0.60)


# ============================================================
# 5. LiDAR 시작
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
print("최종 통합 LiDAR 주행 시작")
print("수정: ① BACK완료 시 gap 재계산  ② 거리기반 조향강도  ③ 거리비례 속도")
print(f"PATH_HALF_WIDTH    = {PATH_HALF_WIDTH:.0f} mm")
print(f"PATH_DANGER_DIST   = {PATH_DANGER_DIST:.0f} mm")
print(f"STUCK_FRONT_DIST   = {STUCK_FRONT_DIST:.0f} mm")
print(f"GAP_MIN_PASS_WIDTH = {GAP_MIN_PASS_WIDTH:.0f} mm")
print(f"AVOID_SPEED        = {AVOID_SPEED_MIN:.2f} ~ {AVOID_SPEED_MAX:.2f}")
print("=" * 60)


# ============================================================
# 6. 스캔 누적 변수
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
# 7. 메인 루프
# ============================================================

while True:
    data = ser_L.read(5)

    if len(data) != 5:
        continue

    # --------------------------------------------------------
    # 패킷 검증
    # --------------------------------------------------------
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

    # --------------------------------------------------------
    # 각도, 거리 계산
    # --------------------------------------------------------
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    angle       = angle_q6 / 64.0

    distance_q2 = data[3] | (data[4] << 8)
    distance    = distance_q2 / 4.0

    if distance < MIN_DIST:
        continue

    scan_buf.append((angle, distance))

    x, y = polar_to_xy(angle, distance)

    # --------------------------------------------------------
    # 1) 정면 차폭 경로 검사
    # --------------------------------------------------------
    if 0 < x < PATH_CHECK_DIST and abs(y) < PATH_HALF_WIDTH:
        path_cnt += 1
        path_min  = min(path_min, x)

    # --------------------------------------------------------
    # 2) 진짜 막힘 판단
    # --------------------------------------------------------
    if 0 < x < STUCK_FRONT_DIST and abs(y) < PATH_HALF_WIDTH:
        front_close_cnt += 1
        front_min        = min(front_min, x)

    # --------------------------------------------------------
    # 3) 좌우 여유공간 점수 계산
    # --------------------------------------------------------
    if 0 < x < SIDE_CHECK_DIST:
        sc = min(distance, SIDE_MAX_SCORE_DIST)
        if y < -PATH_HALF_WIDTH:
            left_score += sc
            left_cnt   += 1
        elif y > PATH_HALF_WIDTH:
            right_score += sc
            right_cnt   += 1

    # --------------------------------------------------------
    # 4) 넓은 전방 열린 길 판단
    # --------------------------------------------------------
    if 0 < x < LOOKAHEAD_DIST:
        if abs(y) < CENTER_WIDTH:
            center_block_cnt += 1
            center_min        = min(center_min, x)
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
            path_min  < PATH_DANGER_DIST
        )
        center_blocked_ahead = (
            center_block_cnt >= CENTER_BLOCK_POINTS and
            center_min        < CENTER_BLOCK_DIST
        )
        stuck = (
            front_close_cnt >= STUCK_POINTS and
            front_min        < STUCK_FRONT_DIST
        )

        open_dir = choose_open_dir(
            left_open_score, right_open_score,
            left_score,      right_score
        )

        # Gap Finding (매 스캔마다 계산)
        gap_angle, gap_score = find_best_gap(scan_buf)
        gap_valid = gap_score >= GAP_MIN_SCORE

        # ====================================================
        # 상태 기반 판단
        # ====================================================

        # ----------------------------------------------------
        # BACK 모드
        # ----------------------------------------------------
        if mode == MODE_BACK:
            send_backward(BACK_SPEED)
            back_count -= 1

            print(
                f"BACK remain={back_count} "
                f"escape_dir={escape_dir} "
                f"gap_angle={gap_angle:.1f} gap_valid={gap_valid}"
            )

            if back_count <= 0:
                # ★ [수정1] 후진 완료 시점에 gap 재계산 → escape_dir 갱신
                fresh_gap_angle, fresh_gap_score = find_best_gap(scan_buf)
                fresh_gap_valid = fresh_gap_score >= GAP_MIN_SCORE

                if fresh_gap_valid:
                    escape_dir = 1 if fresh_gap_angle >= 0 else -1
                    print(
                        f"BACK완료: gap 재계산 → escape_dir={escape_dir} "
                        f"fresh_gap_angle={fresh_gap_angle:.1f} "
                        f"fresh_gap_score={fresh_gap_score:.0f}"
                    )
                else:
                    print(
                        f"BACK완료: gap 없음 → escape_dir={escape_dir} 유지"
                    )

                mode         = MODE_ESCAPE
                escape_count = ESCAPE_CYCLES

        # ----------------------------------------------------
        # ESCAPE 모드
        # ----------------------------------------------------
        elif mode == MODE_ESCAPE:
            real_steer = STEER_SIGN * ESCAPE_STEER * escape_dir

            send_forward_direct(real_steer, ESCAPE_SPEED)
            escape_count -= 1

            print(
                f"SHORT_ESCAPE "
                f"dir={escape_dir} "
                f"real_steer={real_steer:.2f} "
                f"remain={escape_count} "
                f"center_min={center_min:.0f} "
                f"path_cnt={path_cnt}"
            )

            if escape_count <= 0:
                mode = MODE_NORMAL

        # ----------------------------------------------------
        # NORMAL 모드
        # ----------------------------------------------------
        else:
            if stuck:
                escape_improving = (
                    center_min > prev_center_min + IMPROVE_DIST or
                    path_cnt   < prev_path_cnt
                )

                if last_escape_dir == 0:
                    escape_dir = (1 if gap_angle >= 0 else -1) if gap_valid else open_dir

                elif escape_improving:
                    escape_dir        = last_escape_dir
                    escape_fail_count = 0

                else:
                    escape_fail_count += 1
                    if escape_fail_count >= MAX_ESCAPE_FAIL:
                        escape_dir        = (1 if gap_angle >= 0 else -1) if gap_valid else -last_escape_dir
                        escape_fail_count = 0
                    else:
                        escape_dir = last_escape_dir

                last_escape_dir = escape_dir
                prev_center_min = center_min
                prev_path_cnt   = path_cnt

                mode       = MODE_BACK
                back_count = BACK_CYCLES

                send_backward(BACK_SPEED)

                print(
                    f"STUCK → BACK "
                    f"front_min={front_min:.0f} "
                    f"front_cnt={front_close_cnt} "
                    f"center_min={center_min:.0f} "
                    f"gap_angle={gap_angle:.1f} "
                    f"gap_valid={gap_valid} "
                    f"escape_dir={escape_dir} "
                    f"fail_cnt={escape_fail_count} "
                    f"improving={escape_improving}"
                )

            elif path_blocked or center_blocked_ahead:
                # ★ [수정2] 방향은 gap, 강도는 거리 기반
                raw_steer  = calc_avoid_steer(gap_angle, path_min, open_dir, gap_valid)
                real_steer = STEER_SIGN * raw_steer

                # ★ [수정3] 거리 비례 속도
                dynamic_speed = calc_avoid_speed(path_min)

                send_forward(real_steer, dynamic_speed)

                mode_label = "GAP_AVOID" if gap_valid else "OPEN_AVOID(fallback)"
                print(
                    f"{mode_label} "
                    f"gap_angle={gap_angle:.1f} "
                    f"gap_score={gap_score:.0f} "
                    f"path_min={path_min:.0f} "
                    f"raw_steer={raw_steer:.2f} "
                    f"real_steer={real_steer:.2f} "
                    f"speed={dynamic_speed:.2f} "
                    f"center_min={center_min:.0f}"
                )

            else:
                send_forward(0.0, NORMAL_SPEED)

                print(
                    f"CLEAR → FORWARD "
                    f"gap_angle={gap_angle:.1f} "
                    f"gap_score={gap_score:.0f} "
                    f"Lopen={left_open_score:.0f} "
                    f"Ropen={right_open_score:.0f}"
                )

        # ----------------------------------------------------
        # 다음 스캔 초기화
        # ----------------------------------------------------
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
