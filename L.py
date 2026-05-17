import serial
import time
import math
import atexit

# ── 포트 설정 ─────────────────────────────
port_L = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L = serial.Serial(port_L, 460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)

# ── LiDAR 시작 ─────────────────────────────
ser_L.write(bytes([0xA5, 0x40]))   # RESET
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))   # SCAN

# ── VFH 파라미터 ───────────────────────────
BIN_DEG = 5.0
N_BINS = int(360 / BIN_DEG)

ROBOT_WIDTH = 125.0       # 실제 폭 X, 갭 판단용 유효 폭
GAP_MARGIN = 10.0
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN   # 135mm

DETECT = 500.0
EMERGENCY = 150.0

MAX_STEER = 0.85

# 옆쪽 갭을 무리하게 전진으로 따라가지 않도록 낮춤
ROT_THRESH = 85.0

# ── 속도 파라미터 ──────────────────────────
BASE_SPEED = 0.75
MIN_SPEED = 0.45
OPEN_SPEED = 0.80

# ── NO_GAP 탈출 파라미터 ───────────────────
# 후진은 조금 더 충분히, 조향 전진은 조금 짧게
BACK_CYCLES = 5
ESCAPE_CYCLES = 4

BACK_SPEED = 0.40
ESCAPE_SPEED = 0.45

# 0.90은 거의 제자리급이라 0.75로 완화
ESCAPE_STEER = 0.75

# ── 아두이노 timeout 방지용 재전송 ──────────
last_cmd = b"S\n"
last_send_time = time.time()
RESEND_INTERVAL = 0.20


def send_cmd(cmd):
    global last_cmd, last_send_time

    if isinstance(cmd, str):
        cmd = cmd.encode()

    ser_Ardu.write(cmd)
    last_cmd = cmd
    last_send_time = time.time()


def resend_last_cmd_if_needed():
    global last_send_time

    now = time.time()
    if now - last_send_time > RESEND_INTERVAL:
        ser_Ardu.write(last_cmd)
        last_send_time = now


def cleanup():
    try:
        send_cmd(b"S\n")
        ser_L.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        ser_L.close()
        ser_Ardu.close()
    except Exception:
        pass


atexit.register(cleanup)


def build_polar_hist(scan_buf):
    hist = [9999.0] * N_BINS
    has_pt = [False] * N_BINS

    for a, d in scan_buf:
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True

    return hist, has_pt


def find_vfh_gaps(hist, has_pt, detect_dist, min_pass_mm):
    blocked = [has_pt[i] and hist[i] <= detect_dist for i in range(N_BINS)]

    smoothed = blocked[:]
    for i in range(N_BINS):
        if blocked[i] and not blocked[(i - 1) % N_BINS] and not blocked[(i + 1) % N_BINS]:
            smoothed[i] = False
    blocked = smoothed

    gaps = []
    seen = set()
    i = 0

    while i < 2 * N_BINS:
        bi = i % N_BINS

        if not blocked[bi]:
            j = i + 1
            while j < i + N_BINS and not blocked[j % N_BINS]:
                j += 1

            span = j - i

            if span < N_BINS:
                center_cw = ((i + j) / 2.0 * BIN_DEG) % 360.0
                ck = round(center_cw)

                if ck not in seen:
                    seen.add(ck)

                    delta_deg = span * BIN_DEG

                    d_L = hist[(i - 1) % N_BINS] if has_pt[(i - 1) % N_BINS] else detect_dist
                    d_R = hist[j % N_BINS] if has_pt[j % N_BINS] else detect_dist

                    d_L = min(d_L, detect_dist)
                    d_R = min(d_R, detect_dist)

                    gap_w = (d_L + d_R) * math.sin(math.radians(delta_deg / 2.0))
                    center_s = center_cw if center_cw <= 180.0 else center_cw - 360.0

                    gaps.append({
                        "center": center_s,
                        "center_cw": center_cw,
                        "width": gap_w,
                        "passable": gap_w >= min_pass_mm,
                        "delta_deg": delta_deg,
                        "d_L": d_L,
                        "d_R": d_R,
                    })

            i = j

        else:
            i += 1

    return gaps


def select_best_gap(gaps, min_pass_mm):
    if not gaps:
        return None

    passable = [g for g in gaps if g["width"] >= min_pass_mm]
    pool = passable if passable else gaps

    return max(pool, key=lambda g: g["width"] * 0.25 - abs(g["center"]) * 1.9)


def nearest_in_arc(hist, has_pt, center_cw, arc_half=25):
    center_bin = int(center_cw / BIN_DEG) % N_BINS
    n_check = max(1, int(arc_half / BIN_DEG))

    min_d = 9999.0

    for k in range(-n_check, n_check + 1):
        idx = (center_bin + k) % N_BINS
        if has_pt[idx] and hist[idx] < min_d:
            min_d = hist[idx]

    return min_d


# ── 상태 변수 ─────────────────────────────
scan_buf = []

back_cnt = 0
escape_cnt = 0
escape_dir = 1.0

print("=" * 65)
print(" VFH 장애물 회피 코드")
print(" Arduino command: F steer speed / B speed / S")
print(" 마지막 명령 자동 재전송 적용")
print(f" DETECT={DETECT:.0f}mm, GAP_MIN_PASS={GAP_MIN_PASS:.0f}mm")
print(f" ROT_THRESH={ROT_THRESH:.0f}deg")
print(f" BACK={BACK_CYCLES}, ESCAPE={ESCAPE_CYCLES}, ESCAPE_STEER={ESCAPE_STEER:.2f}")
print("=" * 65)


while True:
    resend_last_cmd_if_needed()

    data = ser_L.read(5)

    resend_last_cmd_if_needed()

    if len(data) != 5:
        continue

    # ── RPLIDAR 패킷 유효성 검사 ─────────────
    s_flag = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1

    if s_inv_flag != (1 - s_flag):
        continue

    if (data[1] & 0x01) != 1:
        continue

    quality = data[0] >> 2
    angle = ((data[1] >> 1) | (data[2] << 7)) / 64.0
    distance = (data[3] | (data[4] << 8)) / 4.0

    if quality == 0:
        continue

    if distance < 80:
        continue

    scan_buf.append((angle, distance))

    # ── 한 바퀴 스캔 완료 ───────────────────
    if s_flag == 1:
        hist, has_pt = build_polar_hist(scan_buf)

        if not any(has_pt):
            send_cmd(f"F 0.00 {OPEN_SPEED:.2f}\n")
            back_cnt = 0
            escape_cnt = 0
            print(f"OPEN  F 0.00 {OPEN_SPEED:.2f}")

        else:
            gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
            best = select_best_gap(gaps, GAP_MIN_PASS)

            # ── 정상 전진 회피 ─────────────
            if best is not None and best["passable"] and abs(best["center"]) <= ROT_THRESH:
                d_L = best["d_L"]
                d_R = best["d_R"]

                imbalance = (d_R - d_L) / (d_L + d_R + 1e-9)
                bias = imbalance * (best["delta_deg"] / 3.0)

                target = best["center"] + bias
                steer = target / 90.0 * MAX_STEER
                steer = max(-MAX_STEER, min(MAX_STEER, steer))

                near_d = nearest_in_arc(hist, has_pt, best["center_cw"], arc_half=30)

                ratio = (DETECT - near_d) / (DETECT - EMERGENCY + 5)
                ratio = min(max(ratio, 0.0), 1.0)

                speed = BASE_SPEED * (1.0 - ratio * 0.35)
                speed = max(MIN_SPEED, min(BASE_SPEED, speed))

                send_cmd(f"F {steer:.2f} {speed:.2f}\n")

                back_cnt = 0
                escape_cnt = 0

                print(
                    f"VFH_FWD  gap={best['width']:.0f}mm "
                    f"center={best['center']:+.0f}deg "
                    f"steer={steer:+.2f} speed={speed:.2f} near={near_d:.0f}mm"
                )

            # ── 후진 탈출 ──────────────
            else:
                if gaps:
                    open_g = max(gaps, key=lambda g: g["width"])
                    escape_dir = 1.0 if open_g["center"] > 0 else -1.0
                    widest = open_g["width"]
                    target_dir = open_g["center"]

                else:
                    left_d = nearest_in_arc(hist, has_pt, 315.0, arc_half=35)
                    right_d = nearest_in_arc(hist, has_pt, 45.0, arc_half=35)

                    escape_dir = 1.0 if right_d > left_d else -1.0
                    widest = 0.0
                    target_dir = 45.0 if escape_dir > 0 else -45.0

                if back_cnt < BACK_CYCLES:
                    send_cmd(f"B {BACK_SPEED:.2f}\n")
                    back_cnt += 1
                    escape_cnt = 0

                    print(
                        f"NO_GAP_BACK  {back_cnt}/{BACK_CYCLES} "
                        f"widest={widest:.0f}mm dir={escape_dir:+.0f}"
                    )

                elif escape_cnt < ESCAPE_CYCLES:
                    steer = escape_dir * ESCAPE_STEER
                    send_cmd(f"F {steer:.2f} {ESCAPE_SPEED:.2f}\n")
                    escape_cnt += 1

                    print(
                        f"NO_GAP_ESCAPE  {escape_cnt}/{ESCAPE_CYCLES} "
                        f"steer={steer:+.2f} target={target_dir:+.0f}deg"
                    )

                else:
                    back_cnt = 0
                    escape_cnt = 0
                    print("NO_GAP_RESET")

        scan_buf = []
