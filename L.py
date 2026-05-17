import serial
import time
import math
import atexit

port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L, 460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 115200, timeout=1)

# ── LiDAR 시작 ─────────────────────────────
ser_L.write(bytes([0xA5, 0x40]))   # RESET
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))   # SCAN

# ── 파라미터 ───────────────────────────────
BIN_DEG      = 5.0
N_BINS       = int(360 / BIN_DEG)

ROBOT_WIDTH  = 140.0     # 실제 폭이 아니라 갭 판단용 유효 폭
GAP_MARGIN   = 10.0
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN   # 150mm

DETECT       = 500.0
EMERGENCY    = 150.0

MAX_STEER    = 0.85
ROT_THRESH   = 110.0

# NO_GAP 탈출 파라미터
BACK_CYCLES   = 5
ESCAPE_CYCLES = 6
BACK_SPEED    = 0.35
ESCAPE_SPEED  = 0.35
ESCAPE_STEER  = 0.85


# ── 종료 처리 ─────────────────────────────
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


# ── VFH 함수들 ─────────────────────────────
def build_polar_hist(scan_buf):
    hist   = [9999.0] * N_BINS
    has_pt = [False]  * N_BINS

    for a, d in scan_buf:
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True

    return hist, has_pt


def find_vfh_gaps(hist, has_pt, detect_dist, min_pass_mm):
    blocked = [has_pt[i] and hist[i] <= detect_dist for i in range(N_BINS)]

    # 단일 노이즈 제거
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


def select_best_gap(gaps, min_pass_mm=GAP_MIN_PASS):
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
print(" T 명령 없음 / F, B, S 명령만 사용")
print(f" 감지거리: {DETECT:.0f}mm / 긴급거리: {EMERGENCY:.0f}mm")
print(f" 갭 판단 기준: {GAP_MIN_PASS:.0f}mm")
print("=" * 65)


# ── 메인 루프 ─────────────────────────────
while True:
    data = ser_L.read(5)

    if len(data) != 5:
        continue

    # 패킷 유효성 검사
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

    # 한 바퀴 스캔 완료
    if s_flag == 1:
        hist, has_pt = build_polar_hist(scan_buf)

        if not any(has_pt):
            ser_Ardu.write(b"F 0.00 0.70\n")
            back_cnt = 0
            escape_cnt = 0
            print("OPEN  F 0.00 0.70")

        else:
            gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
            best = select_best_gap(gaps, GAP_MIN_PASS)

            # ── P3: 통과 가능한 갭이 전방 쪽에 있음 ──
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

                speed = 0.60 * (1.0 - ratio * 0.55)

                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())

                back_cnt = 0
                escape_cnt = 0

                print(
                    f"VFH_FWD  gap={best['width']:.0f}mm "
                    f"center={best['center']:+.0f}deg "
                    f"steer={steer:+.2f} speed={speed:.2f} near={near_d:.0f}mm"
                )

            # ── P5: 통과 가능 갭 없음 → 후진 + 강한 조향 전진 ──
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
                    ser_Ardu.write(f"B {BACK_SPEED:.2f}\n".encode())
                    back_cnt += 1
                    escape_cnt = 0

                    print(
                        f"NO_GAP_BACK  {back_cnt}/{BACK_CYCLES} "
                        f"widest={widest:.0f}mm dir={escape_dir:+.0f}"
                    )

                elif escape_cnt < ESCAPE_CYCLES:
                    steer = escape_dir * ESCAPE_STEER

                    ser_Ardu.write(f"F {steer:.2f} {ESCAPE_SPEED:.2f}\n".encode())
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
