import serial
import time
import math
import atexit

# ?? ?ы듃 ?ㅼ젙 ?????????????????????????????
port_L = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L = serial.Serial(port_L, 460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)

# ?? LiDAR ?쒖옉 ?????????????????????????????
ser_L.write(bytes([0xA5, 0x40]))   # RESET
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))   # SCAN

# ?? VFH ?뚮씪誘명꽣 ???????????????????????????
BIN_DEG = 5.0
N_BINS = int(360 / BIN_DEG)

ROBOT_WIDTH = 125.0       # ?ㅼ젣 ??X, 媛??먮떒???좏슚 ??GAP_MARGIN = 10.0
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN   # 135mm

DETECT = 500.0
EMERGENCY = 150.0

MAX_STEER = 0.85
ROT_THRESH = 115.0

# ?? ?꾨갑 異⑸룎 ?꾪뿕 議곌굔 ?????????????????????
# RC移?紐⑥꽌由?異⑸룎源뚯? 怨좊젮?댁꽌 짹35???ъ슜
FRONT_RISK_ARC = 35.0
FRONT_RISK_DIST = 220.0

# ?? ?띾룄 ?뚮씪誘명꽣 ??????????????????????????
BASE_SPEED = 0.75
MIN_SPEED = 0.45
OPEN_SPEED = 0.80

# ?? ?꾩쭊/?덉텧 ?뚮씪誘명꽣 ?????????????????????
BACK_CYCLES = 4
ESCAPE_CYCLES = 5
BACK_SPEED = 0.40
ESCAPE_SPEED = 0.45
ESCAPE_STEER = 0.90

# ?? ?꾨몢?대끂 timeout 諛⑹????ъ쟾????????????
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
        ser_L.write(bytes([0xA5, 0x25]))  # STOP
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


def nearest_in_arc(hist, has_pt, center_cw, arc_half=25):
    center_bin = int(center_cw / BIN_DEG) % N_BINS
    n_check = max(1, int(arc_half / BIN_DEG))

    min_d = 9999.0

    for k in range(-n_check, n_check + 1):
        idx = (center_bin + k) % N_BINS
        if has_pt[idx] and hist[idx] < min_d:
            min_d = hist[idx]

    return min_d


def front_collision_risk(hist, has_pt):
    """
    ?꾨갑 짹35???덉뿉??220mm ?댄븯 ?μ븷臾쇱씠 ?덉쑝硫?    媛??먮떒 ?깃났 ?щ?? ?곴??놁씠 ?꾩쭊 ?덉텧.
    """
    front_near = nearest_in_arc(hist, has_pt, 0.0, arc_half=FRONT_RISK_ARC)

    if front_near <= FRONT_RISK_DIST:
        return True, front_near

    return False, front_near


def find_vfh_gaps(hist, has_pt, detect_dist, min_pass_mm):
    blocked = [has_pt[i] and hist[i] <= detect_dist for i in range(N_BINS)]

    # ?⑥씪 ?몄씠利??쒓굅
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


# ?? ?곹깭 蹂???????????????????????????????
scan_buf = []

back_cnt = 0
escape_cnt = 0
escape_dir = 1.0

print("=" * 65)
print(" VFH ?μ븷臾??뚰뵾 理쒖쥌 肄붾뱶")
print(" Arduino command: F steer speed / B speed / S")
print(" ?꾨갑 짹35??異⑸룎 ?꾪뿕 寃???ы븿")
print(f" GAP_MIN_PASS={GAP_MIN_PASS:.0f}mm")
print(f" FRONT_RISK={FRONT_RISK_DIST:.0f}mm, 짹{FRONT_RISK_ARC:.0f}deg")
print("=" * 65)


while True:
    # ?꾨몢?대끂 timeout 諛⑹?
    resend_last_cmd_if_needed()

    data = ser_L.read(5)

    # read 以??쒓컙??吏?ъ쓣 ???덉쑝誘濡??ъ쟾??    resend_last_cmd_if_needed()

    if len(data) != 5:
        continue

    # ?? RPLIDAR ?⑦궥 ?좏슚??寃???????????????
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

    # ?? ??諛뷀??ㅼ틪 ?꾨즺 ???????????????????
    if s_flag == 1:
        hist, has_pt = build_polar_hist(scan_buf)

        # ?꾨Т寃껊룄 ??蹂댁씠硫?鍮좊Ⅴ寃?吏곸쭊
        if not any(has_pt):
            send_cmd(f"F 0.00 {OPEN_SPEED:.2f}\n")
            back_cnt = 0
            escape_cnt = 0
            print(f"OPEN  F 0.00 {OPEN_SPEED:.2f}")

        else:
            gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
            best = select_best_gap(gaps, GAP_MIN_PASS)

            front_risk, front_d = front_collision_risk(hist, has_pt)

            # ?? ?뺤긽 VFH ?꾩쭊 議곌굔 ???????????
            # ?꾨갑 異⑸룎 ?꾪뿕???덉쑝硫?媛??먮떒 ?깃났?댁뼱???꾩쭊?섏? ?딆쓬
            if (
                not front_risk
                and best is not None
                and best["passable"]
                and abs(best["center"]) <= ROT_THRESH
            ):
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
                    f"steer={steer:+.2f} speed={speed:.2f} "
                    f"front={front_d:.0f}mm"
                )

            # ?? ?꾩쭊 ?덉텧 議곌굔 ???????????????
            # 1. ?꾨갑 짹35???덉뿉 媛源뚯슫 ?μ븷臾??덉쓬
            # 2. ?듦낵 媛?ν븳 媛??놁쓬
            # 3. 媛?씠 ?덈Т ???ㅼそ??            else:
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
                        f"BACK  {back_cnt}/{BACK_CYCLES} "
                        f"front={front_d:.0f}mm "
                        f"widest={widest:.0f}mm dir={escape_dir:+.0f}"
                    )

                elif escape_cnt < ESCAPE_CYCLES:
                    steer = escape_dir * ESCAPE_STEER
                    send_cmd(f"F {steer:.2f} {ESCAPE_SPEED:.2f}\n")
                    escape_cnt += 1

                    print(
                        f"ESCAPE  {escape_cnt}/{ESCAPE_CYCLES} "
                        f"steer={steer:+.2f} "
                        f"target={target_dir:+.0f}deg front={front_d:.0f}mm"
                    )

                else:
                    back_cnt = 0
                    escape_cnt = 0
                    print("ESCAPE_RESET")

        scan_buf = []
