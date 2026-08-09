#!/usr/bin/env python3
"""
parse_dm_log.py — Parse DM motor / CAN / serial log files and extract diagnostic facts.

Handles several common formats:
  1. Hex feedback frames, e.g.  "AA 12 11 00 8A 3D 88 80 A4 0F 68 66 55"  (0xAA..0x55 16-byte frames)
  2. Text lines with key=value fields, e.g. "ERR=5", "position=1.23", "torque=2.5", "T_MOS=85"
  3. Raw DM_CAN.py-style prints ("controlMIT ERROR : Motor ID not found", "切换模式失败")

Usage:
  python parse_dm_log.py <logfile> [--limits PMAX,VMAX,TMAX] [--gear GEAR]

Output: a summary of extracted facts (error codes, extremes of position/velocity/
torque/temperature, frame counts) to help the diagnostic workflow.
"""
import re
import sys
from collections import Counter

# DM feedback frame ERR code meanings (from DM-J4310-2EC manual)
ERR_MEANING = {
    0x0: "失能 (disabled)",
    0x1: "使能 (enabled)",
    0x3: "输出轴校准异常 (output-axis calibration fault)",
    0x4: "传感器输出异常 (sensor output fault)",
    0x5: "电机编码器校准异常 (motor encoder calibration fault)",
    0x8: "超压 (over-voltage)",
    0x9: "欠压 (under-voltage)",
    0xA: "过电流 (over-current)",
    0xB: "MOS 过温 (MOS over-temperature)",
    0xC: "电机线圈过温 (coil over-temperature)",
    0xD: "通讯丢失 (communication loss)",
    0xE: "过载 (overload)",
}

# Default mapping ranges for DM4310 (rad, rad/s, Nm)
DEFAULT_LIMITS = (12.5, 30.0, 10.0)


def u2f(x, x_min, x_max, bits):
    """uint -> float, matching DM uint_to_float."""
    span = x_max - x_min
    return float(x) / ((1 << bits) - 1) * span + x_min


def extract_hex_bytes(text):
    """Pull runs of hex bytes out of a line (space or whitespace separated)."""
    return re.findall(r'\b[0-9A-Fa-f]{2}\b', text)


def parse_feedback_frame(frame):
    """Parse an 8-byte CAN payload into a dict of motor feedback facts.
    Frame format (from manual): MST_ID|ID|ERR<<4, POS[15:8], POS[7:0],
    VEL[11:4], VEL[3:0]|T[11:8], T[7:0], T_MOS, T_Rotor."""
    if len(frame) != 8:
        return None
    d0 = frame[0]
    master_id = d0 & 0x0f
    err = (d0 >> 4) & 0x0f
    pos = u2f((frame[1] << 8) | frame[2], -DEFAULT_LIMITS[0], DEFAULT_LIMITS[0], 16)
    vel = u2f(((frame[3] << 4) | (frame[4] >> 4)), -DEFAULT_LIMITS[1], DEFAULT_LIMITS[1], 12)
    tau = u2f(((frame[4] & 0x0f) << 8) | frame[5], -DEFAULT_LIMITS[2], DEFAULT_LIMITS[2], 12)
    return {
        "master_id": master_id,
        "err": err,
        "pos": pos,
        "vel": vel,
        "tau": tau,
        "t_mos": frame[6],
        "t_rotor": frame[7],
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    path = sys.argv[1]

    limits = DEFAULT_LIMITS
    for a in sys.argv[2:]:
        if a.startswith("--limits"):
            limits = tuple(float(x) for x in a.split("=")[1].split(","))
        elif a.startswith("--gear"):
            pass  # gear used for reporting only; not applied to decode

    frames = []
    err_counter = Counter()
    kv = {}          # last-seen key=value fields across text lines
    kv_series = {}   # numeric series per key
    text_hits = []

    with open(path, errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # Text key=value extraction (ERR=..., position=..., T_MOS=..., etc.)
            for m in re.finditer(r'\b([A-Za-z_]+)\s*[:=]\s*(-?[0-9]+(?:\.[0-9]+)?)', line):
                k, v = m.group(1).lower(), float(m.group(2))
                kv[k] = v
                kv_series.setdefault(k, []).append(v)

            # Error-code mentions like "ERR=A", "ERR 0x5", "过流"
            em = re.search(r'(?:ERR|错误|error)\s*[:=]?\s*0x?([0-9A-Fa-f]{1,2})', line)
            if em:
                code = int(em.group(1), 16)
                err_counter[code] += 1
            if "通讯丢失" in line or "communication" in line.lower():
                text_hits.append("通讯丢失 communication loss")

            # Hex frame detection: locate the 0xAA .. 0x55 16-byte frame within the
            # line. Timestamp digits (e.g. "12:01:01") can look like hex, so scan for
            # the AA header explicitly and require a 55 tail 15 tokens later.
            hexes = extract_hex_bytes(line)
            for i, h in enumerate(hexes):
                if h.upper() != "AA":
                    continue
                if i + 15 < len(hexes) and hexes[i + 15].upper() == "55":
                    frame = [int(x, 16) for x in hexes[i:i + 16]]
                    # payload is bytes [7:15] of the 16-byte frame
                    payload = frame[7:15]
                    if len(payload) == 8:
                        fact = parse_feedback_frame(payload)
                        if fact:
                            frames.append(fact)
                            err_counter[fact["err"]] += 1
                    break

    # ---- Report ----
    print(f"=== 解析结果: {path} ===")
    print(f"识别到的反馈帧数量: {len(frames)}")
    if frames:
        pos = [fr["pos"] for fr in frames]
        vel = [fr["vel"] for fr in frames]
        tau = [fr["tau"] for fr in frames]
        t_mos = [fr["t_mos"] for fr in frames]
        t_rot = [fr["t_rotor"] for fr in frames]
        print(f"位置 range: [{min(pos):.4f}, {max(pos):.4f}] rad")
        print(f"速度 range: [{min(vel):.3f}, {max(vel):.3f}] rad/s")
        print(f"力矩 range: [{min(tau):.3f}, {max(tau):.3f}] Nm  (TMAX={limits[2]})")
        print(f"T_MOS range: [{min(t_mos)}, {max(t_mos)}] °C")
        print(f"T_Rotor range: [{min(t_rot)}, {max(t_rot)}] °C")
        n_high_tau = sum(1 for t in tau if abs(t) >= 0.95 * limits[2])
        if n_high_tau:
            print(f"⚠ {n_high_tau} 帧力矩达到/接近 TMAX({limits[2]} Nm) —— 疑似过载/堵转")

    print("\n错误码统计 (ERR):")
    if err_counter:
        for code, n in sorted(err_counter.items()):
            print(f"  ERR=0x{code:X} ({ERR_MEANING.get(code, '未知')}) × {n}")
    else:
        print("  (日志中未识别到错误码)")

    if text_hits:
        print("\n文本标记:")
        for t in text_hits:
            print(f"  - {t}")

    if kv_series:
        print("\n文本字段极值:")
        for k, vals in kv_series.items():
            print(f"  {k}: [{min(vals)}, {max(vals)}]  ({len(vals)} 样本)")

    print("\n提示: 结合 SKILL.md 第2-3步归类症状并定位根因。若字段缺失，明确标注后请补充日志。")


if __name__ == "__main__":
    main()
