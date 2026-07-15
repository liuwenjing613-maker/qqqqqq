#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/setup_usb_mic.sh
source "${ROOT_DIR}/scripts/lib/setup_usb_mic.sh"

pkill -f 'kws_test.py' 2>/dev/null || true
pkill -f 'arecord.*plughw:0,0' 2>/dev/null || true
pkill -f 'arecord.*plughw:1,0' 2>/dev/null || true
sleep 1
setup_usb_mic

OUT_WAV="${ROOT_DIR}/recordings/mic_speak_test.wav"
mkdir -p "${ROOT_DIR}/recordings"

echo ""
echo "========================================"
echo "  USB 麦克风测试 - 请留意终端输出"
echo "========================================"
echo "设备: plughw:0,0 (C-Media USB Audio)"
echo "3 秒后开始录音，请准备好对着 USB 麦克风说话..."
sleep 3
echo ""
echo ">>>>>> 现在开始说话！请连续说 3 秒 <<<<<<"
echo ""
arecord -D plughw:0,0 -f S16_LE -r 16000 -c 1 -d 3 "${OUT_WAV}"
echo ""
echo "录音结束，正在分析..."
python3 <<PY
import wave, struct, math
path = "${OUT_WAV}"
w = wave.open(path)
s = struct.unpack("<" + "h" * w.getnframes(), w.readframes(w.getnframes()))
sr = 16000
chunk = sr // 5
print()
print("========== 音量分析结果 ==========")
print(f"文件: {path}")
print(f"时长: {len(s)/sr:.1f} 秒")
print()
print("时间轴 (每 0.2 秒一行，# 越多表示声音越大):")
print("-" * 50)
max_rms = 0
speak_chunks = 0
for i in range(0, len(s), chunk):
    part = s[i:i+chunk]
    rms = math.sqrt(sum(x*x for x in part) / len(part))
    max_rms = max(max_rms, rms)
    if rms > 800:
        speak_chunks += 1
    bar = "#" * min(50, int(rms / 150))
    mark = " <-- 有声音" if rms > 800 else (" <-- 很轻" if rms < 300 else "")
    print(f"  {i/sr:4.1f}s  RMS={rms:7.1f}  {bar}{mark}")
overall = math.sqrt(sum(x*x for x in s) / len(s))
peak = max(abs(x) for x in s)
print("-" * 50)
print(f"整体 RMS: {overall:.1f}")
print(f"峰值:     {peak}")
print()
if speak_chunks >= 3 and max_rms > 800:
    print("结论: PASS - 麦克风能稳定采到你的声音")
elif max_rms > 800:
    print("结论: 勉强 - 有声音但太短，请连续说满 3 秒")
elif max_rms > 300:
    print("结论: 偏弱 - 可调大增益: VOICE_USB_MIC_LEVEL=32 bash scripts/mic_level_test.sh")
else:
    print("结论: FAIL - 几乎没采到声音，请检查 USB 连接与麦克风位置")
print("==================================")
PY
