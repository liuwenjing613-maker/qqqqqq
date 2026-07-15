# ROSMASTER M1 Voice Interaction — Phase 1

Goal:

1. Detect the microphone.
2. Record 16 kHz mono WAV audio.
3. Send the WAV file to Qwen ASR.
4. Print the recognized text.
5. Publish the text to ROS 2 topic `/voice/text`.

## Install

```bash
sudo apt update
sudo apt install -y alsa-utils portaudio19-dev python3-pyaudio
python3 -m pip install -r requirements.txt
```

## Configure

```bash
cp .env.example .env
nano .env
```

Set `DASHSCOPE_API_KEY`.

## Check microphone

Board mic: **USB C-Media** (`plughw:0,0`, ALSA card 0, PyAudio index 0).

```bash
bash scripts/check_audio.sh
cd src && python3 list_audio_devices.py
```

## One-shot test

```bash
set -a
source .env
set +a

cd src
python3 asr_once.py --device 0 --seconds 5
```

## ROS 2 test

Terminal A:

```bash
bash scripts/run_voice_asr.sh
```

Terminal B:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /voice/text
```
