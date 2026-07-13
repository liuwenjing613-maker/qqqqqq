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

```bash
bash scripts/check_audio.sh
python3 src/list_audio_devices.py
```

## One-shot test

```bash
set -a
source .env
set +a

python3 src/asr_once.py --device 2 --seconds 5
```

Replace `2` with the actual microphone index.

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
