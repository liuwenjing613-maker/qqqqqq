from audio_utils import list_input_devices


def main() -> int:
    devices = list_input_devices()
    if not devices:
        print("No microphone input device was found.")
        return 1

    print("Available microphone input devices:")
    for device in devices:
        print(
            f'  index={device["index"]} | '
            f'name={device["name"]} | '
            f'max_input_channels={device["max_input_channels"]} | '
            f'default_sample_rate={device["default_sample_rate"]}'
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
