# iot_c2d_receiver

ROS 2 Python package that receives Azure IoT Hub Cloud-to-Device (C2D) messages and stores waypoint YAMLs to disk.

## Features
- Listens to Azure IoT Hub C2D messages via `azure-iot-device`.
- Saves incoming YAML to date-stamped files like `waypoints/YYYY-MM-DD/waypoints_%H%M%S.yaml`.
- Also updates `waypoints/current.yaml` with the most recent payload.
- Configurable via ROS 2 parameters and/or `.env` file.

## Prerequisites
- ROS 2 (e.g. Humble) environment sourced
- Python dependencies:
  - `azure-iot-device`
  - `python-dotenv`

These are declared in `setup.py` and will be installed when building the workspace, or you can:
```
pip3 install azure-iot-device python-dotenv
```

## Secrets (.env)
Provide your IoT Hub device connection string via a centralized `.env`:
- Path: `/home/dev/ORobotics/secrets/.env`
- Content:
```
IOTHUB_DEVICE_CONNECTION_STRING="HostName=...;DeviceId=...;SharedAccessKey=..."
```
Security tip: keep this file out of version control. A `.gitignore` is provided in `/home/dev/ORobotics/secrets/`.

## Build
```
cd /home/dev/ORobotics/localization_ws
colcon build --packages-select iot_c2d_receiver
source install/setup.bash
```

## Run (Launch)
Default launch (uses params from `config/default.yaml`):
```
ros2 launch iot_c2d_receiver c2d_receiver.launch.py
```

Override parameters from command line, e.g.:
```
ros2 launch iot_c2d_receiver c2d_receiver.launch.py \
  connection_string:="<your connection string>" \
  save_dir:="/home/dev/ORobotics/localization_ws/src/iot_c2d_receiver/waypoints" \
  env_file:="/home/dev/ORobotics/secrets/.env"
```

## Parameters
- `connection_string` (string)
  - If set, used directly.
  - If empty, the node expects you to supply it via an `.env` file using `env_file`.
- `save_dir` (string)
  - Folder root where waypoint files are stored.
  - Default: `/home/dev/ORobotics/localization_ws/src/iot_c2d_receiver/waypoints`
- `env_file` (string)
  - Optional path to a `.env` file. If provided and exists, it will be loaded via `python-dotenv`.

## Changing save_dir
You can update `config/default.yaml` or override via launch:
```
ros2 launch iot_c2d_receiver c2d_receiver.launch.py save_dir:="/path/to/your/waypoints"
```
The node will write `YYYY-MM-DD/waypoints_%H%M%S.yaml` and update `current.yaml` under the root.

## Docker usage
Two common options:

- Use `--env-file` to inject env vars:
```
docker run --rm \
  --env-file /home/dev/ORobotics/secrets/.env \
  -v /home/dev/ORobotics/localization_ws/src/iot_c2d_receiver/waypoints:/app/waypoints \
  your-image:tag \
  ros2 launch iot_c2d_receiver c2d_receiver.launch.py save_dir:="/app/waypoints"
```

- Mount a `.env` and pass the path to the node:
```
docker run --rm \
  -v /home/dev/ORobotics/secrets/.env:/app/.env:ro \
  -v /home/dev/ORobotics/localization_ws/src/iot_c2d_receiver/waypoints:/app/waypoints \
  your-image:tag \
  ros2 launch iot_c2d_receiver c2d_receiver.launch.py \
    env_file:="/app/.env" save_dir:="/app/waypoints"
```

## Notes
- If you do not want generated files in your repository, consider adding a `.gitignore` entry for `iot_c2d_receiver/waypoints/`.
- Logs are published via ROS 2 logger. Check the console output for connection status and file save paths.
