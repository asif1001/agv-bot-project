# Dashboard Service Install and Deploy

## Repo-managed files

- `templates/web_sensor_dashboard.html`
- `scripts/start_web_dashboard.sh`
- `systemd/pi-sensor-dashboard.service`

## Build the package

```bash
cd /home/rpi4/ros2_ws
colcon build --packages-select agv_test_pkg
```

## Install or update the systemd service

Copy the repo-managed unit into systemd:

```bash
sudo cp /home/rpi4/ros2_ws/src/agv-bot-project/src/agv_test_pkg/systemd/pi-sensor-dashboard.service /etc/systemd/system/pi-sensor-dashboard.service
sudo systemctl daemon-reload
```

## Start or restart the dashboard

```bash
sudo systemctl restart pi-sensor-dashboard.service
sudo systemctl status pi-sensor-dashboard.service
```

## Service behavior

- Runs as user `rpi4`
- Uses working directory `/home/rpi4/ros2_ws`
- Starts the dashboard from the repo-managed script:
  - `/home/rpi4/ros2_ws/src/agv-bot-project/src/agv_test_pkg/scripts/start_web_dashboard.sh`

## Deploy workflow after code changes

```bash
cd /home/rpi4/ros2_ws
colcon build --packages-select agv_test_pkg
sudo systemctl restart pi-sensor-dashboard.service
```

## Optional cleanup of old duplicate workspace files

After the active systemd unit is updated, these old top-level duplicates can be removed:

- `/home/rpi4/ros2_ws/web_sensor_dashboard.html`
- `/home/rpi4/ros2_ws/start_web_dashboard.sh`
- `/home/rpi4/ros2_ws/pi-sensor-dashboard.service`
