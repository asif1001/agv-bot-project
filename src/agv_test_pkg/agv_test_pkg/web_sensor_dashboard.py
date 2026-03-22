import json
import fcntl
import glob
import os
import select
import signal
import subprocess
import threading
import time
import termios
import struct
from collections import deque
from http import server
from urllib.parse import parse_qs, urlparse
from socketserver import ThreadingMixIn

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


HTML_TEMPLATE_FALLBACK_PATH = '/home/rpi4/ros2_ws/web_sensor_dashboard.html'

CAMERA_PRESETS = {
    'mjpg_640x480_30': {
        'label': 'MJPG 640x480 @ 30 FPS',
        'pixel_format': 'MJPG',
        'width': 640,
        'height': 480,
        'fps': 30,
    },
    'mjpg_1280x720_30': {
        'label': 'MJPG 1280x720 @ 30 FPS',
        'pixel_format': 'MJPG',
        'width': 1280,
        'height': 720,
        'fps': 30,
    },
    'yuyv_640x480_30': {
        'label': 'YUYV 640x480 @ 30 FPS',
        'pixel_format': 'YUYV',
        'width': 640,
        'height': 480,
        'fps': 30,
    },
    'yuyv_320x240_30': {
        'label': 'YUYV 320x240 @ 30 FPS',
        'pixel_format': 'YUYV',
        'width': 320,
        'height': 240,
        'fps': 30,
    },
}


def load_html_page() -> str:
    template_path = os.path.join(get_package_share_directory('agv_test_pkg'), 'templates', 'web_sensor_dashboard.html')
    if not os.path.exists(template_path):
        template_path = HTML_TEMPLATE_FALLBACK_PATH
    with open(template_path, 'r', encoding='utf-8') as file_handle:
        return file_handle.read()


class Esp32SerialMonitor:
    def __init__(self, baud_rate: int = 115200) -> None:
        self.baud_rate = baud_rate
        self.dtr_reset_pending = True
        self.lock = threading.Lock()
        self.lines = deque(maxlen=300)
        self.port = ''
        self.ip_address = ''
        self.last_message_time = 0.0
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _find_port(self) -> str:
        for pattern in ('/dev/ttyACM*', '/dev/ttyUSB*'):
            ports = sorted(glob.glob(pattern))
            if ports:
                return ports[0]
        return ''

    def _configure_port(self, fd: int) -> None:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)

    def _pulse_reset(self, fd: int) -> None:
        tiocmget = 0x5415
        tiocmset = 0x5418
        tiocm_dtr = 0x002
        state = struct.unpack('I', fcntl.ioctl(fd, tiocmget, struct.pack('I', 0)))[0]
        fcntl.ioctl(fd, tiocmset, struct.pack('I', state & ~tiocm_dtr))
        time.sleep(0.2)
        fcntl.ioctl(fd, tiocmset, struct.pack('I', state | tiocm_dtr))

    def _record_line(self, text: str) -> None:
        clean_text = text.strip()
        if not clean_text:
            return
        with self.lock:
            self.lines.append(clean_text)
            self.last_message_time = time.time()
            if 'ESP32 IP Address:' in clean_text:
                self.ip_address = clean_text.split(':', 1)[1].strip()

    def _run(self) -> None:
        buffer = b''
        while not self.stop_event.is_set():
            port = self._find_port()
            with self.lock:
                self.port = port
            if not port:
                time.sleep(1.0)
                continue
            try:
                fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                self._configure_port(fd)
                if self.dtr_reset_pending:
                    self._pulse_reset(fd)
                    self.dtr_reset_pending = False
            except OSError:
                time.sleep(1.0)
                continue
            try:
                while not self.stop_event.is_set():
                    ready, _, _ = select.select([fd], [], [], 0.5)
                    if fd not in ready:
                        continue
                    chunk = os.read(fd, 4096)
                    if not chunk:
                        break
                    buffer += chunk
                    while b'\n' in buffer:
                        raw_line, buffer = buffer.split(b'\n', 1)
                        self._record_line(raw_line.decode('utf-8', 'replace'))
            except OSError:
                time.sleep(0.5)
            finally:
                os.close(fd)

    def snapshot(self) -> dict:
        with self.lock:
            if time.time() - self.last_message_time < 10.0:
                stream_status = 'live'
            elif self.lines:
                stream_status = 'history'
            else:
                stream_status = 'waiting'
            return {
                'port': self.port,
                'ip_address': self.ip_address,
                'last_message_time': self.last_message_time,
                'lines': list(self.lines),
                'stream': stream_status,
            }

    def shutdown(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2.0)


class ManagedProcess:
    def __init__(self, name: str, command: str) -> None:
        self.name = name
        self.command = command
        self.process = None
        self.lock = threading.Lock()

    def is_running(self) -> bool:
        with self.lock:
            return self.process is not None and self.process.poll() is None

    def status(self) -> str:
        with self.lock:
            if self.process is None:
                return 'stopped'
            if self.process.poll() is None:
                return f'running (pid {self.process.pid})'
            code = self.process.returncode
            self.process = None
            return f'exited ({code})'

    def start(self) -> str:
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return f'{self.name} already running'
            self.process = subprocess.Popen(
                ['/bin/bash', '-lc', self.command],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            return f'{self.name} started'

    def stop(self) -> str:
        with self.lock:
            if self.process is None or self.process.poll() is not None:
                self.process = None
                return f'{self.name} already stopped'
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=5)
            self.process = None
            return f'{self.name} stopped'


class DashboardState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.frame_bytes = None
        self.last_frame_time = 0.0
        self.camera_preset_id = 'yuyv_320x240_30'
        self.scan_payload = {
            'ranges': [],
            'angle_min': 0.0,
            'angle_increment': 0.0,
            'range_min': 0.0,
            'range_max': 0.0,
            'stamp': 0.0,
        }
        common_env = 'export ROS_DOMAIN_ID=30; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; source /opt/ros/jazzy/setup.bash; source /home/rpi4/ros2_ws/install/setup.bash; '
        self.camera_process = ManagedProcess(
            'camera',
            self._camera_command(common_env),
        )
        self.lidar_process = ManagedProcess(
            'lidar',
            common_env + 'ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:=/home/rpi4/ros2_ws/src/agv-bot-project/src/ydlidar_ros2_driver/params/X3.yaml',
        )
        self.esp32_monitor = Esp32SerialMonitor()
        self.camera_process.start()
        self.lidar_process.start()

    def _camera_command(self, common_env: str) -> str:
        preset = CAMERA_PRESETS[self.camera_preset_id]
        return (
            common_env
            + 'ros2 run v4l2_camera v4l2_camera_node --ros-args '
            + '-p video_device:=/dev/video0 '
            + f'-p image_size:=[{preset["width"]},{preset["height"]}] '
            + f'-p time_per_frame:=[1,{preset["fps"]}] '
            + f'-p pixel_format:={preset["pixel_format"]} '
            + '-r image_raw:=/camera/image_raw '
            + '-r camera_info:=/camera/camera_info'
        )

    def camera_config(self) -> dict:
        preset = CAMERA_PRESETS[self.camera_preset_id]
        return {
            'selected_preset': self.camera_preset_id,
            'selected_label': preset['label'],
            'presets': [
                {'id': preset_id, 'label': preset_value['label']}
                for preset_id, preset_value in CAMERA_PRESETS.items()
            ],
        }

    def set_camera_preset(self, preset_id: str) -> str:
        if preset_id not in CAMERA_PRESETS:
            raise ValueError('Unsupported camera preset')
        with self.lock:
            self.camera_preset_id = preset_id
            self.frame_bytes = None
            self.last_frame_time = 0.0
        common_env = 'export ROS_DOMAIN_ID=30; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; source /opt/ros/jazzy/setup.bash; source /home/rpi4/ros2_ws/install/setup.bash; '
        self.camera_process.command = self._camera_command(common_env)
        self.camera_process.stop()
        return self.camera_process.start()

    def process_status(self) -> dict:
        esp32_snapshot = self.esp32_monitor.snapshot()
        camera_config = self.camera_config()
        return {
            'camera_process': self.camera_process.status(),
            'lidar_process': self.lidar_process.status(),
            'camera_stream': 'live' if time.time() - self.last_frame_time < 2.0 else 'waiting',
            'lidar_stream': 'live' if time.time() - self.scan_payload['stamp'] < 2.0 else 'waiting',
            'camera_preset': camera_config['selected_preset'],
            'camera_preset_label': camera_config['selected_label'],
            'esp32_stream': esp32_snapshot['stream'],
            'esp32_port': esp32_snapshot['port'],
            'esp32_ip': esp32_snapshot['ip_address'],
            'esp32_last_message_time': esp32_snapshot['last_message_time'],
        }

    def control(self, sensor: str, action: str) -> str:
        if sensor == 'camera':
            process = self.camera_process
        elif sensor == 'lidar':
            process = self.lidar_process
        else:
            raise ValueError('Unsupported sensor')

        if action == 'start':
            return process.start()
        if action == 'stop':
            if sensor == 'camera':
                with self.lock:
                    self.frame_bytes = None
                    self.last_frame_time = 0.0
            if sensor == 'lidar':
                with self.lock:
                    self.scan_payload = {
                        'ranges': [],
                        'angle_min': 0.0,
                        'angle_increment': 0.0,
                        'range_min': 0.0,
                        'range_max': 0.0,
                        'stamp': 0.0,
                    }
            return process.stop()
        raise ValueError('Unsupported action')

    def shutdown(self) -> None:
        self.camera_process.stop()
        self.lidar_process.stop()
        self.esp32_monitor.shutdown()


class SensorDashboardNode(Node):
    def __init__(self, state: DashboardState) -> None:
        super().__init__('web_sensor_dashboard')
        self.state = state
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.get_logger().info('web_sensor_dashboard subscribed to /camera/image_raw and /scan')

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if not ok:
                return
            with self.state.lock:
                self.state.frame_bytes = encoded.tobytes()
                self.state.last_frame_time = time.time()
        except Exception as exc:
            self.get_logger().error(f'Failed to convert camera image: {exc}')

    def scan_callback(self, msg: LaserScan) -> None:
        sample_step = max(1, len(msg.ranges) // 720)
        ranges = []
        for index in range(0, len(msg.ranges), sample_step):
            value = msg.ranges[index]
            if np.isfinite(value):
                ranges.append(float(value))
            else:
                ranges.append(0.0)
        payload = {
            'ranges': ranges,
            'angle_min': float(msg.angle_min),
            'angle_increment': float(msg.angle_increment * sample_step),
            'range_min': float(msg.range_min),
            'range_max': float(msg.range_max),
            'stamp': float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9,
        }
        with self.state.lock:
            self.state.scan_payload = payload


class ThreadedHTTPServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class DashboardRequestHandler(server.BaseHTTPRequestHandler):
    server_version = 'PiSensorDashboard/1.0'

    def do_GET(self):
        parsed = urlparse(self.path)
        request_path = parsed.path

        if request_path in ('/', '/index.html'):
            body = load_html_page().encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if request_path == '/lidar.json':
            with self.server.state.lock:
                payload = json.dumps(self.server.state.scan_payload).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        if request_path == '/status.json':
            payload = json.dumps(self.server.state.process_status()).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        if request_path == '/camera_config.json':
            payload = json.dumps(self.server.state.camera_config()).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        if request_path == '/esp32.json':
            payload = json.dumps(self.server.state.esp32_monitor.snapshot()).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        if request_path == '/camera.mjpg':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            last_sent_at = 0.0
            try:
                while True:
                    with self.server.state.lock:
                        frame = self.server.state.frame_bytes
                    now = time.time()
                    if frame and (now - last_sent_at >= 0.12):
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode('utf-8'))
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                        last_sent_at = now
                    elif not frame:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: text/plain\r\n\r\n')
                        self.wfile.write(b'waiting\r\n')
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                return

        self.send_error(404)

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/camera_config':
            length = int(self.headers.get('Content-Length', '0'))
            body = self.rfile.read(length).decode('utf-8')
            params = parse_qs(body)
            preset_id = params.get('preset', [''])[0]
            try:
                message = self.server.state.set_camera_preset(preset_id)
            except ValueError as exc:
                payload = str(exc).encode('utf-8')
                self.send_response(400)
                self.send_header('Content-Type', 'text/plain; charset=utf-8')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return
            payload = json.dumps({'message': message}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        if parsed.path != '/control':
            self.send_error(404)
            return
        length = int(self.headers.get('Content-Length', '0'))
        body = self.rfile.read(length).decode('utf-8')
        params = parse_qs(body)
        sensor = params.get('sensor', [''])[0]
        action = params.get('action', [''])[0]
        try:
            message = self.server.state.control(sensor, action)
        except ValueError as exc:
            payload = str(exc).encode('utf-8')
            self.send_response(400)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        payload = json.dumps({'message': message}).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, format, *args):
        return


def main(args=None) -> None:
    rclpy.init(args=args)
    state = DashboardState()
    node = SensorDashboardNode(state)

    host = '0.0.0.0'
    port = 8080
    httpd = ThreadedHTTPServer((host, port), DashboardRequestHandler)
    httpd.state = state

    server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(f'Web dashboard serving on http://0.0.0.0:{port}')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        httpd.server_close()
        state.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
