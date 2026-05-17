import json
import math
import os
import subprocess
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse


def _inject_odrive_venv_site_packages() -> None:
    version = f'python{sys.version_info.major}.{sys.version_info.minor}'
    candidates = [
        Path.home() / 'odrive-venv' / 'lib' / version / 'site-packages',
        Path.home() / 'odrive-env' / 'lib' / version / 'site-packages',
    ]
    for candidate in candidates:
        if candidate.exists():
            candidate_str = str(candidate)
            if candidate_str not in sys.path:
                sys.path.insert(0, candidate_str)


_inject_odrive_venv_site_packages()

AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
CONTROL_MODE_VELOCITY_CONTROL = 2
MOTOR_TYPE_HIGH_CURRENT = 0

try:
    import odrive
    try:
        from odrive import enums as odrive_enums

        AXIS_STATE_CLOSED_LOOP_CONTROL = getattr(
            odrive_enums, 'AXIS_STATE_CLOSED_LOOP_CONTROL', AXIS_STATE_CLOSED_LOOP_CONTROL
        )
        AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = getattr(
            odrive_enums,
            'AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION',
            AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION,
        )
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = getattr(
            odrive_enums, 'AXIS_STATE_ENCODER_OFFSET_CALIBRATION', AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        )
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = getattr(
            odrive_enums, 'AXIS_STATE_FULL_CALIBRATION_SEQUENCE', AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        )
        CONTROL_MODE_VELOCITY_CONTROL = getattr(
            odrive_enums, 'CONTROL_MODE_VELOCITY_CONTROL', CONTROL_MODE_VELOCITY_CONTROL
        )
        MOTOR_TYPE_HIGH_CURRENT = getattr(
            odrive_enums, 'MOTOR_TYPE_HIGH_CURRENT', MOTOR_TYPE_HIGH_CURRENT
        )
    except Exception:
        pass
except ImportError:  # pragma: no cover
    odrive = None

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
except ImportError:  # pragma: no cover
    rclpy = None
    SingleThreadedExecutor = None
    Node = None
    qos_profile_sensor_data = None
    LaserScan = None

WHEEL_DIAMETER_METERS = 0.1651
WHEEL_CIRCUMFERENCE_METERS = math.pi * WHEEL_DIAMETER_METERS
STEPS_PER_ROTATION = 100
DEFAULT_HOST = os.environ.get('ODRIVE_DASHBOARD_HOST', '0.0.0.0')
DEFAULT_PORT = int(os.environ.get('ODRIVE_DASHBOARD_PORT', '8081'))
DEFAULT_CAMERA_URL = os.environ.get('ODRIVE_DASHBOARD_CAMERA_URL', '')
DEFAULT_ROSBRIDGE_URL = os.environ.get('ODRIVE_DASHBOARD_ROSBRIDGE_URL', 'ws://127.0.0.1:9090')

HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>AGV Robotics Control Center</title>
  <style>
    :root { color-scheme: dark; }
    body { margin: 0; font-family: Arial, sans-serif; background: linear-gradient(180deg, #020617 0%, #0f172a 55%, #111827 100%); color: #e5eefc; }
    .app-shell { max-width: 1280px; margin: 0 auto; padding: 18px; }
    .hero { display: flex; flex-wrap: wrap; justify-content: space-between; gap: 16px; align-items: center; margin-bottom: 18px; padding: 20px; border-radius: 22px; background: rgba(15, 23, 42, 0.88); border: 1px solid rgba(148, 163, 184, 0.16); box-shadow: 0 18px 40px rgba(0,0,0,0.28); }
    .hero h1 { margin: 0 0 6px; font-size: clamp(24px, 4vw, 36px); }
    .hero p { margin: 0; color: #94a3b8; }
    .hero-metrics { display: grid; grid-template-columns: repeat(3, minmax(120px, 1fr)); gap: 12px; width: min(100%, 430px); }
    .pill { background: rgba(37, 99, 235, 0.12); border: 1px solid rgba(96, 165, 250, 0.18); border-radius: 16px; padding: 12px 14px; }
    .pill-label { font-size: 12px; color: #94a3b8; }
    .pill-value { margin-top: 6px; font-size: 20px; font-weight: 700; }
    .tabbar { display: flex; gap: 10px; flex-wrap: wrap; margin-bottom: 18px; }
    .tabbar button { width: auto; min-width: 120px; margin: 0; padding: 12px 18px; border-radius: 999px; border: 1px solid #334155; background: #0f172a; color: #cbd5e1; font-weight: 700; }
    .tabbar button.active { background: linear-gradient(135deg, #2563eb, #0891b2); color: white; border-color: transparent; }
    .page { display: none; }
    .page.active { display: block; }
    .grid { display: grid; grid-template-columns: repeat(12, minmax(0, 1fr)); gap: 16px; }
    .card { grid-column: span 12; background: rgba(15, 23, 42, 0.9); border: 1px solid rgba(148, 163, 184, 0.14); border-radius: 22px; padding: 18px; box-shadow: 0 18px 40px rgba(0,0,0,0.24); }
    .card h2, .card h3 { margin-top: 0; margin-bottom: 12px; }
    .span-7 { grid-column: span 7; }
    .span-5 { grid-column: span 5; }
    .span-6 { grid-column: span 6; }
    .status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 12px; }
    .metric { background: #111827; border: 1px solid rgba(148, 163, 184, 0.12); border-radius: 16px; padding: 14px; }
    .metric .label { color: #94a3b8; font-size: 12px; }
    .metric .value { font-size: 20px; font-weight: 700; margin-top: 8px; }
    .subsystem-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(210px, 1fr)); gap: 12px; }
    .subsystem { background: #111827; border: 1px solid rgba(148, 163, 184, 0.12); border-radius: 18px; padding: 16px; }
    .subsystem-head { display: flex; justify-content: space-between; align-items: center; gap: 12px; margin-bottom: 12px; }
    .subsystem-head h3 { margin: 0; font-size: 16px; }
    .badge { display: inline-flex; align-items: center; gap: 8px; padding: 6px 10px; border-radius: 999px; font-size: 12px; font-weight: 700; }
    .badge.ok { background: rgba(34, 197, 94, 0.14); color: #86efac; }
    .badge.warn { background: rgba(245, 158, 11, 0.14); color: #fcd34d; }
    .badge.off { background: rgba(148, 163, 184, 0.14); color: #cbd5e1; }
    .badge.err { background: rgba(239, 68, 68, 0.14); color: #fca5a5; }
    .subsystem-list { display: grid; gap: 8px; color: #cbd5e1; font-size: 14px; }
    .subsystem-row { display: flex; justify-content: space-between; gap: 12px; }
    label { display: block; margin: 10px 0 6px; color: #cbd5e1; }
    input { width: 100%; padding: 12px; border-radius: 12px; border: 1px solid #334155; background: #020617; color: white; box-sizing: border-box; }
    button { width: 100%; padding: 12px; border: 0; border-radius: 14px; margin-top: 10px; font-weight: 700; cursor: pointer; transition: transform 0.15s ease; }
    button:hover { transform: translateY(-1px); }
    .primary { background: linear-gradient(135deg, #2563eb, #0891b2); color: white; }
    .danger { background: linear-gradient(135deg, #dc2626, #ef4444); color: white; }
    .secondary { background: #1e293b; color: white; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .camera-frame { width: 100%; aspect-ratio: 16 / 9; background: #020617; border-radius: 18px; border: 1px solid rgba(148, 163, 184, 0.15); object-fit: cover; }
    canvas { width: 100%; background: #020617; border-radius: 18px; display: block; border: 1px solid rgba(148, 163, 184, 0.15); }
    .joystick-wrap { display: grid; gap: 12px; }
    .joystick-pad { position: relative; width: min(320px, 72vw); height: min(320px, 72vw); margin: 0 auto; border-radius: 50%; background: radial-gradient(circle at center, #1e293b 0%, #0f172a 65%, #020617 100%); border: 2px solid #334155; touch-action: none; user-select: none; }
    .joystick-crosshair { position: absolute; inset: 0; }
    .joystick-crosshair::before, .joystick-crosshair::after { content: ''; position: absolute; background: #334155; }
    .joystick-crosshair::before { left: 50%; top: 10%; width: 2px; height: 80%; transform: translateX(-50%); }
    .joystick-crosshair::after { top: 50%; left: 10%; height: 2px; width: 80%; transform: translateY(-50%); }
    .joystick-knob { position: absolute; width: 92px; height: 92px; border-radius: 50%; background: linear-gradient(135deg, #38bdf8, #2563eb); box-shadow: 0 10px 24px rgba(37,99,235,0.45); left: 50%; top: 50%; transform: translate(-50%, -50%); }
    .speed-readout { text-align: center; color: #cbd5e1; font-size: 15px; }
    .hint { color: #94a3b8; font-size: 13px; text-align: center; }
    .log { white-space: pre-wrap; background: #020617; border: 1px solid rgba(148, 163, 184, 0.1); border-radius: 16px; padding: 14px; min-height: 72px; color: #cbd5e1; }
    .actions { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 10px; }
    @media (max-width: 960px) { .span-7, .span-5, .span-6 { grid-column: span 12; } }
    @media (max-width: 720px) {
      .app-shell { padding: 12px; }
      .hero { padding: 16px; }
      .hero-metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .row { grid-template-columns: 1fr; }
      button { padding: 14px; }
    }
  </style>
</head>
<body>
  <div class="app-shell">
    <div class="hero">
      <div>
        <h1>AGV Robotics Control Center</h1>
        <p>Live teleoperation, perception, and subsystem monitoring for your ODrive hoverboard robot.</p>
      </div>
      <div class="hero-metrics">
        <div class="pill"><div class="pill-label">ODrive</div><div id="hero_odrive" class="pill-value">Offline</div></div>
        <div class="pill"><div class="pill-label">LiDAR</div><div id="hero_lidar" class="pill-value">Idle</div></div>
        <div class="pill"><div class="pill-label">Battery</div><div id="hero_battery" class="pill-value">-</div></div>
      </div>
    </div>

    <div class="tabbar">
      <button id="tab_home" class="active" onclick="showTab('home')">Home</button>
      <button id="tab_lidar" onclick="showTab('lidar')">LiDAR</button>
      <button id="tab_system" onclick="showTab('system')">System</button>
    </div>

    <div id="page_home" class="page active">
      <div class="grid">
        <div class="card span-7">
          <h2>Live Camera</h2>
          <img id="camera_feed" class="camera-frame" alt="Camera feed will appear here">
          <label for="camera_url_input">Camera Stream URL</label>
          <input id="camera_url_input" type="text" placeholder="http://127.0.0.1:8080/stream">
          <div class="row">
            <button class="secondary" onclick="setCameraUrl()">Set Camera Feed</button>
            <button class="secondary" onclick="clearCameraUrl()">Clear Camera Feed</button>
          </div>
        </div>

        <div class="card span-5">
          <h2>Joystick Control</h2>
          <div class="joystick-wrap">
            <label for="speed_slider">Speed Level</label>
            <input id="speed_slider" type="range" min="0.1" max="3.0" step="0.1" value="1.0" oninput="updateSpeedReadout()">
            <div id="speed_readout" class="speed-readout">1.0 turns/sec</div>
            <div id="joystick_pad" class="joystick-pad">
              <div class="joystick-crosshair"></div>
              <div id="joystick_knob" class="joystick-knob"></div>
            </div>
            <div class="row">
              <button class="secondary" onmousedown="quickDrive(0,1)" onmouseup="stopDrive()" onmouseleave="stopDrive()" ontouchstart="quickDrive(0,1)" ontouchend="stopDrive()">Forward</button>
              <button class="secondary" onmousedown="quickDrive(0,-1)" onmouseup="stopDrive()" onmouseleave="stopDrive()" ontouchstart="quickDrive(0,-1)" ontouchend="stopDrive()">Backward</button>
            </div>
            <div class="row">
              <button class="secondary" onmousedown="quickDrive(-1,0)" onmouseup="stopDrive()" onmouseleave="stopDrive()" ontouchstart="quickDrive(-1,0)" ontouchend="stopDrive()">Left</button>
              <button class="secondary" onmousedown="quickDrive(1,0)" onmouseup="stopDrive()" onmouseleave="stopDrive()" ontouchstart="quickDrive(1,0)" ontouchend="stopDrive()">Right</button>
            </div>
            <button class="danger" onclick="postJson('/api/stop', {})">Emergency Stop</button>
            <div class="hint">Drag for mixed steering. Release the joystick to stop the robot.</div>
          </div>
        </div>

        <div class="card span-12">
          <h2>Quick Robot Controls</h2>
          <div class="actions">
            <button class="secondary" onclick="postJson('/api/connect', {})">Reconnect ODrive</button>
            <button class="secondary" onclick="postJson('/api/configure', {})">Apply Wheel Config</button>
            <button class="secondary" onclick="postJson('/api/calibrate', {})">Run Calibration</button>
            <button class="primary" onclick="setVelocity(1)">Drive Forward</button>
            <button class="primary" onclick="setVelocity(-1)">Drive Backward</button>
          </div>
          <label for="velocity_input">Velocity (turns/sec)</label>
          <input id="velocity_input" type="number" step="0.1" value="1.0">
        </div>
      </div>
    </div>

    <div id="page_lidar" class="page">
      <div class="grid">
        <div class="card span-12">
          <h2>LiDAR Perception</h2>
          <canvas id="lidar_canvas" width="1100" height="520"></canvas>
        </div>
        <div class="card span-6">
          <h3>LiDAR Metrics</h3>
          <div class="status-grid">
            <div class="metric"><div class="label">LiDAR Status</div><div id="lidar_status" class="value">-</div></div>
            <div class="metric"><div class="label">Points</div><div id="lidar_points" class="value">-</div></div>
            <div class="metric"><div class="label">Age (s)</div><div id="lidar_age" class="value">-</div></div>
            <div class="metric"><div class="label">ROS Bridge</div><div id="rosbridge_status" class="value">-</div></div>
          </div>
        </div>
        <div class="card span-6">
          <h3>Position & Motion</h3>
          <div class="status-grid">
            <div class="metric"><div class="label">Pose X</div><div id="pose_x" class="value">-</div></div>
            <div class="metric"><div class="label">Pose Y</div><div id="pose_y" class="value">-</div></div>
            <div class="metric"><div class="label">Heading</div><div id="pose_theta" class="value">-</div></div>
            <div class="metric"><div class="label">Odom Speed</div><div id="odom_speed" class="value">-</div></div>
          </div>
          <label for="rosbridge_url_input">rosbridge WebSocket URL</label>
          <input id="rosbridge_url_input" type="text" placeholder="ws://127.0.0.1:9090">
          <div class="row">
            <button class="secondary" onclick="connectRosbridge(true)">Connect ROS Bridge</button>
            <button class="secondary" onclick="disconnectRosbridge()">Disconnect ROS Bridge</button>
          </div>
        </div>
      </div>
    </div>

    <div id="page_system" class="page">
      <div class="grid">
        <div class="card span-12">
          <h2>System Health</h2>
          <div class="subsystem-grid">
            <div class="subsystem">
              <div class="subsystem-head"><h3>Battery</h3><span id="battery_badge" class="badge off">Unknown</span></div>
              <div class="subsystem-list">
                <div class="subsystem-row"><span>Charge</span><strong id="battery_percent">-</strong></div>
                <div class="subsystem-row"><span>Bus Voltage</span><strong id="bus_voltage">-</strong></div>
              </div>
            </div>
            <div class="subsystem">
              <div class="subsystem-head"><h3>ODrive</h3><span id="odrive_badge" class="badge off">Offline</span></div>
              <div class="subsystem-list">
                <div class="subsystem-row"><span>Connection</span><strong id="connected">-</strong></div>
                <div class="subsystem-row"><span>Serial</span><strong id="odrive_serial">-</strong></div>
              </div>
            </div>
            <div class="subsystem">
              <div class="subsystem-head"><h3>Hoverboard Wheels</h3><span id="wheels_badge" class="badge off">Offline</span></div>
              <div class="subsystem-list">
                <div class="subsystem-row"><span>Left Wheel</span><strong id="axis0_state">-</strong></div>
                <div class="subsystem-row"><span>Right Wheel</span><strong id="axis1_state">-</strong></div>
                <div class="subsystem-row"><span>Robot Distance</span><strong id="robot_distance">-</strong></div>
              </div>
            </div>
            <div class="subsystem">
              <div class="subsystem-head"><h3>Camera</h3><span id="camera_badge" class="badge off">Offline</span></div>
              <div class="subsystem-list">
                <div class="subsystem-row"><span>Feed URL</span><strong id="camera_status">Not set</strong></div>
              </div>
            </div>
            <div class="subsystem">
              <div class="subsystem-head"><h3>LiDAR</h3><span id="lidar_badge" class="badge off">Idle</span></div>
              <div class="subsystem-list">
                <div class="subsystem-row"><span>Points</span><strong id="system_lidar_points">-</strong></div>
                <div class="subsystem-row"><span>Age</span><strong id="system_lidar_age">-</strong></div>
              </div>
            </div>
            <div class="subsystem">
              <div class="subsystem-head"><h3>Motion Telemetry</h3><span id="motion_badge" class="badge off">Idle</span></div>
              <div class="subsystem-list">
                <div class="subsystem-row"><span>M0 Velocity</span><strong id="axis0_velocity">-</strong></div>
                <div class="subsystem-row"><span>M1 Velocity</span><strong id="axis1_velocity">-</strong></div>
                <div class="subsystem-row"><span>Robot Turns</span><strong id="robot_turns">-</strong></div>
              </div>
            </div>
          </div>
        </div>
        <div class="card span-6">
          <h3>Wheel Position</h3>
          <div class="status-grid">
            <div class="metric"><div class="label">M0 Turns</div><div id="axis0_turns" class="value">-</div></div>
            <div class="metric"><div class="label">M1 Turns</div><div id="axis1_turns" class="value">-</div></div>
          </div>
        </div>
        <div class="card span-6">
          <h3>System Log</h3>
          <div id="log" class="log">Ready.</div>
        </div>
      </div>
    </div>
  </div>
  <script>
    const defaultRosbridgeUrl = '""" + DEFAULT_ROSBRIDGE_URL + """';
    let ros = null;
    let scanTopic = null;
    let odomTopic = null;
    let poseTopic = null;
    let diagnosticsTopic = null;
    let rosbridgeConnected = false;
    let latestRosLidarPointCount = 0;
    let latestRosLidarAgeSec = 0;
    let latestPoseX = null;
    let latestPoseY = null;
    let latestPoseThetaDeg = null;
    let latestOdomSpeed = null;
    let joystickActive = false;
    let joystickPad = null;
    let joystickKnob = null;
    let joystickRadius = 0;

    function showTab(name) {
      for (const page of document.querySelectorAll('.page')) {
        page.classList.remove('active');
      }
      for (const button of document.querySelectorAll('.tabbar button')) {
        button.classList.remove('active');
      }
      document.getElementById('page_' + name).classList.add('active');
      document.getElementById('tab_' + name).classList.add('active');
    }

    function setBadge(id, text, mode) {
      const node = document.getElementById(id);
      if (!node) {
        return;
      }
      node.textContent = text;
      node.className = 'badge ' + mode;
    }

    function quaternionToYawDegrees(q) {
      const siny = 2.0 * ((q.w || 0) * (q.z || 0) + (q.x || 0) * (q.y || 0));
      const cosy = 1.0 - 2.0 * (((q.y || 0) * (q.y || 0)) + ((q.z || 0) * (q.z || 0)));
      return Math.atan2(siny, cosy) * 180.0 / Math.PI;
    }

    function updateRosbridgeIndicators() {
      document.getElementById('rosbridge_status').textContent = rosbridgeConnected ? 'Connected' : 'Disconnected';
      document.getElementById('pose_x').textContent = latestPoseX == null ? '-' : latestPoseX.toFixed(2) + ' m';
      document.getElementById('pose_y').textContent = latestPoseY == null ? '-' : latestPoseY.toFixed(2) + ' m';
      document.getElementById('pose_theta').textContent = latestPoseThetaDeg == null ? '-' : latestPoseThetaDeg.toFixed(1) + '°';
      document.getElementById('odom_speed').textContent = latestOdomSpeed == null ? '-' : latestOdomSpeed.toFixed(2) + ' m/s';
      if (rosbridgeConnected && latestRosLidarPointCount > 0) {
        document.getElementById('lidar_points').textContent = latestRosLidarPointCount;
        document.getElementById('lidar_age').textContent = latestRosLidarAgeSec.toFixed(2);
      }
    }

    function updateSystemSummary(data) {
      const connected = !!data.connected;
      const lidarActive = (data.lidar_point_count || 0) > 0 && (data.lidar_age_sec || 0) < 1.5;
      const batteryPercent = data.battery_percent == null ? '-' : data.battery_percent.toFixed(0) + '%';
      const batteryMode = data.battery_percent == null ? 'off' : (data.battery_percent > 45 ? 'ok' : (data.battery_percent > 20 ? 'warn' : 'err'));
      const wheelActive = connected && (Math.abs(data.axis0_vel_estimate || 0) > 0.02 || Math.abs(data.axis1_vel_estimate || 0) > 0.02);
      const cameraActive = !!(data.camera_url || '');
      document.getElementById('hero_odrive').textContent = connected ? 'Online' : 'Offline';
      document.getElementById('hero_lidar').textContent = lidarActive ? 'Active' : 'Idle';
      document.getElementById('hero_battery').textContent = batteryPercent;
      setBadge('battery_badge', batteryPercent === '-' ? 'Unknown' : (batteryMode === 'ok' ? 'Healthy' : (batteryMode === 'warn' ? 'Low' : 'Critical')), batteryMode);
      setBadge('odrive_badge', connected ? 'Connected' : 'Offline', connected ? 'ok' : 'off');
      setBadge('wheels_badge', connected ? 'Ready' : 'Offline', connected ? 'ok' : 'off');
      setBadge('camera_badge', cameraActive ? 'Active' : 'Offline', cameraActive ? 'ok' : 'off');
      setBadge('lidar_badge', lidarActive ? 'Active' : 'Idle', lidarActive ? 'ok' : 'off');
      setBadge('motion_badge', wheelActive ? 'Moving' : 'Idle', wheelActive ? 'warn' : (connected ? 'ok' : 'off'));
      document.getElementById('battery_percent').textContent = batteryPercent;
      document.getElementById('bus_voltage').textContent = data.bus_voltage == null ? '-' : data.bus_voltage.toFixed(2) + ' V';
      document.getElementById('connected').textContent = connected ? 'Yes' : 'No';
      document.getElementById('odrive_serial').textContent = data.odrive_serial || '-';
      document.getElementById('camera_status').textContent = data.camera_url ? 'Configured' : 'Not set';
      document.getElementById('system_lidar_points').textContent = data.lidar_point_count ?? 0;
      document.getElementById('system_lidar_age').textContent = (data.lidar_age_sec || 0).toFixed(2) + ' s';
      document.getElementById('lidar_status').textContent = lidarActive ? 'Active' : 'Idle';
    }

    function disconnectRosbridge() {
      try { if (scanTopic) scanTopic.unsubscribe(); } catch (_) {}
      try { if (odomTopic) odomTopic.unsubscribe(); } catch (_) {}
      try { if (poseTopic) poseTopic.unsubscribe(); } catch (_) {}
      try { if (diagnosticsTopic) diagnosticsTopic.unsubscribe(); } catch (_) {}
      scanTopic = null;
      odomTopic = null;
      poseTopic = null;
      diagnosticsTopic = null;
      if (ros) {
        try { ros.close(); } catch (_) {}
      }
      ros = null;
      rosbridgeConnected = false;
      updateRosbridgeIndicators();
    }

    function connectRosbridge(userInitiated) {
      const input = document.getElementById('rosbridge_url_input');
      const url = (input && input.value ? input.value : defaultRosbridgeUrl).trim();
      if (input) {
        input.value = url;
      }
      disconnectRosbridge();
      if (!window.ROSLIB) {
        if (userInitiated) {
          document.getElementById('log').textContent = 'roslibjs failed to load.';
        }
        return;
      }
      ros = new ROSLIB.Ros({url});
      ros.on('connection', () => {
        rosbridgeConnected = true;
        updateRosbridgeIndicators();
        document.getElementById('log').textContent = 'Connected to rosbridge at ' + url;
        scanTopic = new ROSLIB.Topic({ros: ros, name: '/scan', messageType: 'sensor_msgs/LaserScan'});
        scanTopic.subscribe(msg => {
          const points = [];
          let angle = msg.angle_min || 0;
          for (const value of (msg.ranges || [])) {
            if (Number.isFinite(value) && value > 0) {
              points.push({a: angle, r: value});
            }
            angle += msg.angle_increment || 0;
          }
          latestRosLidarPointCount = points.length;
          latestRosLidarAgeSec = 0;
          drawLidar(points.filter((_, idx) => idx % Math.max(1, Math.floor(points.length / 360) || 1) === 0));
          updateRosbridgeIndicators();
        });
        odomTopic = new ROSLIB.Topic({ros: ros, name: '/odom', messageType: 'nav_msgs/Odometry'});
        odomTopic.subscribe(msg => {
          const linear = (((msg || {}).twist || {}).twist || {}).linear || {};
          latestOdomSpeed = Math.sqrt((linear.x || 0) ** 2 + (linear.y || 0) ** 2 + (linear.z || 0) ** 2);
          updateRosbridgeIndicators();
        });
        poseTopic = new ROSLIB.Topic({ros: ros, name: '/amcl_pose', messageType: 'geometry_msgs/PoseWithCovarianceStamped'});
        poseTopic.subscribe(msg => {
          const pose = (((msg || {}).pose || {}).pose || {});
          const position = pose.position || {};
          const orientation = pose.orientation || {};
          latestPoseX = position.x || 0;
          latestPoseY = position.y || 0;
          latestPoseThetaDeg = quaternionToYawDegrees(orientation);
          updateRosbridgeIndicators();
        });
      });
      ros.on('error', error => {
        rosbridgeConnected = false;
        updateRosbridgeIndicators();
        if (userInitiated) {
          document.getElementById('log').textContent = 'rosbridge error: ' + error;
        }
      });
      ros.on('close', () => {
        rosbridgeConnected = false;
        updateRosbridgeIndicators();
      });
    }

    function loadRoslib() {
      if (window.ROSLIB || document.querySelector('script[data-roslib="true"]')) {
        return;
      }
      const script = document.createElement('script');
      script.src = 'https://cdn.jsdelivr.net/npm/roslib/build/roslib.min.js';
      script.async = true;
      script.defer = true;
      script.dataset.roslib = 'true';
      script.onload = () => connectRosbridge(false);
      script.onerror = () => {
        document.getElementById('log').textContent = 'Dashboard loaded, but roslib.js could not be loaded. ROS bridge features may stay offline.';
      };
      document.head.appendChild(script);
    }

    function drawLidar(points) {
      const canvas = document.getElementById('lidar_canvas');
      const ctx = canvas.getContext('2d');
      ctx.fillStyle = '#020617';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      const cx = canvas.width / 2;
      const cy = canvas.height / 2;
      const radius = Math.min(canvas.width, canvas.height) * 0.42;
      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 1;
      for (const ratio of [0.25, 0.5, 0.75, 1.0]) {
        ctx.beginPath();
        ctx.arc(cx, cy, radius * ratio, 0, Math.PI * 2);
        ctx.stroke();
      }
      ctx.beginPath();
      ctx.moveTo(cx - radius, cy);
      ctx.lineTo(cx + radius, cy);
      ctx.moveTo(cx, cy - radius);
      ctx.lineTo(cx, cy + radius);
      ctx.stroke();
      if (!points || points.length === 0) {
        ctx.fillStyle = '#94a3b8';
        ctx.font = '20px Arial';
        ctx.fillText('No LiDAR data', cx - 55, cy);
        return;
      }
      let maxRange = 0;
      for (const point of points) {
        if (point.r > maxRange) {
          maxRange = point.r;
        }
      }
      maxRange = Math.max(maxRange, 1.0);
      ctx.fillStyle = '#22c55e';
      for (const point of points) {
        const scale = point.r / maxRange;
        const x = cx + Math.cos(point.a) * radius * scale;
        const y = cy - Math.sin(point.a) * radius * scale;
        ctx.fillRect(x, y, 3, 3);
      }
    }

    async function postJson(url, body) {
      const response = await fetch(url, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
      const data = await response.json();
      document.getElementById('log').textContent = JSON.stringify(data, null, 2);
      return data;
    }

    function currentSpeedLevel() {
      return parseFloat(document.getElementById('speed_slider').value || '1.0');
    }

    function updateSpeedReadout() {
      const value = currentSpeedLevel();
      document.getElementById('speed_readout').textContent = value.toFixed(1) + ' turns/sec';
      document.getElementById('velocity_input').value = value.toFixed(1);
    }

    function setKnob(xNorm, yNorm) {
      if (!joystickKnob || !joystickPad) {
        return;
      }
      const maxOffset = joystickRadius;
      const offsetX = xNorm * maxOffset;
      const offsetY = -yNorm * maxOffset;
      joystickKnob.style.transform = `translate(calc(-50% + ${offsetX}px), calc(-50% + ${offsetY}px))`;
    }

    function sendDrive(xNorm, yNorm) {
      const speed = currentSpeedLevel();
      postJson('/api/drive', {linear: yNorm, angular: xNorm, speed: speed});
    }

    function stopDrive() {
      joystickActive = false;
      setKnob(0, 0);
      postJson('/api/stop', {});
    }

    function quickDrive(xNorm, yNorm) {
      setKnob(xNorm * 0.75, yNorm * 0.75);
      sendDrive(xNorm, yNorm);
    }

    function attachJoystick() {
      joystickPad = document.getElementById('joystick_pad');
      joystickKnob = document.getElementById('joystick_knob');
      if (!joystickPad || !joystickKnob) {
        return;
      }
      const compute = () => {
        joystickRadius = Math.max(20, (joystickPad.clientWidth - joystickKnob.clientWidth) / 2);
      };
      compute();
      window.addEventListener('resize', compute);
      const updateFromEvent = event => {
        const touch = event.touches ? event.touches[0] : event;
        const rect = joystickPad.getBoundingClientRect();
        const dx = touch.clientX - (rect.left + rect.width / 2);
        const dy = touch.clientY - (rect.top + rect.height / 2);
        const distance = Math.min(Math.sqrt(dx * dx + dy * dy), joystickRadius);
        const angle = Math.atan2(dy, dx);
        const clampedX = distance * Math.cos(angle);
        const clampedY = distance * Math.sin(angle);
        const xNorm = joystickRadius > 0 ? clampedX / joystickRadius : 0;
        const yNorm = joystickRadius > 0 ? -clampedY / joystickRadius : 0;
        setKnob(xNorm, yNorm);
        sendDrive(xNorm, yNorm);
      };
      const start = event => {
        joystickActive = true;
        event.preventDefault();
        updateFromEvent(event);
      };
      const move = event => {
        if (!joystickActive) {
          return;
        }
        event.preventDefault();
        updateFromEvent(event);
      };
      const end = event => {
        if (!joystickActive) {
          return;
        }
        event.preventDefault();
        stopDrive();
      };
      joystickPad.addEventListener('mousedown', start);
      joystickPad.addEventListener('touchstart', start, {passive: false});
      window.addEventListener('mousemove', move, {passive: false});
      window.addEventListener('touchmove', move, {passive: false});
      window.addEventListener('mouseup', end);
      window.addEventListener('touchend', end);
      window.addEventListener('touchcancel', end);
      setKnob(0, 0);
    }

    async function refreshStatus() {
      const response = await fetch('/api/status');
      const data = await response.json();
      document.getElementById('axis0_state').textContent = data.axis0_state;
      document.getElementById('axis1_state').textContent = data.axis1_state;
      document.getElementById('axis0_turns').textContent = data.axis0_pos_estimate.toFixed(3);
      document.getElementById('axis1_turns').textContent = data.axis1_pos_estimate.toFixed(3);
      document.getElementById('axis0_velocity').textContent = data.axis0_vel_estimate.toFixed(3);
      document.getElementById('axis1_velocity').textContent = data.axis1_vel_estimate.toFixed(3);
      document.getElementById('robot_turns').textContent = data.robot_turns.toFixed(3);
      document.getElementById('robot_distance').textContent = data.robot_distance_m.toFixed(3) + ' m';
      document.getElementById('lidar_points').textContent = data.lidar_point_count ?? 0;
      document.getElementById('lidar_age').textContent = data.lidar_age_sec.toFixed(2);
      document.getElementById('camera_url_input').value = data.camera_url || '';
      document.getElementById('rosbridge_url_input').value = document.getElementById('rosbridge_url_input').value || defaultRosbridgeUrl;
      document.getElementById('camera_feed').src = data.camera_url || '';
      updateSystemSummary(data);
      updateRosbridgeIndicators();
    }

    async function refreshLidar() {
      if (rosbridgeConnected) {
        latestRosLidarAgeSec += 0.3;
        updateRosbridgeIndicators();
        return;
      }
      const response = await fetch('/api/lidar');
      const data = await response.json();
      drawLidar(data.points || []);
    }

    function setVelocity(direction) {
      const velocity = parseFloat(document.getElementById('velocity_input').value || '0');
      postJson('/api/velocity', {velocity: direction * velocity});
    }

    function setCameraUrl() {
      const url = document.getElementById('camera_url_input').value || '';
      postJson('/api/camera', {url: url});
    }

    function clearCameraUrl() {
      postJson('/api/camera', {url: ''});
    }

    updateSpeedReadout();
    attachJoystick();
    refreshStatus();
    refreshLidar();
    loadRoslib();
    setInterval(refreshStatus, 500);
    setInterval(refreshLidar, 300);
  </script>
</body>
</html>
"""


class SensorState:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._lidar_points: List[Dict[str, float]] = []
        self._lidar_stamp = 0.0
        self.camera_url = DEFAULT_CAMERA_URL
        self._ros_started = False
        self._executor = None
        self._node = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if rclpy is None or Node is None or LaserScan is None or SingleThreadedExecutor is None:
            return
        with self._lock:
            if self._ros_started:
                return
            rclpy.init(args=None)
            self._executor = SingleThreadedExecutor()
            self._node = LidarSubscriberNode(self)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
            self._ros_started = True

    def _spin(self) -> None:
        while rclpy is not None and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                break

    def stop(self) -> None:
        with self._lock:
            if not self._ros_started:
                return
            try:
                if self._executor is not None and self._node is not None:
                    self._executor.remove_node(self._node)
                if self._node is not None:
                    self._node.destroy_node()
            finally:
                try:
                    if rclpy is not None and rclpy.ok():
                        rclpy.shutdown()
                except Exception:
                    pass
                self._ros_started = False

    def update_lidar(self, msg: Any) -> None:
        points: List[Dict[str, float]] = []
        angle = float(msg.angle_min)
        for value in msg.ranges:
            distance = float(value)
            if math.isfinite(distance) and distance > 0.0:
                points.append({'a': angle, 'r': distance})
            angle += float(msg.angle_increment)
        with self._lock:
            self._lidar_points = points[:: max(1, len(points) // 360 or 1)]
            self._lidar_stamp = time.time()

    def get_lidar_payload(self) -> Dict[str, Any]:
        with self._lock:
            return {
                'points': list(self._lidar_points),
                'point_count': len(self._lidar_points),
                'age_sec': max(0.0, time.time() - self._lidar_stamp) if self._lidar_stamp else 0.0,
            }

    def get_camera_url(self) -> str:
        with self._lock:
            return self.camera_url

    def set_camera_url(self, url: str) -> None:
        with self._lock:
            self.camera_url = str(url).strip()


if Node is not None and LaserScan is not None:
    class LidarSubscriberNode(Node):
        def __init__(self, state: 'SensorState') -> None:
            super().__init__('odrive_dashboard_lidar_listener')
            self._state = state
            self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        def _on_scan(self, msg: LaserScan) -> None:
            self._state.update_lidar(msg)


class ODriveController:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._motion_thread: Optional[threading.Thread] = None
        self._stop_motion = threading.Event()
        self.odrv = None
        self.axis0 = None
        self.axis1 = None
        self.last_message = 'Controller initialized.'

    def connect(self) -> None:
        if odrive is None:
            raise RuntimeError('The odrive Python package is not installed in the current environment.')
        with self._lock:
            self.odrv = odrive.find_any(timeout=10)
            self.axis0 = self.odrv.axis0
            self.axis1 = self.odrv.axis1
            self.last_message = f'Connected to ODrive {self.odrv.serial_number}.'

    def ensure_connected(self) -> None:
        with self._lock:
            if self.odrv is None or self.axis0 is None or self.axis1 is None:
                self.connect()

    def _apply_axis_config(self, axis: Any) -> None:
        axis.motor.config.pole_pairs = 15
        axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        axis.motor.config.calibration_current = 8
        axis.motor.config.resistance_calib_max_voltage = 4
        axis.encoder.config.mode = 1
        axis.encoder.config.cpr = 90
        axis.encoder.config.bandwidth = 100

    def apply_axis0_config(self) -> None:
        self.ensure_connected()
        with self._lock:
            self._apply_axis_config(self.axis0)
            self._apply_axis_config(self.axis1)
            self.last_message = 'Applied hoverboard motor and hall encoder configuration to both axes.'

    def run_calibration(self) -> None:
        self.ensure_connected()
        with self._lock:
            self.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.last_message = 'Started full calibration sequence on both axes.'

    def _enter_velocity_mode(self, axis: Any) -> None:
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def set_velocity(self, turns_per_sec: float) -> None:
        self.ensure_connected()
        with self._lock:
            self._enter_velocity_mode(self.axis0)
            self._enter_velocity_mode(self.axis1)
            self.axis0.controller.input_vel = float(turns_per_sec)
            self.axis1.controller.input_vel = float(-turns_per_sec)
            self.last_message = f'Set robot velocity to {turns_per_sec:.3f} turns/sec.'

    def drive(self, linear: float, angular: float, speed: float) -> None:
        self.ensure_connected()
        with self._lock:
            max_speed = max(0.0, float(speed))
            if max_speed <= 0.0:
                self.axis0.controller.input_vel = 0.0
                self.axis1.controller.input_vel = 0.0
                self.last_message = 'Drive speed was zero, stopping robot.'
                return
            linear_cmd = max(-1.0, min(1.0, float(linear)))
            angular_cmd = max(-1.0, min(1.0, float(angular)))
            axis0_vel = (linear_cmd + angular_cmd) * max_speed
            axis1_forward = (linear_cmd - angular_cmd) * max_speed
            self._enter_velocity_mode(self.axis0)
            self._enter_velocity_mode(self.axis1)
            self.axis0.controller.input_vel = axis0_vel
            self.axis1.controller.input_vel = -axis1_forward
            self.last_message = (
                f'Drive command linear={linear_cmd:.2f} angular={angular_cmd:.2f} speed={max_speed:.2f}.'
            )

    def stop(self) -> None:
        with self._lock:
            if self.axis0 is not None:
                try:
                    self.axis0.controller.input_vel = 0.0
                except Exception:
                    pass
            if self.axis1 is not None:
                try:
                    self.axis1.controller.input_vel = 0.0
                except Exception:
                    pass
            self._stop_motion.set()
            self.last_message = 'Stop requested.'

    def move_rotations(self, rotations: float, velocity: float) -> None:
        self.ensure_connected()
        self.stop()
        self._stop_motion = threading.Event()

        def worker() -> None:
            try:
                with self._lock:
                    start_pos0 = float(self.axis0.encoder.pos_estimate)
                    start_pos1 = float(self.axis1.encoder.pos_estimate)
                    target_pos0 = start_pos0 + rotations
                    target_pos1 = start_pos1 - rotations
                    direction = 1.0 if rotations >= 0.0 else -1.0
                    base_velocity = abs(float(velocity))
                    if base_velocity <= 0.0:
                        raise RuntimeError('Velocity must be greater than zero.')
                    self._enter_velocity_mode(self.axis0)
                    self._enter_velocity_mode(self.axis1)
                    self.axis0.controller.input_vel = direction * base_velocity
                    self.axis1.controller.input_vel = -direction * base_velocity
                    self.last_message = (
                        f'Moving robot {rotations:.3f} wheel rotations at {base_velocity:.3f} turns/sec.'
                    )

                while not self._stop_motion.is_set():
                    with self._lock:
                        current_pos0 = float(self.axis0.encoder.pos_estimate)
                        current_pos1 = float(self.axis1.encoder.pos_estimate)
                        remaining0 = target_pos0 - current_pos0
                        remaining1 = target_pos1 - current_pos1
                        done0 = remaining0 <= 0.0 if direction > 0.0 else remaining0 >= 0.0
                        done1 = remaining1 >= 0.0 if direction > 0.0 else remaining1 <= 0.0
                        done = done0 and done1
                        if done:
                            self.axis0.controller.input_vel = 0.0
                            self.axis1.controller.input_vel = 0.0
                            self.last_message = f'Completed robot move of {rotations:.3f} wheel rotations.'
                            break
                        if abs(remaining0) < 1.0 or abs(remaining1) < 1.0:
                            reduced_velocity = direction * min(base_velocity, 0.2)
                            self.axis0.controller.input_vel = reduced_velocity
                            self.axis1.controller.input_vel = -reduced_velocity
                    time.sleep(0.01)
                else:
                    with self._lock:
                        self.axis0.controller.input_vel = 0.0
                        self.axis1.controller.input_vel = 0.0
                        self.last_message = 'Motion stopped before reaching target.'
            except Exception as exc:
                with self._lock:
                    self.last_message = f'Motion error: {exc}'
                try:
                    self.axis0.controller.input_vel = 0.0
                except Exception:
                    pass
                try:
                    self.axis1.controller.input_vel = 0.0
                except Exception:
                    pass

        self._motion_thread = threading.Thread(target=worker, daemon=True)
        self._motion_thread.start()

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            connected = self.odrv is not None and self.axis0 is not None and self.axis1 is not None
            if not connected:
                return {
                    'connected': False,
                    'odrive_serial': '',
                    'bus_voltage': None,
                    'battery_percent': None,
                    'axis0_state': 'disconnected',
                    'axis1_state': 'disconnected',
                    'axis0_pos_estimate': 0.0,
                    'axis1_pos_estimate': 0.0,
                    'axis0_vel_estimate': 0.0,
                    'axis1_vel_estimate': 0.0,
                    'robot_turns': 0.0,
                    'robot_distance_m': 0.0,
                    'message': self.last_message,
                }
            try:
                axis0_pos_estimate = float(self.axis0.encoder.pos_estimate)
                axis1_pos_estimate = float(self.axis1.encoder.pos_estimate)
                axis0_vel_estimate = float(self.axis0.encoder.vel_estimate)
                axis1_vel_estimate = float(self.axis1.encoder.vel_estimate)
                axis0_state = int(self.axis0.current_state)
                axis1_state = int(self.axis1.current_state)
            except Exception as exc:
                return {
                    'connected': False,
                    'odrive_serial': '',
                    'bus_voltage': None,
                    'battery_percent': None,
                    'axis0_state': f'error: {exc}',
                    'axis1_state': f'error: {exc}',
                    'axis0_pos_estimate': 0.0,
                    'axis1_pos_estimate': 0.0,
                    'axis0_vel_estimate': 0.0,
                    'axis1_vel_estimate': 0.0,
                    'robot_turns': 0.0,
                    'robot_distance_m': 0.0,
                    'message': self.last_message,
                }
            robot_turns = (axis0_pos_estimate - axis1_pos_estimate) / 2.0
            bus_voltage = None
            battery_percent = None
            odrive_serial = ''
            try:
                bus_voltage = float(self.odrv.vbus_voltage)
                battery_percent = max(0.0, min(100.0, ((bus_voltage - 33.0) / (41.0 - 33.0)) * 100.0))
            except Exception:
                pass
            try:
                odrive_serial = str(self.odrv.serial_number)
            except Exception:
                pass
            return {
                'connected': True,
                'odrive_serial': odrive_serial,
                'bus_voltage': bus_voltage,
                'battery_percent': battery_percent,
                'axis0_state': axis0_state,
                'axis1_state': axis1_state,
                'axis0_pos_estimate': axis0_pos_estimate,
                'axis1_pos_estimate': axis1_pos_estimate,
                'axis0_vel_estimate': axis0_vel_estimate,
                'axis1_vel_estimate': axis1_vel_estimate,
                'robot_turns': robot_turns,
                'robot_distance_m': robot_turns * WHEEL_CIRCUMFERENCE_METERS,
                'message': self.last_message,
            }


CONTROLLER = ODriveController()
SENSORS = SensorState()


class DashboardHandler(BaseHTTPRequestHandler):
    def _send_json(self, payload: Dict[str, Any], status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get('Content-Length', '0'))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length).decode('utf-8'))

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path == '/':
            body = HTML_PAGE.encode('utf-8')
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == '/api/status':
            status = CONTROLLER.get_status()
            lidar = SENSORS.get_lidar_payload()
            status['lidar_point_count'] = lidar['point_count']
            status['lidar_age_sec'] = lidar['age_sec']
            status['camera_url'] = SENSORS.get_camera_url()
            self._send_json(status)
            return
        if parsed.path == '/api/lidar':
            self._send_json(SENSORS.get_lidar_payload())
            return
        self._send_json({'error': 'Not found'}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        try:
            payload = self._read_json()
            if parsed.path == '/api/connect':
                CONTROLLER.connect()
                self._send_json(CONTROLLER.get_status())
                return
            if parsed.path == '/api/configure':
                CONTROLLER.apply_axis0_config()
                self._send_json(CONTROLLER.get_status())
                return
            if parsed.path == '/api/calibrate':
                CONTROLLER.run_calibration()
                self._send_json(CONTROLLER.get_status())
                return
            if parsed.path == '/api/velocity':
                CONTROLLER.set_velocity(float(payload.get('velocity', 0.0)))
                self._send_json(CONTROLLER.get_status())
                return
            if parsed.path == '/api/drive':
                CONTROLLER.drive(
                    float(payload.get('linear', 0.0)),
                    float(payload.get('angular', 0.0)),
                    float(payload.get('speed', 0.0)),
                )
                self._send_json(CONTROLLER.get_status())
                return
            if parsed.path == '/api/move':
                CONTROLLER.move_rotations(
                    float(payload.get('rotations', 0.0)),
                    float(payload.get('velocity', 0.0)),
                )
                self._send_json(CONTROLLER.get_status())
                return
            if parsed.path == '/api/stop':
                CONTROLLER.stop()
                self._send_json(CONTROLLER.get_status())
                return
            if parsed.path == '/api/camera':
                SENSORS.set_camera_url(str(payload.get('url', '')))
                status = CONTROLLER.get_status()
                status['camera_url'] = SENSORS.get_camera_url()
                lidar = SENSORS.get_lidar_payload()
                status['lidar_point_count'] = lidar['point_count']
                status['lidar_age_sec'] = lidar['age_sec']
                self._send_json(status)
                return
            self._send_json({'error': 'Not found'}, HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self._send_json({'error': str(exc)}, HTTPStatus.BAD_REQUEST)

    def log_message(self, format: str, *args: Any) -> None:
        return


def main() -> None:
    SENSORS.start()
    server = ThreadingHTTPServer((DEFAULT_HOST, DEFAULT_PORT), DashboardHandler)
    print(f'ODrive dashboard listening on http://{DEFAULT_HOST}:{DEFAULT_PORT}')
    print('Open http://127.0.0.1:8081 in your browser.')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        CONTROLLER.stop()
        SENSORS.stop()
        server.server_close()


if __name__ == '__main__':
    main()
