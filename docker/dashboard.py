#!/usr/bin/env python3
"""
AGX ROS 1 Dashboard - 零依賴 Web 控制面板
啟動: python3 dashboard.py [--port 8081]
"""

import http.server
import json
import subprocess
import os
import sys
import argparse
import threading
import time
from urllib.parse import urlparse, parse_qs

# --- Configuration ---
PROJECT_NAME = "agx_ros_1"
SERVICES = {
    # ROS 1 Services
    "roscore":    {"name": "ROS 1 Master",     "container": "roscore",    "icon": "🖥️", "service": "roscore"},
    "control":    {"name": "ROS 1 底層控制",   "container": "control",    "icon": "🎮", "service": "control"},
    "foxglove_ros1": {"name": "Foxglove Bridge",  "container": "foxglove_ros1", "icon": "📊", "service": "foxglove_ros1"},
}

TASKS = {
    "control": {
        "label": "Control Tasks (ROS 1)",
        "container": "control",
        "items": {
            "agx_bringup":  {"name": "機器人啟動", "cmd": "rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0", "icon": "🚀"},
            "agx_keyboard": {"name": "鍵盤控制",   "cmd": "rosrun six_wheels_teleop imu_teletop_0908", "icon": "⌨️"},
            "agx_lidar":    {"name": "Lidar 啟動", "cmd": "roslaunch velodyne_pointcloud VLP16_points.launch", "icon": "📡"},
            "agx_camera":   {"name": "Realsense",  "cmd": "roslaunch realsense2_camera rs_camera.launch", "icon": "📷"},
            "agx_hdl":      {"name": "HDL 定位",   "cmd": "roslaunch hdl_localization hdl_localization.launch", "icon": "📍"},
            "agx_imu":      {"name": "IMU 濾波",   "cmd": "rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _remove_gravity_vector:=true _output_rate:=100.0 /imu/data_raw:=/camera/imu", "icon": "🧹"},
            "agx_collect" : {"name": "數據收集",   "cmd": "rosrun six_wheels_teleop inference_collect", "icon": "📥"},
            "agx_RL":       {"name": "RL推論", "cmd": "python3 /root/keyboard_control_ws/src/six_wheels_teleop/src/inference_ros1.py", "icon": "🤖"},
        }
    }
}

# --- Detect environment ---
def detect_env():
    """Auto-detect AGX vs PC mode"""
    try:
        ctx = subprocess.check_output(["docker", "context", "show"], text=True).strip()
        if "agx" in ctx:
            return "agx", "../.env.agx"
    except Exception:
        pass
    try:
        arch = subprocess.check_output(["uname", "-m"], text=True).strip()
        if arch == "aarch64":
            return "agx", "../.env.agx"
    except Exception:
        pass
    return "pc", "../.env"

MODE, ENV_FILE = detect_env()
COMPOSE_FILE = "docker-compose.yaml"
# If AGX mode, we might need additional compose file (based on Makefile)
EXTRA_COMPOSE = "docker-compose.agx.yaml" if MODE == "agx" else ""

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
cmd_process_pipe = None

# ROS 1 Publisher Script (Base64 encoded later)
PY_PUB_SCRIPT = """
import rospy
from geometry_msgs.msg import Twist
import sys
import json
try:
    rospy.init_node('web_teleop', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    while not rospy.is_shutdown():
        line = sys.stdin.readline()
        if not line: break
        try:
            d = json.loads(line)
            t = Twist()
            t.linear.x = float(d.get('lx', 0.0))
            t.angular.z = float(d.get('az', 0.0))
            pub.publish(t)
        except Exception:
            pass
except Exception as e:
    sys.stderr.write(str(e))
"""

def ensure_cmd_process():
    global cmd_process_pipe
    if cmd_process_pipe is None or cmd_process_pipe.poll() is not None:
        import base64
        encoded = base64.b64encode(PY_PUB_SCRIPT.encode('utf-8')).decode('utf-8')
        # Use 'control' container for ROS 1 tasks
        cmd = [
            'docker', 'exec', '-i', 'control', 'bash', '-c',
            f'source /opt/ros/noetic/setup.bash && python3 -c "$(echo {encoded} | base64 -d)"'
        ]
        try:
            cmd_process_pipe = subprocess.Popen(cmd, stdin=subprocess.PIPE, text=True)
            print("[Info] Started persistent ROS 1 publisher process.")
        except Exception as e:
            print(f"[Error] Failed to start persistent publisher: {e}")

def run_cmd(cmd, timeout=30):
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True,
            timeout=timeout, cwd=PROJECT_DIR
        )
        return {
            "ok": result.returncode == 0,
            "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip(),
        }
    except subprocess.TimeoutExpired:
        return {"ok": False, "stdout": "", "stderr": "Command timed out"}
    except Exception as e:
        return {"ok": False, "stdout": "", "stderr": str(e)}

def get_container_status():
    result = run_cmd(
        f'docker ps -a --filter "label=com.docker.compose.project={PROJECT_NAME}" '
        f'--format "{{{{.Names}}}}|{{{{.Status}}}}|{{{{.State}}}}"'
    )
    statuses = {}
    if result["ok"] and result["stdout"]:
        for line in result["stdout"].splitlines():
            parts = line.split("|")
            if len(parts) >= 3:
                # Docker Compose names are usually PROJECT_NAME-SERVICE-1
                # We extract the service name part
                name = parts[0]
                statuses[name] = {"status": parts[1], "state": parts[2]}
                # Also map by service name for easier lookup
                for svc_id, svc_info in SERVICES.items():
                    if svc_info["container"] in name:
                         statuses[svc_id] = {"status": parts[1], "state": parts[2]}
    return statuses

def get_tmux_sessions():
    result = run_cmd('tmux ls -F "#{session_name}" 2>/dev/null')
    if result["ok"] and result["stdout"]:
        return [s for s in result["stdout"].splitlines()
                if s.startswith("agx_")]
    return []

def compose_cmd_for():
    # Base command
    cmd = f'docker compose --env-file {ENV_FILE} -f {COMPOSE_FILE}'
    if EXTRA_COMPOSE:
        cmd += f' -f {EXTRA_COMPOSE}'
    cmd += f' -p {PROJECT_NAME}'
    return cmd

# --- API Handlers ---
def handle_api(path, params):
    action = path.replace("/api/", "")

    if action == "status":
        containers = get_container_status()
        sessions = get_tmux_sessions()
        return {"containers": containers, "sessions": sessions, "mode": MODE}

    elif action == "service/all-up":
        return run_cmd(f'{compose_cmd_for()} up -d', timeout=120)

    elif action == "service/all-down":
        return run_cmd(f'{compose_cmd_for()} down --remove-orphans', timeout=60)

    elif action.startswith("service/"):
        svc_action = action.split("/")[-1]
        service_id = params.get("folder", [None])[0]
        if not service_id or service_id not in SERVICES:
            return {"ok": False, "error": "Invalid service"}
        service_name = SERVICES[service_id].get("service", service_id)
        
        if svc_action == "up":
            return run_cmd(f'{compose_cmd_for()} up -d {service_name}', timeout=120)
        elif svc_action == "down":
            return run_cmd(f'{compose_cmd_for()} rm -s -v -f {service_name}', timeout=60)
        elif svc_action == "build":
            return run_cmd(f'{compose_cmd_for()} build {service_name}', timeout=600)
        elif svc_action == "rebuild":
            return run_cmd(f'{compose_cmd_for()} up -d --build --force-recreate {service_name}', timeout=600)

    elif action == "task/launch":
        task_id = params.get("task", [None])[0]
        group = params.get("group", [None])[0]
        if not task_id or not group or group not in TASKS:
            return {"ok": False, "error": "Invalid task"}
        task_info = TASKS[group]["items"].get(task_id)
        if not task_info:
            return {"ok": False, "error": "Task not found"}
        container = TASKS[group]["container"]
        sessions = get_tmux_sessions()
        if task_id in sessions:
            return {"ok": True, "stdout": f"Task '{task_id}' is already running."}
        cmd = task_info["cmd"]
        run_cmd(f'tmux new-session -d -s {task_id}')
        time.sleep(0.5)
        run_cmd(f'tmux send-keys -t {task_id}:0 "docker exec -it {container} bash -ic \'{cmd}\'" C-m')
        return {"ok": True, "stdout": f"Task '{task_id}' launched."}

    elif action == "task/stop":
        task_id = params.get("task", [None])[0]
        if task_id == "all":
            sessions = get_tmux_sessions()
            for s in sessions:
                run_cmd(f'tmux kill-session -t {s}')
            return {"ok": True, "stdout": f"Stopped {len(sessions)} tasks."}
        if not task_id:
            return {"ok": False, "error": "No task specified"}
        return run_cmd(f'tmux kill-session -t {task_id}')

    elif action == "task/logs":
        task_id = params.get("task", [None])[0]
        if not task_id:
            return {"ok": False, "error": "No task specified"}
        # Capture the last 100 lines from the tmux session's pane
        return run_cmd(f'tmux capture-pane -pt {task_id} -S -100')

    elif action == "logs":
        folder = params.get("folder", [None])[0]
        lines = params.get("lines", ["50"])[0]
        if folder and folder in SERVICES:
            container = SERVICES[folder]["container"]
            # We need to find the actual container name which might have suffixes
            r_ps = run_cmd(f'docker ps --filter "label=com.docker.compose.service={folder}" --format "{{{{.Names}}}}"')
            c_name = r_ps["stdout"].split('\n')[0] if r_ps["ok"] and r_ps["stdout"] else container
            return run_cmd(f'docker logs --tail {lines} {c_name}', timeout=10)
        else:
            return run_cmd(f'{compose_cmd_for()} logs --tail {lines}', timeout=10)

    elif action == "cmd_vel":
        lx = params.get("lx", ["0.0"])[0]
        az = params.get("az", ["0.0"])[0]
        ensure_cmd_process()
        global cmd_process_pipe
        if cmd_process_pipe and cmd_process_pipe.poll() is None:
            try:
                cmd_process_pipe.stdin.write(json.dumps({"lx": lx, "az": az}) + "\n")
                cmd_process_pipe.stdin.flush()
                return {"ok": True}
            except Exception as e:
                return {"ok": False, "error": str(e)}
        else:
            return {"ok": False, "error": "Control container not running or ROS 1 failing"}

    return {"ok": False, "error": "Unknown action"}

# --- HTML Dashboard (Built-in) ---
DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AGX ROS 1 Dashboard</title>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-primary: #0a0b10;
            --bg-secondary: #12141c;
            --bg-card: rgba(25, 28, 40, 0.7);
            --bg-hover: rgba(35, 40, 60, 0.8);
            --border: rgba(255, 255, 255, 0.1);
            --accent: #3b82f6;
            --accent-glow: rgba(59, 130, 246, 0.4);
            --success: #10b981;
            --danger: #ef4444;
            --warning: #f59e0b;
            --text-main: #f8fafc;
            --text-dim: #94a3b8;
            --radius: 12px;
        }
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Inter', system-ui, sans-serif;
            background: var(--bg-primary);
            color: var(--text-main);
            background-image: radial-gradient(circle at 50% 0%, #1e293b 0%, var(--bg-primary) 70%);
            min-height: 100vh;
        }
        .header {
            padding: 20px 40px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            backdrop-filter: blur(10px);
            border-bottom: 1px solid var(--border);
            position: sticky; top: 0; z-index: 100;
        }
        .header h1 { font-size: 1.5rem; font-weight: 700; display: flex; align-items: center; gap: 12px; }
        .mode-badge {
            font-size: 0.7rem; font-weight: 800; padding: 4px 10px; border-radius: 20px;
            text-transform: uppercase; border: 1px solid currentColor;
        }
        .mode-pc { color: var(--accent); }
        .mode-agx { color: #a855f7; }

        .container { max-width: 1200px; margin: 0 auto; padding: 30px; }
        .section-header {
            display: flex; align-items: center; gap: 15px; margin-bottom: 20px; margin-top: 40px;
            color: var(--text-dim); text-transform: uppercase; font-size: 0.8rem; letter-spacing: 2px;
        }
        .section-header::after { content: ''; flex: 1; height: 1px; background: var(--border); }

        .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(350px, 1fr)); gap: 20px; }
        .card {
            background: var(--bg-card); border: 1px solid var(--border); border-radius: var(--radius);
            padding: 20px; transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
            backdrop-filter: blur(5px);
        }
        .card:hover { transform: translateY(-5px); border-color: var(--accent); box-shadow: 0 10px 30px -10px var(--accent-glow); }
        .card.running { border-left: 4px solid var(--success); }
        
        .card-top { display: flex; justify-content: space-between; align-items: center; margin-bottom: 15px; }
        .card-info { display: flex; align-items: center; gap: 12px; }
        .card-icon { font-size: 1.5rem; }
        .card-name { font-weight: 600; font-size: 1.1rem; }
        .card-status-text { font-size: 0.75rem; color: var(--text-dim); margin-bottom: 20px; }

        .actions { display: flex; gap: 8px; flex-wrap: wrap; }
        .btn {
            background: var(--bg-secondary); border: 1px solid var(--border); color: var(--text-main);
            padding: 8px 16px; border-radius: 8px; cursor: pointer; font-size: 0.85rem; font-weight: 500;
            transition: all 0.2s; display: flex; align-items: center; gap: 6px;
        }
        .btn:hover:not(:disabled) { background: var(--bg-hover); border-color: var(--text-dim); }
        .btn:disabled { opacity: 0.3; cursor: not-allowed; }
        .btn-success { color: var(--success); border-color: rgba(16, 185, 129, 0.3); }
        .btn-success:hover:not(:disabled) { background: rgba(16, 185, 129, 0.1); }
        .btn-danger { color: var(--danger); border-color: rgba(239, 68, 68, 0.3); }
        .btn-danger:hover:not(:disabled) { background: rgba(239, 68, 68, 0.1); }
        .btn-primary { background: var(--accent); border: none; }
        .btn-primary:hover:not(:disabled) { filter: brightness(1.1); }

        .task-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(280px, 1fr)); gap: 15px; }
        .task-card {
            background: var(--bg-secondary); border: 1px solid var(--border); border-radius: 10px;
            padding: 15px; display: flex; justify-content: space-between; align-items: center;
        }
        .task-card.active { border-color: var(--success); background: rgba(16, 185, 129, 0.05); }

        /* Joystick UI */
        .controller {
            background: var(--bg-card); border: 1px solid var(--border); border-radius: var(--radius);
            padding: 30px; margin-top: 40px; text-align: center;
        }
        .joystick-grid {
            display: inline-grid; grid-template-columns: repeat(3, 80px); gap: 15px; margin: 20px auto;
        }
        .joy-btn {
            width: 80px; height: 80px; border-radius: 15px; background: var(--bg-secondary);
            border: 2px solid var(--border); color: var(--text-main); font-size: 1.2rem;
            cursor: pointer; display: flex; flex-direction: column; justify-content: center; align-items: center;
            transition: all 0.1s; user-select: none;
        }
        .joy-btn:active, .joy-btn.active { background: var(--accent); border-color: var(--accent); transform: scale(0.9); }
        .joy-btn.joy-stop { color: var(--danger); border-color: var(--danger); }
        .joy-btn.joy-stop:active { background: var(--danger); color: white; }
        .joy-label { font-size: 0.6rem; margin-top: 4px; color: var(--text-dim); }

        .speed-bar { margin: 20px 0; display: flex; align-items: center; justify-content: center; gap: 15px; }
        input[type=range] { width: 200px; accent-color: var(--accent); }

        #logs {
            background: #000; color: #a0ff80; padding: 20px; border-radius: 10px;
            font-family: monospace; font-size: 0.8rem; max-height: 400px; overflow-y: auto;
            margin-top: 20px; display: none; white-space: pre-wrap;
        }
        #logs.show { display: block; }

        .toast-container { position: fixed; bottom: 20px; right: 20px; z-index: 1000; display: flex; flex-direction: column; gap: 10px; }
        .toast {
            background: var(--bg-secondary); border: 1px solid var(--border); padding: 12px 24px;
            border-radius: 8px; font-size: 0.9rem; box-shadow: 0 10px 20px rgba(0,0,0,0.4);
            animation: slideIn 0.3s ease-out; border-left: 4px solid var(--accent);
        }
        @keyframes slideIn { from { transform: translateX(100%); opacity: 0; } to { transform: translateX(0); opacity: 1; } }
    </style>
</head>
<body>

<div class="header">
    <h1><span>🤖</span> AGX ROS 1 Dashboard</h1>
    <div>
        <span id="mode-label" class="mode-badge">PC</span>
    </div>
</div>

<div class="container">
    <div class="actions" style="margin-bottom: 20px;">
        <button class="btn btn-primary" onclick="apiCall('service/all-up')">▶ 全部啟動</button>
        <button class="btn btn-danger" onclick="apiCall('service/all-down')">⏹ 全部停止</button>
        <button class="btn" onclick="refresh()">🔄 重新整理</button>
    </div>

    <div class="section-header">核心服務</div>
    <div class="grid" id="service-grid"></div>

    <div class="section-header">自動化任務 (Tmux)</div>
    <div class="task-grid" id="task-grid"></div>

    <div class="section-header">虛擬搖桿</div>
    <div class="controller">
        <div class="speed-bar">
            <span>速度:</span>
            <input type="range" id="speed-slider" min="0.1" max="1.5" step="0.1" value="0.5">
            <span id="speed-val">0.5 m/s</span>
        </div>
        <div class="joystick-grid">
            <div></div>
            <button class="joy-btn" id="btn-w" data-key="w">W <span class="joy-label">前進</span></button>
            <div></div>
            <button class="joy-btn" id="btn-a" data-key="a">A <span class="joy-label">左轉</span></button>
            <button class="joy-btn joy-stop" onclick="stopMove()">⏹ <span class="joy-label">停止</span></button>
            <button class="joy-btn" id="btn-d" data-key="d">D <span class="joy-label">右轉</span></button>
            <div></div>
            <button class="joy-btn" id="btn-s" data-key="s">S <span class="joy-label">後退</span></button>
            <div></div>
        </div>
        <div id="cmd-status" style="font-size: 0.7rem; color: var(--text-dim); font-family: monospace;">Idle</div>
    </div>

    <div class="section-header">終端日誌</div>
    <div id="logs"></div>
</div>

<div class="toast-container" id="toast-container"></div>

<script>
    const SERVICES = __SERVICES_JSON__;
    const TASKS = __TASKS_JSON__;
    let statuses = {};

    async function apiCall(endpoint, params = {}) {
        const qs = new URLSearchParams(params).toString();
        try {
            const res = await fetch(`/api/${endpoint}${qs ? '?' + qs : ''}`);
            const data = await res.json();
            if (endpoint.includes('logs')) return data;
            if (!data.ok) showToast(data.stderr || data.error, 'danger');
            else if (!endpoint.includes('status')) showToast('操作成功');
            return data;
        } catch (e) {
            showToast('API Error: ' + e.message, 'danger');
            return { ok: false };
        }
    }

    function showToast(msg, type = 'info') {
        const t = document.createElement('div');
        t.className = 'toast';
        if (type === 'danger') t.style.borderLeftColor = 'var(--danger)';
        t.textContent = msg;
        document.getElementById('toast-container').appendChild(t);
        setTimeout(() => t.remove(), 3000);
    }

    function render() {
        // Services
        const sGrid = document.getElementById('service-grid');
        sGrid.innerHTML = '';
        Object.entries(SERVICES).forEach(([id, svc]) => {
            const stat = statuses.containers[id] || { state: 'stopped', status: 'Offline' };
            const card = document.createElement('div');
            card.className = `card ${stat.state === 'running' ? 'running' : ''}`;
            card.innerHTML = `
                <div class="card-top">
                    <div class="card-info">
                        <span class="card-icon">${svc.icon}</span>
                        <span class="card-name">${svc.name}</span>
                    </div>
                </div>
                <div class="card-status-text">${stat.status}</div>
                <div class="actions">
                    <button class="btn btn-success" onclick="apiCall('service/up', {folder:'${id}'})" ${stat.state==='running'?'disabled':''}>Start</button>
                    <button class="btn btn-danger" onclick="apiCall('service/down', {folder:'${id}'})" ${stat.state!=='running'?'disabled':''}>Stop</button>
                    <button class="btn" onclick="showLogs('${id}')">Logs</button>
                </div>
            `;
            sGrid.appendChild(card);
        });

        // Tasks
        const tGrid = document.getElementById('task-grid');
        tGrid.innerHTML = '';
        Object.values(TASKS).forEach(group => {
            Object.entries(group.items).forEach(([id, task]) => {
                const isActive = statuses.sessions.includes(id);
                const card = document.createElement('div');
                card.className = `task-card ${isActive ? 'active' : ''}`;
                card.innerHTML = `
                    <div style="display:flex; align-items:center; gap:10px">
                        <span>${task.icon}</span>
                        <div>
                            <div style="font-size:0.9rem; font-weight:600">${task.name}</div>
                        </div>
                    </div>
                    <div class="actions">
                    <button class="btn ${isActive?'btn-danger':'btn-success'}" onclick="apiCall('task/${isActive?'stop':'launch'}', {group:'control', task:'${id}'})">
                        ${isActive ? '⏹ 停止' : '▶ 啟動'}
                    </button>
                    ${isActive ? `<button class="btn" onclick="showTaskLogs('${id}')">📋 Log</button>` : ''}
                </div>
            `;
            tGrid.appendChild(card);
        });
    });
}

    async function showTaskLogs(id) {
        const logEl = document.getElementById('logs');
        logEl.classList.add('show');
        logEl.textContent = `[Scanning Tmux Session: ${id}...]\n`;
        
        // Polling logs for debugging
        if (window.logTimer) clearInterval(window.logTimer);
        const fetchLogs = async () => {
            const data = await apiCall('task/logs', { task: id });
            logEl.textContent = data.stdout || data.stderr || 'No output in session yet...';
            logEl.scrollTop = logEl.scrollHeight;
        };
        fetchLogs();
        window.logTimer = setInterval(fetchLogs, 2000);
    }

    async function showLogs(id) {
        if (window.logTimer) clearInterval(window.logTimer);
        const data = await apiCall('logs', { folder: id, lines: 100 });
        const logEl = document.getElementById('logs');
        logEl.textContent = data.stdout || data.stderr || 'No logs found';
        logEl.classList.add('show');
        logEl.scrollTop = logEl.scrollHeight;
    }

    async function refresh() {
        const data = await apiCall('status');
        if (data.containers) {
            statuses = data;
            const label = document.getElementById('mode-label');
            label.textContent = data.mode;
            label.className = `mode-badge mode-${data.mode}`;
            render();
        }
    }

    // Joystick Logic
    let activeKey = null;
    let driveInterval = null;
    const speedSlider = document.getElementById('speed-slider');
    const speedVal = document.getElementById('speed-val');
    speedSlider.oninput = () => speedVal.textContent = speedSlider.value + ' m/s';

    function sendDrive(key) {
        const spd = speedSlider.value;
        let lx=0, az=0;
        if (key === 'w') lx = spd;
        if (key === 's') lx = -spd;
        if (key === 'a') az = spd * 1.5;
        if (key === 'd') az = -spd * 1.5;
        document.getElementById('cmd-status').textContent = `Linear: ${lx}, Angular: ${az}`;
        fetch(`/api/cmd_vel?lx=${lx}&az=${az}`);
    }

    function startMove(key) {
        if (activeKey === key) return;
        stopMove();
        activeKey = key;
        document.getElementById(`btn-${key}`).classList.add('active');
        sendDrive(key);
        driveInterval = setInterval(() => sendDrive(key), 200);
    }

    function stopMove() {
        if (driveInterval) clearInterval(driveInterval);
        driveInterval = null;
        activeKey = null;
        document.querySelectorAll('.joy-btn').forEach(b => b.classList.remove('active'));
        document.getElementById('cmd-status').textContent = 'Idle';
        fetch(`/api/cmd_vel?lx=0&az=0`);
    }

    // Events
    document.querySelectorAll('.joy-btn[data-key]').forEach(btn => {
        const key = btn.dataset.key;
        btn.onmousedown = () => startMove(key);
        btn.ontouchstart = (e) => { e.preventDefault(); startMove(key); };
        btn.onmouseup = btn.onmouseleave = btn.ontouchend = stopMove;
    });

    document.onkeydown = (e) => {
        const k = e.key.toLowerCase();
        if (['w','a','s','d'].includes(k)) startMove(k);
        if (e.code === 'Space') stopMove();
    };
    document.onkeyup = (e) => {
        if (['w','a','s','d'].includes(e.key.toLowerCase())) stopMove();
    };

    refresh();
    setInterval(refresh, 3000);
</script>
</body>
</html>
"""

class DashboardHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        params = parse_qs(parsed.query)

        if path.startswith("/api/"):
            result = handle_api(path, params)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(result, ensure_ascii=False).encode())
        else:
            html = DASHBOARD_HTML.replace(
                "__SERVICES_JSON__", json.dumps(SERVICES, ensure_ascii=False)
            ).replace(
                "__TASKS_JSON__", json.dumps(TASKS, ensure_ascii=False)
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(html.encode())

    def log_message(self, format, *args):
        pass

def main():
    parser = argparse.ArgumentParser(description="AGX ROS 1 Dashboard")
    parser.add_argument("--port", type=int, default=8081, help="Port (default: 8081)")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host (default: 0.0.0.0)")
    args = parser.parse_args()

    server = http.server.HTTPServer((args.host, args.port), DashboardHandler)
    print(f"""
    AGX ROS 1 Dashboard started
    ================================
    URL:  http://localhost:{args.port}
    Mode: {MODE.upper()}
    Env:  {ENV_FILE}
    ================================
    Press Ctrl+C to stop
    """)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Info] Dashboard stopped.")
        server.server_close()

if __name__ == "__main__":
    main()
