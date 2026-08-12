#!/usr/bin/env python3
"""把 /mission/graph_dot 实时渲染为浏览器可查看的 SVG 页面。"""

import argparse
import functools
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import MissionStatus
from robot_interfaces.srv import JumpMissionState, ShiftMissionState
from std_msgs.msg import String
from std_srvs.srv import Trigger


HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <title>Echo Drone 决策图</title>
  <style>
    body { margin: 0; background: #f1f3f5; font-family: sans-serif; }
    header {
      position: sticky; top: 0; z-index: 1; padding: 10px 16px;
      background: #212529; color: white;
    }
    .legend { margin-left: 18px; color: #dee2e6; font-size: 13px; }
    #controls { display: inline-flex; flex-wrap: wrap; gap: 6px; margin-left: 16px; }
    button, input { font: inherit; }
    button { padding: 4px 8px; cursor: pointer; }
    input { width: 54px; }
    #result { margin-left: 12px; color: #ffd43b; font-size: 13px; }
    #graph-shell { height: calc(100vh - 58px); overflow: auto; }
    #graph { display: block; min-width: 1100px; }
    #log-panel { margin: 0; padding: 6px 16px; background: #fff; border-bottom: 1px solid #ced4da; }
    #log { max-height: 160px; overflow: auto; margin: 6px 0 0; white-space: pre-wrap; font: 12px monospace; }
  </style>
</head>
<body>
  <header>
    Echo Drone 决策拓扑（每秒自动刷新）
    <span class="legend">绿=运行　黄=暂停　灰=空闲　红=中止　蓝=候选后继</span>
    <span id="controls">
      <button onclick="control('start')">Start</button>
      <button onclick="control('pause')">暂停</button>
      <button onclick="control('resume')">继续</button>
      <button onclick="control('step_once')">单步</button>
      <button onclick="control('dump_context')">打印上下文</button>
      <button onclick="shift(-1)">ID -1</button>
      <button onclick="shift(1)">ID +1</button>
      <input id="state-id" type="number" min="0" max="41" placeholder="ID">
      <button onclick="jumpFromInput()">跳转</button>
      <button onclick="control('abort')">中止</button>
      <button onclick="zoomGraph(-0.2)">缩小</button>
      <button onclick="zoomGraph(0.2)">放大</button>
      <button onclick="fitGraph()">适配</button>
    </span>
    <span id="result">点击节点可跳转；所有跳转仍受 dry-run 安全门控。</span>
  </header>
  <details id="log-panel">
    <summary>调试日志（最近命令和上下文）</summary>
    <pre id="log"></pre>
  </details>
  <div id="graph-shell">
    <object id="graph" type="image/svg+xml" aria-label="等待 /mission/graph_dot 数据"></object>
  </div>
  <script>
    const image = document.getElementById('graph');
    const graphShell = document.getElementById('graph-shell');
    const result = document.getElementById('result');
    const log = document.getElementById('log');
    let graphZoom = 1.0;
    function resizeGraph() {
      const svg = image.contentDocument && image.contentDocument.documentElement;
      if (!svg) return;
      const viewBox = (svg.getAttribute('viewBox') || '').trim().split(/\\s+/).map(Number);
      if (viewBox.length !== 4 || !viewBox[2] || !viewBox[3]) return;
      const baseWidth = Math.max(1100, graphShell.clientWidth - 8);
      const width = Math.round(baseWidth * graphZoom);
      image.style.width = width + 'px';
      image.style.height = Math.round(width * viewBox[3] / viewBox[2]) + 'px';
    }
    function zoomGraph(delta) {
      graphZoom = Math.max(0.4, Math.min(3.0, graphZoom + delta));
      resizeGraph();
      result.textContent = '图缩放 ' + Math.round(graphZoom * 100) + '%；可在画布内滚动查看。';
    }
    function fitGraph() {
      graphZoom = 1.0;
      resizeGraph();
      result.textContent = '图已适配画布宽度。';
    }
    function appendLog(message) {
      const time = new Date().toLocaleTimeString();
      log.textContent = '[' + time + '] ' + message + '\\n' + log.textContent;
    }
    async function control(action, extra = {}) {
      result.textContent = '正在请求 ' + action + '...';
      try {
        const response = await fetch('/api/control', {
          method: 'POST', headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(Object.assign({action: action}, extra))
        });
        const body = await response.json();
        result.textContent = body.message;
        appendLog(action + ': ' + body.message);
      } catch (error) {
        result.textContent = '控制请求失败：' + error;
        appendLog(action + ': ' + result.textContent);
      }
    }
    function shift(delta) { control('shift', {delta: delta, reset_context: true}); }
    function jump(id) {
      if (Number.isInteger(id) && id >= 0 && id <= 41 &&
          window.confirm('确认跳转到状态 ID ' + id + '？')) {
        control('jump', {state_id: id, reset_context: true});
      }
    }
    function jumpFromInput() { jump(Number(document.getElementById('state-id').value)); }
    function bindNodeClicks() {
      const svg = image.contentDocument;
      if (!svg) return;
      svg.querySelectorAll('g.node').forEach((node) => {
        const title = node.querySelector('title');
        const match = title && title.textContent.match(/^s(\\d+)$/);
        if (!match) return;
        node.style.cursor = 'pointer';
        node.addEventListener('click', () => jump(Number(match[1])));
      });
    }
    let loadedGraphVersion = -1;
    async function checkGraphVersion() {
      try {
        const response = await fetch('/api/graph-version', {cache: 'no-store'});
        const body = await response.json();
        if (body.version > 0 && body.version !== loadedGraphVersion) {
          loadedGraphVersion = body.version;
          image.data = 'graph.svg?v=' + body.version;
        }
      } catch (error) {
        result.textContent = '图版本检查失败：' + error;
      }
    }
    image.addEventListener('load', () => { bindNodeClicks(); resizeGraph(); });
    window.addEventListener('resize', resizeGraph);
    checkGraphVersion();
    setInterval(checkGraphVersion, 1000);
  </script>
</body>
</html>
"""


class ViewerHandler(SimpleHTTPRequestHandler):
    """静态托管页面，并把本机浏览器控制请求转交给 ROS 服务。"""

    def __init__(self, *args, viewer=None, **kwargs):
        self.viewer = viewer
        super().__init__(*args, **kwargs)

    def log_message(self, format_, *args):
        del format_, args

    def do_POST(self):
        if self.path != '/api/control':
            self.send_error(404)
            return
        try:
            content_length = int(self.headers.get('Content-Length', '0'))
            payload = json.loads(self.rfile.read(content_length).decode('utf-8'))
            response, status = self.viewer.request_control(payload)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
            response, status = {'message': '无效控制请求: {}'.format(error)}, 400
        data = json.dumps(response, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == '/api/graph-version':
            data = json.dumps(
                {'version': self.viewer.get_graph_version()}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        super().do_GET()


class MissionGraphViewer(Node):
    """订阅动态图 DOT，并通过 Graphviz 原子更新 SVG 文件。"""

    def __init__(self, output_directory):
        super().__init__('mission_graph_viewer')
        self.output_directory = output_directory
        self.dot_path = output_directory / 'graph.dot'
        self.temp_dot_path = output_directory / 'graph.dot.tmp'
        self.svg_path = output_directory / 'graph.svg'
        self.temp_svg_path = output_directory / 'graph.svg.tmp'
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscription = self.create_subscription(
            String, '/mission/graph_dot', self.render, qos)
        self.status_subscription = self.create_subscription(
            MissionStatus, '/mission/status', self.receive_status, 10)
        self.trigger_clients = {
            'start': self.create_client(Trigger, '/mission/start'),
            'pause': self.create_client(Trigger, '/mission/pause'),
            'resume': self.create_client(Trigger, '/mission/resume'),
            'step_once': self.create_client(Trigger, '/mission/step_once'),
            # 只读输出当前状态、姿态和任务标志，供浏览器日志面板显示。
            'dump_context': self.create_client(Trigger, '/mission/dump_context'),
            'abort': self.create_client(Trigger, '/mission/abort'),
        }
        self.jump_client = self.create_client(
            JumpMissionState, '/mission/jump_to_state')
        self.shift_client = self.create_client(
            ShiftMissionState, '/mission/shift_state')
        self.dry_run = None
        self.last_dot = None
        self.graph_version = 0
        self.graph_lock = threading.Lock()
        self.render_count = 0

    def receive_status(self, message):
        """只在 dry-run 决策节点上开放浏览器控制，避免误连实机节点。"""
        self.dry_run = message.dry_run

    def call_service(self, client, request):
        """在 HTTP 工作线程等待 ROS executor 回调，最多等待两秒。"""
        if not client.service_is_ready():
            return {'ok': False, 'message': 'ROS 服务尚未就绪'}, 503
        completed = threading.Event()
        result = {}

        def finished(future):
            try:
                response = future.result()
                result['ok'] = bool(response.success)
                result['message'] = response.message
            except Exception as error:  # rclpy 会把通信异常放进 future。
                result['ok'] = False
                result['message'] = 'ROS 服务调用失败: {}'.format(error)
            completed.set()

        client.call_async(request).add_done_callback(finished)
        if not completed.wait(timeout=2.0):
            return {'ok': False, 'message': 'ROS 服务调用超时'}, 504
        return result, 200 if result['ok'] else 409

    def request_control(self, payload):
        """校验浏览器参数后调用现有调试服务，安全规则仍由决策节点最终裁决。"""
        if self.dry_run is not True:
            return {
                'ok': False,
                'message': '控制面板仅在确认 /mission/status 的 dry_run=true 后启用',
            }, 403
        action = payload.get('action')
        if action in self.trigger_clients:
            return self.call_service(self.trigger_clients[action], Trigger.Request())
        if action == 'jump':
            state_id = payload.get('state_id')
            if not isinstance(state_id, int) or isinstance(state_id, bool):
                return {'ok': False, 'message': 'state_id 必须是整数'}, 400
            request = JumpMissionState.Request()
            request.state_id = state_id
            request.reset_context = bool(payload.get('reset_context', True))
            return self.call_service(self.jump_client, request)
        if action == 'shift':
            delta = payload.get('delta')
            if not isinstance(delta, int) or isinstance(delta, bool) or delta == 0:
                return {'ok': False, 'message': 'delta 必须是非零整数'}, 400
            request = ShiftMissionState.Request()
            request.delta = delta
            request.reset_context = bool(payload.get('reset_context', True))
            return self.call_service(self.shift_client, request)
        return {'ok': False, 'message': '未知控制动作'}, 400

    def render(self, message):
        """把最新 DOT 文本渲染成 SVG，成功后再替换浏览器读取的文件。"""
        # 决策节点为新加入的订阅者会以 1 Hz 重发同一份图。DOT 未变就不执行
        # Graphviz，也不递增网页版本号，避免页面看起来反复整体闪烁。
        with self.graph_lock:
            if message.data == self.last_dot:
                return
        self.temp_dot_path.write_text(message.data, encoding='utf-8')
        result = subprocess.run(
            ['dot', '-Tsvg', '-o', str(self.temp_svg_path)],
            input=message.data,
            text=True,
            capture_output=True,
            check=False)
        if result.returncode != 0:
            self.get_logger().error(
                'Graphviz 渲染失败: {}'.format(result.stderr.strip()))
            return
        self.temp_dot_path.replace(self.dot_path)
        self.temp_svg_path.replace(self.svg_path)
        with self.graph_lock:
            self.last_dot = message.data
            self.graph_version += 1
        self.render_count += 1
        if self.render_count == 1:
            self.get_logger().info('已收到决策图并生成 {}'.format(self.svg_path))

    def get_graph_version(self):
        """供网页轻量轮询；只有 SVG 实际更新时才变化。"""
        with self.graph_lock:
            return self.graph_version


def parse_args():
    parser = argparse.ArgumentParser(description='实时显示 Echo Drone 决策有向图')
    parser.add_argument(
        '--output-dir', default='/tmp/echo_drone_mission_graph',
        help='SVG/HTML 输出目录')
    parser.add_argument('--port', type=int, default=8765, help='本地 HTTP 端口')
    return parser.parse_known_args()


def main():
    args, ros_args = parse_args()
    output_directory = Path(args.output_dir).resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    (output_directory / 'index.html').write_text(HTML, encoding='utf-8')

    rclpy.init(args=ros_args)
    node = MissionGraphViewer(output_directory)
    handler = functools.partial(
        ViewerHandler, directory=str(output_directory), viewer=node)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(
        '浏览器打开 http://127.0.0.1:{} 查看实时拓扑'.format(args.port))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
