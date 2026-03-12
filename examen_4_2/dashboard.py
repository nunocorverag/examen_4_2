#!/usr/bin/env python3
"""
PuzzleBot Dashboard — Motor Monitor
────────────────────────────────────
localhost:8080

Charts (2x2):
 Motor 1: Setpoint vs Real  |  Motor 2: Setpoint vs Real
 Ambos motores juntos       |  Error de seguimiento
"""
import rclpy, json, threading, time
from http.server import HTTPServer, BaseHTTPRequestHandler
from rclpy.node import Node
from std_msgs.msg import Float32
from collections import deque

MOTOR_WIN = 500

class DataStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.mt  = deque(maxlen=MOTOR_WIN)
        self.sp1 = deque(maxlen=MOTOR_WIN)
        self.sp2 = deque(maxlen=MOTOR_WIN)
        self.mo1 = deque(maxlen=MOTOR_WIN)
        self.mo2 = deque(maxlen=MOTOR_WIN)
        self._last_sp1 = 0.0
        self._last_sp2 = 0.0
        self._last_mo1 = 0.0
        self._last_mo2 = 0.0
        self.t0 = time.time()

    def push_sp1(self, v):
        with self.lock: self._last_sp1 = v
    def push_sp2(self, v):
        with self.lock: self._last_sp2 = v
    def push_mo1(self, v):
        with self.lock: self._last_mo1 = v
    def push_mo2(self, v):
        with self.lock: self._last_mo2 = v

    def tick(self):
        with self.lock:
            now = round(time.time() - self.t0, 2)
            self.mt.append(now)
            self.sp1.append(self._last_sp1)
            self.sp2.append(self._last_sp2)
            self.mo1.append(self._last_mo1)
            self.mo2.append(self._last_mo2)

    def reset(self):
        with self.lock:
            self.mt.clear(); self.sp1.clear(); self.sp2.clear()
            self.mo1.clear(); self.mo2.clear()
            self.t0 = time.time()

    def snapshot(self):
        with self.lock:
            return {
                'mt':  list(self.mt),
                'sp1': list(self.sp1),
                'sp2': list(self.sp2),
                'mo1': list(self.mo1),
                'mo2': list(self.mo2),
            }

store = DataStore()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        p = self.path.split('?')[0]
        if p in ('/', '/index.html'):
            self._resp(200, 'text/html', HTML.encode())
        elif p == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                while True:
                    s = store.snapshot()
                    self.wfile.write(f'data: {json.dumps(s, default=float)}\n\n'.encode())
                    self.wfile.flush()
                    time.sleep(0.1)
            except: pass
        elif p == '/api/reset':
            store.reset()
            self._resp(200, 'text/plain', b'ok')
        else:
            self._resp(404, 'text/plain', b'not found')

    def _resp(self, code, ct, body):
        self.send_response(code)
        self.send_header('Content-Type', ct)
        self.end_headers()
        self.wfile.write(body)


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard')
        self.declare_parameter('port', 8080)
        port = self.get_parameter('port').value

        self.create_subscription(Float32, '/set_point_1',    lambda m: store.push_sp1(m.data), 10)
        self.create_subscription(Float32, '/set_point_2',    lambda m: store.push_sp2(m.data), 10)
        self.create_subscription(Float32, '/motor_output_1', lambda m: store.push_mo1(m.data), 10)
        self.create_subscription(Float32, '/motor_output_2', lambda m: store.push_mo2(m.data), 10)
        self.create_timer(0.05, store.tick)  # 20 Hz

        self._srv = HTTPServer(('0.0.0.0', port), Handler)
        threading.Thread(target=self._srv.serve_forever, daemon=True).start()
        self.get_logger().info(f'Dashboard → http://localhost:{port}')


HTML = r"""<!DOCTYPE html><html><head><meta charset="UTF-8">
<title>Motor Dashboard</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#07070c;color:#ccc;font-family:'JetBrains Mono','Fira Code',monospace;font-size:11px;height:100vh;display:flex;flex-direction:column;overflow:hidden}
.hdr{padding:6px 14px;display:flex;align-items:center;gap:12px;border-bottom:1px solid #161620;flex-shrink:0}
.hdr h1{font-size:13px;letter-spacing:2px;color:#fff}
.dot{width:7px;height:7px;border-radius:50%;display:inline-block}
.on{background:#22c55e;box-shadow:0 0 6px #22c55e}.off{background:#ef4444}
.stats{display:flex;gap:16px;padding:4px 14px;border-bottom:1px solid #0e0e16;font-size:10px;flex-shrink:0}
.stats label{color:#444;margin-right:3px}.stats span{color:#ddd;font-weight:600}
.mv1{color:#00e5ff!important}.mv2{color:#ff6b35!important}
.ml{margin-left:auto}
.btn{padding:3px 10px;font-size:9px;font-weight:600;background:#0e0e16;color:#ef4444;
 border:1px solid #1a1a28;border-radius:3px;cursor:pointer;font-family:inherit}
.btn:hover{background:#1a0808}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:1px;padding:1px;flex:1;min-height:0}
.cell{background:#0a0a12;position:relative;padding:6px 4px 4px;overflow:hidden;display:flex;flex-direction:column}
.cell canvas{flex:1;min-height:0;width:100%!important}
.ct{font-size:8px;color:#3a3a50;letter-spacing:.5px;text-transform:uppercase;margin-bottom:3px;padding-left:2px;flex-shrink:0}
.cell.m1{border-top:2px solid #00e5ff33}
.cell.m2{border-top:2px solid #ff6b3533}
.cell.both{border-top:2px solid #ffffff0f}
.cell.err{border-top:2px solid #8b5cf633}
</style></head><body>

<div class="hdr">
 <span class="dot off" id="dot"></span>
 <h1>MOTOR DASHBOARD</h1>
 <div class="stats" style="border:none;padding:0;margin:0;gap:14px">
  <div><label>M1 SP</label><span class="mv1" id="v-sp1">—</span></div>
  <div><label>M1 OUT</label><span class="mv1" id="v-mo1">—</span></div>
  <div><label>M2 SP</label><span class="mv2" id="v-sp2">—</span></div>
  <div><label>M2 OUT</label><span class="mv2" id="v-mo2">—</span></div>
  <div><label>ERR1</label><span id="v-e1">—</span></div>
  <div><label>ERR2</label><span id="v-e2">—</span></div>
 </div>
 <button class="btn ml" onclick="fetch('/api/reset')">Reset</button>
</div>

<div class="grid">
 <div class="cell m1">
  <div class="ct">Motor 1 — Setpoint (---) vs Real (rad/s)</div>
  <canvas id="cm1"></canvas>
 </div>
 <div class="cell m2">
  <div class="ct">Motor 2 — Setpoint (---) vs Real (rad/s)</div>
  <canvas id="cm2"></canvas>
 </div>
 <div class="cell both">
  <div class="ct">Velocidad Real — Motor 1 (cyan) + Motor 2 (naranja)</div>
  <canvas id="cboth"></canvas>
 </div>
 <div class="cell err">
  <div class="ct">Error de seguimiento — Setpoint − Real</div>
  <canvas id="cerr"></canvas>
 </div>
</div>

<script>
const tO = () => ({
  responsive: true, maintainAspectRatio: false, animation: {duration:0},
  scales: {
    x: {ticks:{color:'#2a2a40',maxTicksLimit:8,font:{size:8}}, grid:{color:'#0f0f1e'}},
    y: {ticks:{color:'#2a2a40',font:{size:8}}, grid:{color:'#0f0f1e'}}
  },
  plugins: {legend:{display:false}},
  elements: {point:{radius:0}, line:{borderWidth:1.8}}
});

function mk(id, datasets) {
  return new Chart(document.getElementById(id), {
    type:'line', data:{labels:[], datasets}, options:tO()
  });
}

const cm1   = mk('cm1',   [{data:[], borderColor:'#00e5ff'}, {data:[], borderColor:'#ffffff44', borderDash:[5,3], borderWidth:1.2}]);
const cm2   = mk('cm2',   [{data:[], borderColor:'#ff6b35'}, {data:[], borderColor:'#ffffff44', borderDash:[5,3], borderWidth:1.2}]);
const cboth = mk('cboth', [{data:[], borderColor:'#00e5ff', borderWidth:1.5}, {data:[], borderColor:'#ff6b35', borderWidth:1.5}]);
const cerr  = mk('cerr',  [{data:[], borderColor:'#00e5ff88'}, {data:[], borderColor:'#ff6b3588'}]);

const es = new EventSource('/stream');
es.onopen  = () => document.getElementById('dot').className = 'dot on';
es.onerror = () => document.getElementById('dot').className = 'dot off';

es.onmessage = ev => {
  const d = JSON.parse(ev.data);
  const M = d.mt.length;
  if (M < 2) return;

  const lb   = d.mt.map(v => v.toFixed(1));
  const err1 = d.sp1.map((s,i) => s - d.mo1[i]);
  const err2 = d.sp2.map((s,i) => s - d.mo2[i]);

  cm1.data.labels = lb; cm1.data.datasets[0].data = d.mo1; cm1.data.datasets[1].data = d.sp1; cm1.update('none');
  cm2.data.labels = lb; cm2.data.datasets[0].data = d.mo2; cm2.data.datasets[1].data = d.sp2; cm2.update('none');
  cboth.data.labels = lb; cboth.data.datasets[0].data = d.mo1; cboth.data.datasets[1].data = d.mo2; cboth.update('none');
  cerr.data.labels = lb; cerr.data.datasets[0].data = err1; cerr.data.datasets[1].data = err2; cerr.update('none');

  const L = M-1;
  document.getElementById('v-sp1').textContent = d.sp1[L].toFixed(3);
  document.getElementById('v-mo1').textContent = d.mo1[L].toFixed(3);
  document.getElementById('v-sp2').textContent = d.sp2[L].toFixed(3);
  document.getElementById('v-mo2').textContent = d.mo2[L].toFixed(3);
  document.getElementById('v-e1').textContent  = err1[L].toFixed(3);
  document.getElementById('v-e2').textContent  = err2[L].toFixed(3);
};
</script></body></html>"""


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._srv.shutdown()
        try: node.destroy_node()
        except: pass
        try: rclpy.shutdown()
        except: pass

if __name__ == '__main__':
    main()