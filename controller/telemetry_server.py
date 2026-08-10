#!/usr/bin/env python3
"""Live telemetry dashboard for the robot.

Listens on a loopback UDP port for ODOM lines forwarded by xbox_control.py and
serves a self-contained web page that graphs them.

    python3 telemetry_server.py          # http://<pi>:8080

Standard library only -- no pip, no CDN. The Pi may have no internet, and a
dashboard that breaks when the network is down is useless precisely when you
need it. The chart is hand-drawn on a canvas for the same reason.

It reads from xbox_control.py rather than from the board, on purpose: the
firmware sends telemetry to whichever client contacted it last, so a second
UDP client would hijack the driver station's feed.
"""

import http.server
import json
import os
import socket
import threading
import time
from collections import deque

HTTP_PORT = int(os.environ.get("DASHBOARD_HTTP_PORT", "8080"))
UDP_PORT = int(os.environ.get("DASHBOARD_PORT", "9999"))
HISTORY = 600           # samples kept (~60s at 10Hz telemetry)
STALE_AFTER = 2.0       # seconds without data before we call the link down

_lock = threading.Lock()
_samples = deque(maxlen=HISTORY)
_last_rx = 0.0


def udp_listener():
    """Collect ODOM lines forever."""
    global _last_rx
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", UDP_PORT))
    while True:
        data, _ = sock.recvfrom(1500)
        msg = data.decode(errors="replace").strip()
        if not msg.startswith("ODOM,"):
            continue
        parts = msg.split(",")
        if len(parts) != 7:
            continue
        try:
            x, y, theta, vx, vy = (float(v) for v in parts[1:6])
        except ValueError:
            continue
        with _lock:
            _samples.append({
                "t": round(time.time(), 3),
                "x": x, "y": y, "theta": theta,
                "vx": vx, "vy": vy,
                "stationary": parts[6] == "1",
            })
            _last_rx = time.time()


PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>Robot Telemetry</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
  :root { color-scheme: dark; }
  body { margin:0; background:#12141a; color:#e6e8ee;
         font:14px/1.5 ui-sans-serif,system-ui,-apple-system,sans-serif; padding:18px; }
  h1 { font-size:16px; font-weight:600; margin:0 0 14px; letter-spacing:.02em; }
  .row { display:flex; flex-wrap:wrap; gap:10px; margin-bottom:16px; }
  .tile { background:#1b1e26; border:1px solid #2a2f3a; border-radius:8px;
          padding:10px 14px; min-width:104px; }
  .tile .k { font-size:11px; text-transform:uppercase; letter-spacing:.06em; color:#8b93a7; }
  .tile .v { font-size:19px; font-variant-numeric:tabular-nums; margin-top:2px; }
  .status { display:inline-flex; align-items:center; gap:7px; font-weight:600; }
  .dot { width:9px; height:9px; border-radius:50%; }
  .up   { background:#3ddc84; box-shadow:0 0 8px #3ddc8488; }
  .down { background:#ff5c5c; box-shadow:0 0 8px #ff5c5c88; }
  canvas { width:100%; height:220px; background:#1b1e26;
           border:1px solid #2a2f3a; border-radius:8px; display:block; margin-bottom:14px; }
  .cap { font-size:12px; color:#8b93a7; margin:0 0 6px; }
</style></head><body>
<h1>Robot Telemetry <span id="status" class="status"></span></h1>
<div class="row">
  <div class="tile"><div class="k">X</div><div class="v" id="x">--</div></div>
  <div class="tile"><div class="k">Y</div><div class="v" id="y">--</div></div>
  <div class="tile"><div class="k">Heading</div><div class="v" id="th">--</div></div>
  <div class="tile"><div class="k">Speed</div><div class="v" id="sp">--</div></div>
  <div class="tile"><div class="k">State</div><div class="v" id="st">--</div></div>
</div>
<p class="cap">Position (m) &mdash; x blue, y orange</p><canvas id="c1"></canvas>
<p class="cap">Heading (deg)</p><canvas id="c2"></canvas>
<p class="cap">Path (x vs y, metres)</p><canvas id="c3"></canvas>
<script>
function sizeCanvas(c){const r=devicePixelRatio||1,b=c.getBoundingClientRect();
  c.width=b.width*r;c.height=b.height*r;const g=c.getContext('2d');g.setTransform(r,0,0,r,0,0);return g;}

function plot(c, series, colors){
  const g=sizeCanvas(c), W=c.getBoundingClientRect().width, H=c.getBoundingClientRect().height;
  g.clearRect(0,0,W,H);
  let lo=Infinity, hi=-Infinity;
  for(const s of series) for(const v of s){ if(v<lo)lo=v; if(v>hi)hi=v; }
  if(!isFinite(lo)){ lo=-1; hi=1; }
  if(hi-lo < 1e-6){ hi+=0.5; lo-=0.5; }
  const pad=(hi-lo)*0.12; lo-=pad; hi+=pad;
  // gridlines + labels
  g.strokeStyle='#2a2f3a'; g.fillStyle='#8b93a7'; g.font='10px ui-sans-serif'; g.lineWidth=1;
  for(let i=0;i<=4;i++){ const yy=H-(i/4)*H; g.beginPath(); g.moveTo(34,yy); g.lineTo(W,yy); g.stroke();
    g.fillText((lo+(hi-lo)*i/4).toFixed(2), 2, yy-2); }
  series.forEach((s,si)=>{
    if(s.length<2) return;
    g.strokeStyle=colors[si]; g.lineWidth=1.6; g.beginPath();
    s.forEach((v,i)=>{ const xx=34+(i/(s.length-1))*(W-38), yy=H-((v-lo)/(hi-lo))*H;
      i?g.lineTo(xx,yy):g.moveTo(xx,yy); });
    g.stroke();
  });
}

function plotPath(c, xs, ys){
  const g=sizeCanvas(c), W=c.getBoundingClientRect().width, H=c.getBoundingClientRect().height;
  g.clearRect(0,0,W,H);
  if(xs.length<2) return;
  // equal aspect so the path is not visually distorted
  let minx=Math.min(...xs),maxx=Math.max(...xs),miny=Math.min(...ys),maxy=Math.max(...ys);
  const cx=(minx+maxx)/2, cy=(miny+maxy)/2;
  let span=Math.max(maxx-minx, maxy-miny, 0.5)*1.2;
  const sc=Math.min(W,H)/span;
  g.strokeStyle='#2a2f3a'; g.beginPath(); g.moveTo(0,H/2); g.lineTo(W,H/2);
  g.moveTo(W/2,0); g.lineTo(W/2,H); g.stroke();
  g.strokeStyle='#7aa2ff'; g.lineWidth=1.8; g.beginPath();
  xs.forEach((v,i)=>{ const xx=W/2+(v-cx)*sc, yy=H/2-(ys[i]-cy)*sc; i?g.lineTo(xx,yy):g.moveTo(xx,yy); });
  g.stroke();
  const lx=W/2+(xs[xs.length-1]-cx)*sc, ly=H/2-(ys[ys.length-1]-cy)*sc;
  g.fillStyle='#3ddc84'; g.beginPath(); g.arc(lx,ly,4,0,7); g.fill();
}

async function tick(){
  try{
    const r=await fetch('/data'); const d=await r.json();
    const s=document.getElementById('status');
    s.innerHTML = d.connected
      ? '<span class="dot up"></span>connected'
      : '<span class="dot down"></span>no data';
    const n=d.samples.length;
    if(n){
      const last=d.samples[n-1];
      document.getElementById('x').textContent=last.x.toFixed(2)+' m';
      document.getElementById('y').textContent=last.y.toFixed(2)+' m';
      document.getElementById('th').textContent=last.theta.toFixed(1)+'\\u00b0';
      document.getElementById('sp').textContent=Math.hypot(last.vx,last.vy).toFixed(2)+' m/s';
      document.getElementById('st').textContent=last.stationary?'parked':'moving';
      const xs=d.samples.map(p=>p.x), ys=d.samples.map(p=>p.y);
      plot(document.getElementById('c1'),[xs,ys],['#7aa2ff','#ffab5c']);
      plot(document.getElementById('c2'),[d.samples.map(p=>p.theta)],['#3ddc84']);
      plotPath(document.getElementById('c3'),xs,ys);
    }
  }catch(e){ /* server restarting; next tick retries */ }
}
setInterval(tick,250); tick();
</script></body></html>
"""


class Handler(http.server.BaseHTTPRequestHandler):
    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/data"):
            with _lock:
                payload = {
                    "connected": (time.time() - _last_rx) < STALE_AFTER,
                    "samples": list(_samples),
                }
            self._send(json.dumps(payload).encode(), "application/json")
        elif self.path in ("/", "/index.html"):
            self._send(PAGE.encode(), "text/html; charset=utf-8")
        else:
            self.send_error(404)

    def log_message(self, *args):
        pass  # a request line per 250ms poll would bury the journal


def main():
    threading.Thread(target=udp_listener, daemon=True).start()
    # 0.0.0.0 so you can open it from your laptop, not just the Pi.
    srv = http.server.ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"[telemetry] http://0.0.0.0:{HTTP_PORT}  (udp in on 127.0.0.1:{UDP_PORT})")
    srv.serve_forever()


if __name__ == "__main__":
    main()
