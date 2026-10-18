#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LTS Respooler 局域网固件分发页（给板子 OTA 用）。

用法:
    python ota_server.py [目录] [端口]
    # 默认目录 = 本脚本所在目录，默认端口 8000
    # 例: python ota_server.py "D:/WorkPRJ/VscodePRJ/stepmotor - Copy/.pio/build/esp32c3" 8000

页面功能:
    - 显示当前 firmware.bin 的大小 / 时间 / SHA256
    - 拖拽或点选上传自己的 .bin（自动校验 ESP32 镜像头 0xE9，旧文件存 .bak）
    - 显示板子要填的 OTA 地址（http://<本机IP>:端口/firmware.bin），一键复制
    - 板子直接 GET /firmware.bin 即可下载（这就是固件里 OTAURL 要填的地址）
"""
import hashlib
import json
import os
import shutil
import sys
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

ROOT = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8000
FW = os.path.join(ROOT, "firmware.bin")
MAX_UPLOAD = 4 * 1024 * 1024      # 4 MB: OTA app 分区 1.25 MB, 留足余量

PAGE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LTS Respooler · 固件分发</title>
<style>
  :root{--bg:#0f1420;--card:#18202f;--line:#26324a;--txt:#e8eefb;--dim:#8b9bb8;
        --accent:#4da3ff;--ok:#37d67a;--warn:#ffb020;--err:#ff5c5c}
  *{box-sizing:border-box}
  body{margin:0;min-height:100vh;background:radial-gradient(1200px 600px at 20% -10%,#1d2b45 0%,var(--bg) 55%);
       color:var(--txt);font:15px/1.6 -apple-system,"Segoe UI","Microsoft YaHei",sans-serif;
       display:flex;justify-content:center;padding:40px 18px}
  .wrap{width:100%;max-width:720px}
  h1{font-size:20px;margin:0 0 4px;letter-spacing:.3px}
  .sub{color:var(--dim);font-size:13px;margin-bottom:26px}
  .card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:20px;margin-bottom:16px}
  .card h2{font-size:13px;text-transform:uppercase;letter-spacing:.12em;color:var(--dim);margin:0 0 14px;font-weight:600}
  .row{display:flex;justify-content:space-between;gap:12px;padding:6px 0;font-size:14px}
  .row .k{color:var(--dim)}
  .row .v{font-family:ui-monospace,Consolas,monospace;text-align:right;word-break:break-all}
  .ok{color:var(--ok)} .warn{color:var(--warn)} .err{color:var(--err)}
  .urlbox{display:flex;gap:8px;align-items:center;background:#101725;border:1px solid var(--line);
          border-radius:10px;padding:10px 12px;margin-top:6px}
  .urlbox code{flex:1;font-family:ui-monospace,Consolas,monospace;font-size:13px;color:var(--accent);word-break:break-all}
  button{background:var(--accent);color:#06101f;border:0;border-radius:9px;padding:9px 14px;
         font-size:13px;font-weight:700;cursor:pointer;transition:.15s}
  button:hover{filter:brightness(1.1)} button:active{transform:translateY(1px)}
  button.ghost{background:transparent;color:var(--txt);border:1px solid var(--line);font-weight:600}
  .actions{display:flex;gap:10px;margin-top:16px;flex-wrap:wrap}
  a.btn{display:inline-block;background:var(--accent);color:#06101f;text-decoration:none;
        border-radius:9px;padding:10px 16px;font-size:13px;font-weight:700}
  a.btn:hover{filter:brightness(1.1)}
  #drop{border:2px dashed var(--line);border-radius:12px;padding:34px 20px;text-align:center;
        color:var(--dim);cursor:pointer;transition:.15s;margin-top:4px}
  #drop.hot{border-color:var(--accent);background:#14203380;color:var(--txt)}
  #drop b{color:var(--txt)}
  .hint{font-size:12px;color:var(--dim);margin-top:12px}
  #msg{margin-top:14px;font-size:13px;min-height:20px}
  .bar{height:4px;background:#0d1522;border-radius:4px;overflow:hidden;margin-top:12px;display:none}
  .bar i{display:block;height:100%;width:0;background:var(--accent);transition:width .2s}
</style>
</head>
<body>
<div class="wrap">
  <h1>LTS Respooler · 固件分发</h1>
  <div class="sub">给板子 OTA 用的本地固件源 · 同一局域网可访问</div>

  <div class="card">
    <h2>当前固件</h2>
    <div class="row"><span class="k">文件</span><span class="v">firmware.bin</span></div>
    <div class="row"><span class="k">大小</span><span class="v" id="size">—</span></div>
    <div class="row"><span class="k">构建时间</span><span class="v" id="mtime">—</span></div>
    <div class="row"><span class="k">SHA256</span><span class="v" id="sha">—</span></div>
    <div class="row"><span class="k">状态</span><span class="v" id="state">—</span></div>
    <div class="actions">
      <a class="btn" href="/firmware.bin" download>↓ 下载 firmware.bin</a>
      <button class="ghost" onclick="loadInfo()">刷新信息</button>
    </div>
  </div>

  <div class="card">
    <h2>板子 OTA 地址</h2>
    <div class="urlbox">
      <code id="url">—</code>
      <button class="ghost" onclick="copyUrl()">复制</button>
    </div>
    <div class="hint">串口发：<code>{"set":{"OTAURL":"<span id="url2">—</span>"}}</code> 然后 <code>{"CMD":"OTA"}</code></div>
  </div>

  <div class="card">
    <h2>上传自己的 bin</h2>
    <div id="drop">
      拖入 <b>.bin</b> 文件，或点击选择<br>
      <span style="font-size:12px">取 <b>.pio/build/esp32c3/firmware.bin</b>，不是合并后的 merged.bin</span>
    </div>
    <div class="bar" id="bar"><i></i></div>
    <div id="msg"></div>
    <input type="file" id="file" accept=".bin" hidden>
  </div>
</div>

<script>
const $ = s => document.querySelector(s);
const fmt = n => n > 1048576 ? (n/1048576).toFixed(2)+' MB' : (n/1024).toFixed(0)+' KB';

function setState(t, cls){ const e=$('#state'); e.textContent=t; e.className='v '+(cls||''); }

async function loadInfo(){
  try{
    const r = await fetch('/api/info', {cache:'no-store'});
    const d = await r.json();
    if(!d.exists){ setState('尚未上传固件', 'err'); return; }
    $('#size').textContent = fmt(d.size) + '  (' + d.size.toLocaleString() + ' B)';
    $('#mtime').textContent = d.mtime;
    $('#sha').textContent = d.sha256.slice(0,16) + '…';
    const over = d.size > 1310720;
    setState(over ? '超过 app 分区 1.25 MB，OTA 会失败' : '可 OTA', over ? 'err' : 'ok');
  }catch(e){ setState('无法读取状态', 'err'); }
}

function copyUrl(){
  navigator.clipboard.writeText($('#url').textContent).then(()=>{
    $('#msg').innerHTML = '<span class="ok">已复制</span>';
  });
}

async function upload(file){
  if(!file) return;
  if(!/\.bin$/i.test(file.name)){ $('#msg').innerHTML='<span class="err">只接受 .bin 文件</span>'; return; }
  const bar = $('#bar'); bar.style.display='block'; bar.firstElementChild.style.width='0%';
  $('#msg').textContent = '上传中 ' + file.name + ' …';
  try{
    const xhr = new XMLHttpRequest();
    xhr.open('POST','/upload');
    xhr.upload.onprogress = e => { if(e.lengthComputable) bar.firstElementChild.style.width = (e.loaded/e.total*100)+'%'; };
    xhr.onload = () => {
      let d = {}; try{ d = JSON.parse(xhr.responseText); }catch(_){}
      if(xhr.status===200 && d.ok){
        $('#msg').innerHTML = '<span class="ok">✓ 已替换，板子下次 OTA 就会下载这个</span>';
        loadInfo();
      }else{
        $('#msg').innerHTML = '<span class="err">✗ ' + (d.error || ('HTTP '+xhr.status)) + '</span>';
      }
      setTimeout(()=>{ bar.style.display='none'; }, 800);
    };
    xhr.onerror = () => { $('#msg').innerHTML='<span class="err">✗ 网络错误</span>'; bar.style.display='none'; };
    xhr.send(file);
  }catch(e){ $('#msg').innerHTML='<span class="err">✗ '+e+'</span>'; }
}

const drop = $('#drop'), input = $('#file');
drop.onclick = ()=> input.click();
input.onchange = ()=> upload(input.files[0]);
['dragenter','dragover'].forEach(ev=>drop.addEventListener(ev,e=>{e.preventDefault();drop.classList.add('hot');}));
['dragleave','drop'].forEach(ev=>drop.addEventListener(ev,e=>{e.preventDefault();drop.classList.remove('hot');}));
drop.addEventListener('drop', e=> upload(e.dataTransfer.files[0]));

const u = location.origin + '/firmware.bin';
$('#url').textContent = u; $('#url2').textContent = u;
loadInfo(); setInterval(loadInfo, 5000);
</script>
</body></html>
"""


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=ROOT, **kwargs)

    # --- helpers -------------------------------------------------
    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _fw_info(self):
        if not os.path.isfile(FW):
            return {"exists": False}
        st = os.stat(FW)
        h = hashlib.sha256()
        with open(FW, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return {
            "exists": True,
            "name": "firmware.bin",
            "size": st.st_size,
            "mtime": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(st.st_mtime)),
            "sha256": h.hexdigest(),
        }

    # --- routes --------------------------------------------------
    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/info":
            return self._json(self._fw_info())
        if path == "/firmware.bin":
            if not os.path.isfile(FW):
                return self._json({"error": "firmware.bin not uploaded yet"}, 404)
            data = open(FW, "rb").read()
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
            return
        return super().do_GET()

    def do_POST(self):
        if urlparse(self.path).path != "/upload":
            return self._json({"ok": False, "error": "not found"}, 404)
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = 0
        if n <= 0:
            return self._json({"ok": False, "error": "空文件"}, 400)
        if n > MAX_UPLOAD:
            return self._json({"ok": False, "error": "文件超过 4 MB"}, 413)
        data = self.rfile.read(n)
        # ESP32 应用镜像首字节必须是 0xE9（合并 bin / 压缩包 都不是）
        if not data or data[0] != 0xE9:
            return self._json(
                {"ok": False, "error": "不是 ESP32 应用镜像（首字节应为 0xE9）；"
                                       "请用 .pio/build/esp32c3/firmware.bin"}, 400)
        if os.path.isfile(FW):
            shutil.copy2(FW, FW + ".bak")
        with open(FW, "wb") as f:
            f.write(data)
        info = self._fw_info()
        print("[upload] {} bytes, sha256={}".format(n, info.get("sha256", "")[:16]), flush=True)
        return self._json({"ok": True, **info})

    def log_message(self, fmt, *args):
        if "firmware.bin" in (fmt % args) or "upload" in (fmt % args):
            print("[http] " + (fmt % args), flush=True)


if __name__ == "__main__":
    if not os.path.isdir(ROOT):
        sys.exit("目录不存在: " + ROOT)
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print("固件目录: " + ROOT)
    print("页面:     http://localhost:{}/      （手机/板子用 http://<本机IP>:{}/）".format(PORT, PORT))
    print("OTA URL:  http://<本机IP>:{}/firmware.bin".format(PORT))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")
