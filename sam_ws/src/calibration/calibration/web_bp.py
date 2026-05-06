"""Flask blueprint for calibration session control, mounted under /calib.

Mount from mainwindow.py with:
    from calibration.web_bp import calib_bp
    app.register_blueprint(calib_bp, url_prefix='/calib')

The blueprint loads lazily and is safe to import even if the calibration
package has missing optional dependencies (e.g., scipy) — endpoints that
need them will error at call-time with a clear message.
"""
from __future__ import annotations

import threading
from typing import Dict

try:
    from flask import Blueprint, request, jsonify, render_template_string
except ImportError:  # pragma: no cover
    Blueprint = None  # type: ignore

from .session_manager import CalibrationSession, PoseMeta, new_session_id
from . import bridge


if Blueprint is None:
    calib_bp = None  # Flask unavailable
else:
    calib_bp = Blueprint('calib', __name__)

    _sessions: Dict[str, Dict] = {}
    _live: Dict[str, CalibrationSession] = {}
    _lock = threading.Lock()

    def _progress_cb(sid):
        def cb(progress: float, message: str):
            with _lock:
                _sessions[sid]['progress'] = float(progress)
                _sessions[sid]['message']  = message
        return cb

    @calib_bp.route('/ping')
    def ping():
        return jsonify({'ok': True, 'service': 'calibration', 'version': '0.1.0'})

    @calib_bp.route('/sessions')
    def list_sessions():
        with _lock:
            return jsonify(_sessions)

    @calib_bp.route('/session/<sid>/status')
    def session_status(sid: str):
        with _lock:
            return jsonify(_sessions.get(sid, {'status': 'unknown'}))

    # ---------- Session creation ----------
    @calib_bp.route('/stage1/create', methods=['POST'])
    def stage1_create():
        data = request.get_json(silent=True) or request.form
        unit = data['unit']
        board_yaml = data.get('board_yaml', '/root/sam_ws/config/board_a_r2_params.yaml')
        sid = data.get('session_id') or new_session_id(f'stage1_{unit}')
        sess = CalibrationSession('stage1', sid, board_yaml, [unit])
        with _lock:
            _sessions[sid] = {'stage': 'stage1', 'unit': unit, 'status': 'ready',
                              'progress': 0.0, 'message': 'session ready'}
            _live[sid] = sess
        return jsonify({'session_id': sid, 'ok': True})

    @calib_bp.route('/stage2/create', methods=['POST'])
    def stage2_create():
        data = request.get_json(silent=True) or request.form
        units = data.getlist('units') if hasattr(data, 'getlist') else data.get('units')
        if isinstance(units, str):
            units = [u.strip() for u in units.split(',') if u.strip()]
        board_yaml = data.get('board_yaml', '/root/sam_ws/config/board_b_r3_params.yaml')
        sid = data.get('session_id') or new_session_id('stage2')
        sess = CalibrationSession('stage2', sid, board_yaml, list(units))
        with _lock:
            _sessions[sid] = {'stage': 'stage2', 'units': list(units), 'status': 'ready',
                              'progress': 0.0, 'message': 'session ready'}
            _live[sid] = sess
        return jsonify({'session_id': sid, 'ok': True})

    # ---------- Pose capture ----------
    @calib_bp.route('/capture_pose', methods=['POST'])
    def capture_pose():
        data = request.get_json(silent=True) or request.form
        sid = data['session_id']
        with _lock:
            sess = _live.get(sid)
            meta = _sessions.get(sid, {})
        if sess is None:
            return jsonify({'ok': False, 'error': 'session not found'}), 404

        pose_meta = PoseMeta(
            stage=sess.stage,
            step=data.get('step', 'lidar_cam'),
            pose_index=int(data.get('pose_index', 0)),
            tilt_deg=float(data.get('tilt_deg', 0)),
            distance_m=(float(data['distance_m']) if data.get('distance_m') else None),
            note=data.get('note', ''),
        )

        try:
            if sess.stage == 'stage1' and pose_meta.step == 'intrinsic':
                unit = meta['unit']
                path = sess.capture_stage1_intrinsic(unit, pose_meta)
                return jsonify({'ok': True, 'path': path})
            elif sess.stage == 'stage1':
                unit = meta['unit']
                pcl_override = int(data.get('pcl_target_override', 1))
                bridge.set_pcl_target_override({unit: pcl_override})
                paths = sess.capture_stage1_lidar_cam(unit, pose_meta)
                return jsonify({'ok': True, **paths})
            else:  # stage2
                pcl_override = int(data.get('pcl_target_override', 3))
                result = sess.capture_stage2_pose(pose_meta,
                                                  pcl_target_override=pcl_override)
                return jsonify({'ok': True, 'capture': result})
        except Exception as e:
            return jsonify({'ok': False, 'error': str(e)}), 500

    # ---------- BA runners (background) ----------
    @calib_bp.route('/stage1/run_ba', methods=['POST'])
    def stage1_run_ba():
        from .stage1.stage1_runner import run_stage1  # lazy import
        data = request.get_json(silent=True) or request.form
        sid = data['session_id']
        with _lock:
            sess = _live.get(sid)
            meta = _sessions.get(sid)
        if sess is None or meta is None:
            return jsonify({'ok': False, 'error': 'session not found'}), 404

        def target():
            try:
                with _lock:
                    _sessions[sid]['status'] = 'running'
                result = run_stage1(meta['unit'], sid,
                                    board_yaml=sess.board_yaml,
                                    progress_cb=_progress_cb(sid))
                with _lock:
                    _sessions[sid]['status'] = 'done' if result['pass'] else 'failed'
                    _sessions[sid]['result'] = result['result']
                    _sessions[sid]['out_dir'] = result['out_dir']
            except Exception as e:
                with _lock:
                    _sessions[sid]['status'] = 'error'
                    _sessions[sid]['message'] = str(e)

        threading.Thread(target=target, daemon=True).start()
        return jsonify({'ok': True, 'session_id': sid})

    @calib_bp.route('/stage2/run_ba', methods=['POST'])
    def stage2_run_ba():
        from .stage2.stage2_runner import run_stage2  # lazy import
        data = request.get_json(silent=True) or request.form
        sid = data['session_id']
        with _lock:
            sess = _live.get(sid)
            meta = _sessions.get(sid)
        if sess is None or meta is None:
            return jsonify({'ok': False, 'error': 'session not found'}), 404

        def target():
            try:
                with _lock:
                    _sessions[sid]['status'] = 'running'
                result = run_stage2(meta['units'], sid,
                                    board_yaml=sess.board_yaml,
                                    progress_cb=_progress_cb(sid))
                with _lock:
                    _sessions[sid]['status'] = 'done' if result['pass'] else 'failed'
                    _sessions[sid]['result'] = result['result']
                    _sessions[sid]['out_dir'] = result['out_dir']
            except Exception as e:
                with _lock:
                    _sessions[sid]['status'] = 'error'
                    _sessions[sid]['message'] = str(e)

        threading.Thread(target=target, daemon=True).start()
        return jsonify({'ok': True, 'session_id': sid})

    # ---------- Minimal UI ----------
    @calib_bp.route('/ui')
    def ui():
        return render_template_string(_CALIB_UI_HTML)


_CALIB_UI_HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Calibration</title>
<style>
  body { font-family: sans-serif; margin: 12px; }
  h2 { margin-bottom: 4px; }
  .row { display: flex; gap: 12px; margin: 8px 0; }
  .card { border: 1px solid #ccc; padding: 10px; min-width: 280px; }
  button { padding: 6px 12px; margin: 4px; }
  pre { background: #f5f5f5; padding: 8px; max-height: 200px; overflow: auto; }
  input, select { margin: 2px; }
</style></head>
<body>
<h2>Calibration — Stage 1 / Stage 2</h2>
<div class="row">
  <div class="card">
    <h3>Stage 1 (per-unit)</h3>
    Unit: <select id="s1_unit">
      <option>Gantry_Global1</option>
      <option>Gantry_Global2</option>
      <option>Gantry_Global3</option>
      <option>Gantry_Global4</option>
      <option>Robot_Local</option>
    </select><br>
    <button onclick="s1create()">Create session</button>
    <button onclick="s1cap('intrinsic')">Capture Intrinsic Pose</button>
    <button onclick="s1cap('lidar_cam')">Capture LiDAR-Cam Pose</button>
    <button onclick="s1run()">Run BA</button>
    <div id="s1_status"></div>
  </div>
  <div class="card">
    <h3>Stage 2 (inter-system)</h3>
    Units (comma): <input id="s2_units" value="Gantry_Global1,Gantry_Global2,Gantry_Global3,Gantry_Global4" size="60"><br>
    <button onclick="s2create()">Create session</button>
    Tilt: <select id="s2_tilt"><option>0</option><option>15</option><option>30</option></select>
    <button onclick="s2cap()">Capture Pose</button>
    <button onclick="s2run()">Run BA</button>
    <div id="s2_status"></div>
  </div>
</div>
<h3>Sessions</h3>
<pre id="sessions">{}</pre>
<script>
let s1sid=null, s2sid=null, s1idx=0, s2idx=0;
async function s1create(){
  const r = await fetch('/calib/stage1/create', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({unit: s1_unit.value})});
  const j = await r.json(); s1sid = j.session_id; s1idx=0;
  document.getElementById('s1_status').textContent = 'session ' + s1sid;
}
async function s1cap(step){
  if (!s1sid) { alert('Create session first'); return; }
  s1idx += 1;
  const r = await fetch('/calib/capture_pose', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({session_id: s1sid, step, pose_index: s1idx})});
  const j = await r.json();
  document.getElementById('s1_status').textContent = JSON.stringify(j);
}
async function s1run(){
  if (!s1sid) return;
  await fetch('/calib/stage1/run_ba', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({session_id: s1sid})});
  setTimeout(refresh, 500);
}
async function s2create(){
  const units = s2_units.value.split(',').map(s=>s.trim());
  const r = await fetch('/calib/stage2/create', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({units})});
  const j = await r.json(); s2sid = j.session_id; s2idx=0;
  document.getElementById('s2_status').textContent = 'session ' + s2sid;
}
async function s2cap(){
  if (!s2sid) return;
  s2idx += 1;
  const r = await fetch('/calib/capture_pose', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({session_id: s2sid, pose_index: s2idx, tilt_deg: s2_tilt.value})});
  const j = await r.json();
  document.getElementById('s2_status').textContent = JSON.stringify(j);
}
async function s2run(){
  if (!s2sid) return;
  await fetch('/calib/stage2/run_ba', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({session_id: s2sid})});
  setTimeout(refresh, 500);
}
async function refresh(){
  const r = await fetch('/calib/sessions'); const j = await r.json();
  document.getElementById('sessions').textContent = JSON.stringify(j, null, 2);
}
setInterval(refresh, 2000);
</script>
</body></html>
"""
