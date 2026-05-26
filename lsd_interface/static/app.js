/**
 * JRT U81 LDS 웹 제어 — WebSocket 클라이언트
 * ─────────────────────────────────────────────────────────────────────
 * WebSocket 메시지 타입:
 *   수신: 'status' | 'distance' | 'ack' | 'error'
 *   송신: { cmd: 'power_on' | 'power_off' | 'measure_once' |
 *                'continuous_on' | 'continuous_off' }
 */

'use strict';

// ── 상태 ─────────────────────────────────────────────────────────────
let ws            = null;
let isPowered     = false;
let isContinuous  = false;
let measureMode   = 'slow';
let logCount      = 0;
const MAX_LOG     = 50;
let toastTimer    = null;
let reconnTimer   = null;

const MODE_DESC = {
  slow: '원거리·저반사 환경 최적화 — 타임아웃 8초/회',
  auto: '일반 환경 균형 모드 — 타임아웃 5초/회',
  fast: '근거리·고반사 환경 최적화 — 타임아웃 3초/회',
};

// ── WebSocket 초기화 ──────────────────────────────────────────────────

function initWS() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const url   = `${proto}://${location.host}/ws`;

  ws = new WebSocket(url);

  ws.onopen = () => {
    clearTimeout(reconnTimer);
    setConnected(true);
    showToast('서버에 연결됐습니다.', 'success');
  };

  ws.onclose = () => {
    setConnected(false);
    ws = null;
    showToast('연결이 끊겼습니다. 재연결 중...', 'error');
    reconnTimer = setTimeout(initWS, 3000);
  };

  ws.onerror = () => {
    // onclose 가 이어서 호출되므로 별도 처리 불필요
  };

  ws.onmessage = (event) => {
    let msg;
    try { msg = JSON.parse(event.data); }
    catch { console.warn('JSON 파싱 실패:', event.data); return; }
    handleMessage(msg);
  };
}

// ── 메시지 라우팅 ─────────────────────────────────────────────────────

function handleMessage(msg) {
  switch (msg.type) {

    case 'status':
      applyStatus(msg);
      break;

    case 'distance':
      if (msg.success) {
        renderDistance(msg);
        appendLog(msg);
      } else {
        showDistanceError(msg.error || '측정 오류');
      }
      break;

    case 'ack':
      if (msg.message) {
        showToast(msg.message, msg.success ? 'success' : 'error');
      }
      break;

    case 'voltage':
      if (msg.success && msg.voltage_mv != null) {
        updateVoltage(msg.voltage_mv, msg.battery_pct);
        showToast(`공급 전압: ${msg.message}`, msg.voltage_mv >= 2800 ? 'success' : 'error');
      } else {
        showToast(msg.message || '전압 읽기 실패', 'error');
      }
      break;

    case 'error':
      showToast(msg.message || '오류 발생', 'error');
      break;

    default:
      console.warn('알 수 없는 메시지 타입:', msg.type);
  }
}

// ── 상태 동기화 ───────────────────────────────────────────────────────

function applyStatus(status) {
  isPowered    = !!status.powered;
  isContinuous = !!status.continuous;

  // 포트 뱃지
  if (status.port) {
    el('portBadge').textContent = status.port;
  }

  // 전압 뱃지 (battery_pct 있으면 % 동반 표시 — 무선 브리지)
  if (status.voltage_mv != null) {
    updateVoltage(status.voltage_mv, status.battery_pct);
  }

  // 센서 상태 필
  setPill('statePower', isPowered,    '레이저 ON',  '레이저 OFF');
  setPill('stateCont',  isContinuous, '연속 ON',    '연속 OFF', 'cont');

  // 버튼 활성/비활성
  el('btnOn').disabled   = isPowered;
  el('btnOff').disabled  = !isPowered;
  el('btnOnce').disabled = !isPowered || isContinuous;
  el('btnCont').disabled = !isPowered;

  // 연속 버튼 상태
  const btnCont = el('btnCont');
  el('contIcon').textContent  = isContinuous ? '■' : '▶';
  el('contLabel').textContent = isContinuous ? '연속 중지' : '연속 시작';
  btnCont.classList.toggle('active', isContinuous);

  // 측정 모드 버튼 상태
  if (status.measure_mode) {
    applyMode(status.measure_mode);
  }
}

function setPill(id, isOn, onLabel, offLabel, extraClass = '') {
  const pill = el(id);
  pill.textContent = isOn ? onLabel : offLabel;
  pill.className   = 'state-pill' + (isOn ? ' on' : '') + (isOn && extraClass ? ` ${extraClass}` : '');
}

// ── 거리 표시 ─────────────────────────────────────────────────────────

function renderDistance(data) {
  const mmEl  = el('distanceMain');
  const mEl   = el('distanceSub');
  const badge = el('lowConfBadge');

  mmEl.textContent = `${data.mm.toLocaleString('ko-KR')} mm`;
  mmEl.classList.remove('error');

  if (data.low_confidence) {
    mmEl.classList.add('low-conf');
    badge.style.display = '';
  } else {
    mmEl.classList.remove('low-conf');
    badge.style.display = 'none';
  }

  mEl.textContent = `${data.m.toFixed(3)} m`;
  renderSQ(data.signal_quality);
}

function showDistanceError(msg) {
  const mmEl = el('distanceMain');
  mmEl.textContent = 'ERR';
  mmEl.classList.add('error');
  mmEl.classList.remove('low-conf');
  el('lowConfBadge').style.display = 'none';
  el('distanceSub').textContent  = msg;
  renderSQ(-1);
}

function renderSQ(sq) {
  const barEl   = el('sqBar');
  const valEl   = el('sqValue');
  const gradeEl = el('sqGrade');

  if (sq < 0) {
    barEl.style.width = '0%';
    valEl.textContent = '--';
    gradeEl.textContent = '';
    gradeEl.className   = 'sq-grade';
    return;
  }

  valEl.textContent = sq;

  // 신호품질 숫자가 낮을수록 좋음 (0~2000 기준으로 역산)
  const MAX_SQ = 2000;
  const pct    = Math.max(0, Math.min(100, (1 - sq / MAX_SQ) * 100));
  barEl.style.width = `${pct.toFixed(1)}%`;

  let grade, color;
  if      (sq <  100) { grade = '우수'; color = 'good'; }
  else if (sq <  500) { grade = '양호'; color = 'normal'; }
  else if (sq < 1000) { grade = '보통'; color = 'warn'; }
  else                { grade = '불량'; color = 'bad'; }

  barEl.style.background = getComputedStyle(document.documentElement)
                            .getPropertyValue(`--${color === 'good' || color === 'normal' ? 'green' : color === 'warn' ? 'yellow' : 'red'}`).trim();

  gradeEl.textContent = grade;
  gradeEl.className   = `sq-grade ${color}`;
}

// ── 측정 이력 ─────────────────────────────────────────────────────────

function appendLog(data) {
  const list = el('logList');

  // '기록 없음' 문구 제거
  const empty = list.querySelector('.log-empty');
  if (empty) empty.remove();

  const now  = new Date(data.timestamp * 1000);
  const time = now.toLocaleTimeString('ko-KR', { hour12: false, hour:'2-digit', minute:'2-digit', second:'2-digit' });

  const entry = document.createElement('div');
  entry.className = 'log-entry' + (data.low_confidence ? ' low-conf' : '');
  const lcMark = data.low_confidence ? '<span class="log-lowconf-mark">~</span>' : '';
  entry.innerHTML =
    `<span class="log-time">${time}</span>` +
    lcMark +
    `<span class="log-mm">${data.mm.toLocaleString('ko-KR')} mm</span>` +
    `<span class="log-m">${data.m.toFixed(3)} m</span>` +
    `<span class="log-sq">SQ:${data.signal_quality}</span>`;

  list.prepend(entry);

  // 최대 건수 유지
  while (list.children.length > MAX_LOG) {
    list.removeChild(list.lastChild);
  }

  logCount++;
  el('logCount').textContent = `${logCount} 건`;
}

function clearLog() {
  el('logList').innerHTML = '<div class="log-empty">아직 측정 기록이 없습니다.</div>';
  logCount = 0;
  el('logCount').textContent = '0 건';
}

// ── 전압 표시 ─────────────────────────────────────────────────────────

function updateVoltage(mv, pct) {
  const badge = el('voltBadge');
  const v = (mv / 1000).toFixed(3);
  // battery_pct(무선 브리지) 있으면 배터리 아이콘 + % 동반 표시
  const hasPct = (pct !== undefined && pct !== null);
  const battIcon = hasPct ? (pct >= 60 ? '🔋' : pct >= 20 ? '🔋' : '🪫') : '⚡';
  badge.textContent = hasPct ? `${battIcon} ${v} V · ${pct}%` : `⚡ ${v} V`;
  badge.style.display = '';
  if (mv < 2800) {
    badge.classList.add('low');
    badge.title = `전압 낮음: ${v} V < 2.8V${hasPct ? ` (${pct}%)` : ''} — 교체/확인 필요 (클릭하여 갱신)`;
  } else {
    badge.classList.remove('low');
    badge.title = `전압: ${v} V${hasPct ? ` · 잔량 ${pct}%` : ''} (클릭하여 갱신)`;
  }
}

// ── 연결 상태 표시 ────────────────────────────────────────────────────

function setConnected(connected) {
  const dot  = el('connDot');
  const text = el('connText');
  dot.className   = 'dot ' + (connected ? 'connected' : 'error');
  text.textContent = connected ? '연결됨' : '연결 끊김';

  if (!connected) {
    // 연결 해제 시 버튼 전부 비활성
    ['btnOn','btnOff','btnOnce','btnCont','btnModeSlow','btnModeAuto','btnModeFast'].forEach(id => {
      el(id).disabled = true;
    });
  }
}

// ── 토스트 알림 ───────────────────────────────────────────────────────

function showToast(message, type = 'info') {
  const toast = el('toast');
  toast.textContent = message;
  toast.className   = `toast show ${type}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { toast.classList.remove('show'); }, 3200);
}

// ── 명령 전송 헬퍼 ────────────────────────────────────────────────────

function sendCmd(cmd, extra = {}) {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    showToast('서버에 연결되지 않았습니다.', 'error');
    return;
  }
  ws.send(JSON.stringify({ cmd, ...extra }));
}

function toggleContinuous() {
  sendCmd(isContinuous ? 'continuous_off' : 'continuous_on');
}

function setMode(mode) {
  sendCmd('set_mode', { mode });
}

function applyMode(mode) {
  measureMode = mode || 'slow';
  ['slow', 'auto', 'fast'].forEach(m => {
    const btn = el('btnMode' + m.charAt(0).toUpperCase() + m.slice(1));
    if (btn) btn.classList.toggle('active', m === measureMode);
  });
  const descEl = el('modeDesc');
  if (descEl) descEl.textContent = MODE_DESC[measureMode] || '';
}

// ── DOM 유틸 ─────────────────────────────────────────────────────────

function el(id) { return document.getElementById(id); }

// ── 시작 ─────────────────────────────────────────────────────────────

initWS();
