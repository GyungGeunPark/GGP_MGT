"""
JRT U81 레이저 거리 센서 컨트롤러 — 바이너리 프로토콜
────────────────────────────────────────────────────────────────────────
하드웨어 연결 (USB Interface Board / CH340):
  Module PWREN → USB adapter RTS  (HIGH = 모듈 ON)
  Module nRST  → USB adapter DTR  (HIGH = 리셋 해제)
  Module TXD   → USB RXD  (교차)
  Module RXD   → USB TXD  (교차)

시작 시퀀스 (PDF 6.2절):
  1) RTS=True, DTR=True   → PWREN LOW  → 모듈 OFF
  2) 150ms 대기
  3) RTS=False, DTR=False → PWREN HIGH → 모듈 ON
  4) 500ms 부팅 대기
  5) 0x55 전송 → 자동 보레이트 감지
  6) 모듈이 0x00(자신의 주소)으로 응답 → 통신 준비 완료

에러 복구 (아이들/전원 순간 차단 후 자동 처리):
  - 명령 타임아웃 감지 → _reinitialize() 자동 호출
  - PWREN 재토글 + 0x55 재전송 → 이전 레이저 상태 복원 → 명령 재시도
"""

import struct
import threading
import time
import logging
from typing import Callable, Optional

try:
    import serial
except ImportError:
    raise ImportError("pyserial 미설치: pip install pyserial")

logger = logging.getLogger(__name__)


# ── 상태 코드 (PDF 6.6절) ──────────────────────────────────────────────
STATUS_CODES = {
    0x0000: '정상',
    0x0001: '입력 전력 부족 (전압 ≥ 2.2V 필요)',
    0x0002: '내부 오류 (무시 가능)',
    0x0003: '모듈 온도 너무 낮음 (< -20℃)',
    0x0004: '모듈 온도 너무 높음 (> +40℃)',
    0x0005: '측정 대상 범위 초과',
    0x0006: '잘못된 측정 결과',
    0x0007: '배경 광도 너무 강함',
    0x0008: '레이저 신호 너무 약함',
    0x0009: '레이저 신호 너무 강함',
    0x000A: '하드웨어 오류 1',
    0x000B: '하드웨어 오류 2',
    0x000C: '하드웨어 오류 3',
    0x000D: '하드웨어 오류 4',
    0x000E: '하드웨어 오류 5',
    0x000F: '레이저 신호 불안정',
    0x0010: '하드웨어 오류 6',
    0x0011: '하드웨어 오류 7',
    0x0081: '잘못된 프레임',
}


class LDSController:
    """JRT U8XX 레이저 거리 센서 바이너리 프로토콜 제어기"""

    # ── 바이너리 명령 프레임 (PDF 6.4절) ──────────────────────────────
    CMD_BAUD_DETECT  = bytes([0x55])
    CMD_LASER_ON     = bytes([0xAA, 0x00, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x01, 0xC1])
    CMD_LASER_OFF    = bytes([0xAA, 0x00, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x00, 0xC0])
    CMD_MEASURE_ONCE_AUTO = bytes([0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x21])
    CMD_MEASURE_ONCE_SLOW = bytes([0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x01, 0x22])
    CMD_MEASURE_ONCE_FAST = bytes([0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x02, 0x23])
    CMD_MEASURE_CONT_AUTO = bytes([0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x25])
    CMD_MEASURE_CONT_SLOW = bytes([0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x05, 0x26])
    CMD_MEASURE_CONT_FAST = bytes([0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x06, 0x27])

    # 측정 모드별 명령 + 타임아웃 (slow: 긴 광자 누적 → 원거리·저반사 측정 가능)
    MEASURE_MODES = {
        'slow': {'once': CMD_MEASURE_ONCE_SLOW, 'cont': CMD_MEASURE_CONT_SLOW, 'timeout': 8.0},
        'auto': {'once': CMD_MEASURE_ONCE_AUTO, 'cont': CMD_MEASURE_CONT_AUTO, 'timeout': 5.0},
        'fast': {'once': CMD_MEASURE_ONCE_FAST, 'cont': CMD_MEASURE_CONT_FAST, 'timeout': 3.0},
    }
    CMD_STOP_CONT    = bytes([0x58])  # ASCII 'X'
    CMD_READ_STATUS  = bytes([0xAA, 0x80, 0x00, 0x00, 0x80])
    # REG_BAT_VLTG 읽기: 공급 전압 BCD (mV) 읽기
    # CS = (0x80 + 0x00 + 0x06) & 0xFF = 0x86
    CMD_READ_VOLTAGE = bytes([0xAA, 0x80, 0x00, 0x06, 0x86])

    # ── 응답 길이 ──────────────────────────────────────────────────────
    RESP_LEN_ACK  = 9   # 쓰기 명령 에코 응답
    RESP_LEN_MEAS = 13  # 측정 결과 응답

    # ── 타이밍 상수 ────────────────────────────────────────────────────
    PWREN_OFF_WAIT      = 0.15  # PWREN LOW 유지 시간(초)
    BOOT_WAIT           = 0.50  # PWREN HIGH → 부팅 완료 대기(초)
    BAUD_DETECT_WAIT    = 0.40  # 0x55 전송 후 응답 대기(초)
    BAUD_DETECT_RETRY   = 3     # 0x55 재시도 횟수
    LASER_STABILIZE     = 0.40  # 레이저 ON 후 광학계 안정화(초)
    ACK_TIMEOUT         = 0.60  # 쓰기 명령 에코 대기 최대(초)
    MEASURE_TIMEOUT     = 5.0   # 기본 타임아웃 (auto 모드); 실제 사용은 MEASURE_MODES[mode]['timeout']
    SERIAL_BYTE_TIMEOUT = 0.10  # ser.read(1) 블로킹 최대(초) — stop 응답성
    CONT_MAX_FAIL       = 3     # 연속 측정 연속 실패 허용 횟수 → 재초기화
    WEAK_SIGNAL_MAX_RETRY = 3   # 0x0008 신호 약함 시 Slow 모드 최대 재시도 횟수

    def __init__(self, port: str = '/dev/ttyLDS', baudrate: int = 19200,
                 measure_mode: str = 'slow'):
        self.port       = port
        self.baudrate   = baudrate
        self.ser: Optional[serial.Serial] = None
        self.is_powered     = False
        self.is_continuous  = False
        self.measure_mode   = measure_mode if measure_mode in self.MEASURE_MODES else 'slow'
        self.voltage_mv: Optional[int] = None  # 공급 전압 캐시 (mV), None=미측정
        self._initialized   = False  # 0x55 자동 보레이트 감지 완료 여부

        self._lock           = threading.Lock()
        self._stop_event     = threading.Event()
        self._data_callback: Optional[Callable[[dict], None]] = None
        self._continuous_thread: Optional[threading.Thread] = None

    # ── 연결 / 해제 ───────────────────────────────────────────────────

    def connect(self) -> bool:
        """포트 열기 + PWREN 제어 + 자동 보레이트 초기화"""
        try:
            self.ser = serial.Serial(
                port      = self.port,
                baudrate  = self.baudrate,
                bytesize  = serial.EIGHTBITS,
                parity    = serial.PARITY_NONE,
                stopbits  = serial.STOPBITS_ONE,
                timeout   = self.SERIAL_BYTE_TIMEOUT,
                xonxoff   = False,
                rtscts    = False,
                dsrdtr    = False,
            )
            logger.info("포트 열림: %s @ %d bps", self.port, self.baudrate)
        except serial.SerialException as exc:
            logger.error("포트 열기 실패: %s", exc)
            return False

        return self._do_startup()

    def disconnect(self):
        """연속 측정 중지 후 포트 닫기"""
        self.stop_continuous()
        if self.ser and self.ser.is_open:
            try:
                self.ser.setRTS(True)   # PWREN LOW → 모듈 OFF
                self.ser.setDTR(True)
            except Exception:
                pass
            self.ser.close()
            self._initialized = False
            self.is_powered   = False
            logger.info("포트 닫힘: %s", self.port)

    def is_connected(self) -> bool:
        return (self.ser is not None
                and self.ser.is_open
                and self._initialized)

    # ── 전원(레이저) 제어 ─────────────────────────────────────────────

    def power_on(self) -> dict:
        """레이저 ON (레지스터 0x01BE = 0x01)"""
        result = self._write_cmd_with_retry(self.CMD_LASER_ON, 'LASER_ON')
        if result['success']:
            self.is_powered = True
            time.sleep(self.LASER_STABILIZE)
        return result

    def power_off(self) -> dict:
        """레이저 OFF (레지스터 0x01BE = 0x00) + 연속 측정 중지"""
        if self.is_continuous:
            self.stop_continuous(_restore_laser=False)  # 어차피 레이저 OFF할 것이므로 복원 불필요
            time.sleep(0.5)
            self._flush_input()
        result = self._write_cmd_with_retry(self.CMD_LASER_OFF, 'LASER_OFF')
        if result['success']:
            self.is_powered = False
        return result

    # ── 측정 ─────────────────────────────────────────────────────────

    def measure_once(self) -> dict:
        """1회 자동 측정 → mm 단위 결과 (실패 시 자동 재초기화 후 1회 재시도)"""
        if not self.ser or not self.ser.is_open:
            return _err('포트가 열려 있지 않습니다.')
        if not self.is_powered:
            return _err('레이저가 OFF 상태입니다. 먼저 전원을 ON 하세요.')
        if self.is_continuous:
            return _err('연속 측정 중입니다. 먼저 중지하세요.')

        self._stop_event.clear()  # 연속 중지 후 남은 stop 신호 방어적 제거
        result = self._do_measure_once()

        # 타임아웃 → 자동 재초기화 후 재시도
        if result is None:
            logger.warning("측정 타임아웃 — 자동 재초기화 후 재시도")
            if self._reinitialize():
                result = self._do_measure_once()

        # 0x0008 '레이저 신호 너무 약함' → Slow 모드로 자동 폴백
        if (result is not None
                and not result.get('success')
                and '신호 너무 약함' in result.get('error', '')
                and self.measure_mode != 'slow'):
            logger.warning("신호 약함(0x0008) 감지 — Slow 모드로 자동 폴백 측정")
            result = self._do_measure_once('slow')

        # Slow 모드에서도 0x0008 → WEAK_SIGNAL_MAX_RETRY 회 자동 재시도
        # 신호가 경계값 근처일 때 여러 번 시도하면 통합 시간 누적으로 성공 확률 향상
        for _retry in range(self.WEAK_SIGNAL_MAX_RETRY):
            if result is not None and result.get('success'):
                break
            if not (result is not None
                    and not result.get('success')
                    and '신호 너무 약함' in result.get('error', '')):
                break  # 다른 오류 → 재시도 불필요
            logger.warning("신호 약함 — Slow 재시도 (%d/%d)",
                           _retry + 1, self.WEAK_SIGNAL_MAX_RETRY)
            result = self._do_measure_once('slow')

        # 측정 후 레이저 ON 상태 복원 (센서가 측정 후 레이저를 끌 수 있음)
        self._restore_laser()

        return result or _err('측정 실패 (재초기화 후에도 응답 없음)')

    def start_continuous(self, callback: Callable[[dict], None]) -> dict:
        """연속 자동 측정 시작"""
        if not self.ser or not self.ser.is_open:
            return _err('포트가 열려 있지 않습니다.')
        if not self.is_powered:
            return _err('레이저가 OFF 상태입니다. 먼저 전원을 ON 하세요.')
        if self.is_continuous:
            return _err('이미 연속 측정 중입니다.')

        self._data_callback = callback
        self._stop_event.clear()
        self._continuous_thread = threading.Thread(
            target=self._continuous_loop, daemon=True, name='LDS-Cont')
        self._continuous_thread.start()
        self.is_continuous = True
        logger.info("연속 측정 시작")
        return {'success': True}

    def stop_continuous(self, _restore_laser: bool = True) -> dict:
        """연속 측정 중지

        _restore_laser: True(기본값)면 중지 후 레이저 ON 상태 복원.
                        power_off()처럼 레이저를 꺼야 할 경우 False 전달.
        """
        if not self.is_continuous:
            return _err('연속 측정 중이 아닙니다.')

        was_powered = self.is_powered

        self._stop_event.set()
        if self.ser and self.ser.is_open:
            try:
                with self._lock:
                    self.ser.write(self.CMD_STOP_CONT)
            except Exception as exc:
                logger.warning("연속 중지 명령 전송 오류: %s", exc)

        if self._continuous_thread and self._continuous_thread.is_alive():
            self._continuous_thread.join(timeout=3.0)

        self._stop_event.clear()  # 스레드 종료 확인 후 stop 신호 초기화
        self.is_continuous = False

        # 연속 중지 후 레이저 ON 상태 복원
        if _restore_laser and was_powered:
            self._restore_laser()

        logger.info("연속 측정 중지")
        return {'success': True}

    def get_status(self) -> dict:
        return {
            'connected':    self.is_connected(),
            'port':         self.port,
            'powered':      self.is_powered,
            'continuous':   self.is_continuous,
            'initialized':  self._initialized,
            'measure_mode': self.measure_mode,
            'voltage_mv':   self.voltage_mv,
        }

    def set_measure_mode(self, mode: str) -> dict:
        """측정 모드 설정 (slow/auto/fast). 연속 측정 중에는 변경 불가."""
        if mode not in self.MEASURE_MODES:
            return _err(f'알 수 없는 측정 모드: {mode!r}. 가능한 값: slow, auto, fast')
        if self.is_continuous:
            return _err('연속 측정 중에는 모드를 변경할 수 없습니다.')
        self.measure_mode = mode
        logger.info("측정 모드 변경: %s", mode)
        return {'success': True, 'measure_mode': mode}

    def read_voltage(self) -> dict:
        """
        REG_BAT_VLTG (0x0006) 읽기 → 공급 전압.

        응답 형식: [AA][00][00][06][00][01][V_H][V_L][CS]
          V_H, V_L: BCD 인코딩 mV (예: 0x33 0x19 → 3319 mV = 3.319 V)
        전압이 2.8V 미만이면 레이저 출력 저하로 0x0008 유발 가능.
        """
        if not self.is_connected():
            return _err('센서 연결 안됨')
        with self._lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(self.CMD_READ_VOLTAGE)
                resp = self._read_nbytes(9, timeout=0.5)
            except serial.SerialException as exc:
                return _err(f'전압 읽기 오류: {exc}')
        if len(resp) < 9 or resp[0] != 0xAA:
            return _err(f'전압 응답 오류 ({resp.hex() if resp else "없음"})')
        v_h, v_l = resp[6], resp[7]
        mv = ((v_h >> 4) * 1000 + (v_h & 0x0F) * 100 +
              (v_l >> 4) * 10  + (v_l & 0x0F))
        self.voltage_mv = mv
        voltage = mv / 1000.0
        logger.info("공급 전압 갱신: %.3f V", voltage)
        if voltage < 2.8:
            logger.warning("전압 낮음 (%.3f V) — 레이저 출력 저하 → 측정 거리 단축 가능", voltage)
        return {'success': True, 'voltage_v': voltage, 'voltage_mv': mv}

    # ── 핵심: 시작 시퀀스 ─────────────────────────────────────────────

    def _do_startup(self) -> bool:
        """
        PWREN 토글 + 0x55 자동 보레이트 감지 시퀀스 (PDF 6.2절).
        connect() 최초 실행 및 _reinitialize() 에서 호출.
        """
        try:
            # 1) PWREN LOW → 완전 OFF
            self.ser.setRTS(True)
            self.ser.setDTR(True)
            time.sleep(self.PWREN_OFF_WAIT)

            # 2) PWREN HIGH → ON + 부팅 대기
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            time.sleep(self.BOOT_WAIT)
            self.ser.reset_input_buffer()

            # 3) 0x55 자동 보레이트 감지 — 최대 BAUD_DETECT_RETRY 회 재시도
            for attempt in range(self.BAUD_DETECT_RETRY):
                self.ser.write(self.CMD_BAUD_DETECT)
                resp = self._read_nbytes(1, timeout=self.BAUD_DETECT_WAIT)
                if resp and resp[0] == 0x00:
                    self._initialized = True
                    logger.info("자동 보레이트 감지 성공 (시도 %d)", attempt + 1)
                    self._update_voltage()  # 초기 전압 읽기
                    return True
                logger.debug("0x55 응답 없음 (시도 %d/%d): %r",
                             attempt + 1, self.BAUD_DETECT_RETRY, resp)
                time.sleep(0.20)

            logger.error("자동 보레이트 감지 실패 (%d회 시도)", self.BAUD_DETECT_RETRY)
            return False

        except serial.SerialException as exc:
            logger.error("시작 시퀀스 오류: %s", exc)
            return False

    # ── 핵심: 자동 재초기화 ───────────────────────────────────────────

    def _reinitialize(self) -> bool:
        """
        아이들/전원 순간 차단 후 센서가 리셋된 경우 자동 복구.

        복구 순서:
          1. _initialized = False (상태 무효화)
          2. 연속 중지 명령 전송 (혹시 연속 모드 잔류 시)
          3. 버퍼 플러시
          4. PWREN 재토글 + 0x55 재전송
          5. 이전 레이저 상태 복원 (is_powered 기준)
        """
        logger.warning("자동 재초기화 시작 (이전 레이저 상태: %s)",
                       'ON' if self.is_powered else 'OFF')
        self._initialized = False
        prev_powered = self.is_powered

        # 연속 중지 명령 시도 (무시해도 됨)
        try:
            self.ser.write(self.CMD_STOP_CONT)
            time.sleep(0.15)
        except Exception:
            pass

        self._flush_input()

        # 시작 시퀀스 재실행
        if not self._do_startup():
            logger.error("재초기화 실패: 시작 시퀀스 불가")
            return False

        # 레이저 상태 복원
        if prev_powered:
            r = self._try_write_cmd(self.CMD_LASER_ON, 'LASER_ON(복원)')
            if r['success']:
                self.is_powered = True
                time.sleep(self.LASER_STABILIZE)
                logger.info("재초기화 성공 — 레이저 ON 복원 완료")
            else:
                self.is_powered = False
                logger.warning("재초기화 성공 — 레이저 ON 복원 실패")
        else:
            logger.info("재초기화 성공 — 레이저 OFF 상태 유지")

        return True

    def _restore_laser(self):
        """
        레이저 ON 재전송 + 안정화 대기.

        측정/연속 중지 후 레이저 ON 상태를 명시적으로 보장한다.
        센서가 측정 완료 후 내부적으로 레이저를 끌 수 있으므로,
        is_powered 플래그와 실제 센서 상태를 일치시키기 위해 사용.
        """
        r = self._try_write_cmd(self.CMD_LASER_ON, 'LASER_ON(상태유지)')
        if r['success']:
            self.is_powered = True
            time.sleep(self.LASER_STABILIZE)
            logger.debug("레이저 ON 상태 유지 확인")
        else:
            logger.warning("레이저 ON 상태 유지 실패: %s", r.get('error'))

    def _update_voltage(self):
        """
        공급 전압 캐시 업데이트 (내부용, 잠금 없음).

        _do_startup() 직후 호출되어 voltage_mv를 초기화한다.
        전압 < 2.8V 시 경고 로그.
        """
        try:
            self.ser.write(self.CMD_READ_VOLTAGE)
            resp = self._read_nbytes(9, timeout=0.5)
            if len(resp) >= 9 and resp[0] == 0xAA:
                v_h, v_l = resp[6], resp[7]
                mv = ((v_h >> 4) * 1000 + (v_h & 0x0F) * 100 +
                      (v_l >> 4) * 10  + (v_l & 0x0F))
                self.voltage_mv = mv
                v = mv / 1000.0
                level = 'OK' if mv >= 2800 else 'LOW'
                logger.info("공급 전압: %.3f V [%s]", v, level)
                if mv < 2800:
                    logger.warning(
                        "전압 낮음 (%.3f V < 2.8V) — 레이저 출력 저하로 0x0008 오류 유발 가능. "
                        "USB 케이블/포트 교체 또는 외부 3.3V 전원 권장", v)
        except Exception as exc:
            logger.debug("전압 초기 읽기 실패 (무시): %s", exc)

    # ── 측정 / 연속 루프 내부 ─────────────────────────────────────────

    def _do_measure_once(self, mode: Optional[str] = None) -> Optional[dict]:
        """
        1회 측정 명령 전송 + 응답 프레임 수신.
        mode: 'slow'|'auto'|'fast', None이면 self.measure_mode 사용.
        타임아웃 시 None 반환 (호출자가 재초기화 여부 결정).
        """
        m = mode if mode in self.MEASURE_MODES else self.measure_mode
        cfg = self.MEASURE_MODES[m]
        with self._lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cfg['once'])
            except serial.SerialException as exc:
                logger.error("측정 명령 전송 오류: %s", exc)
                return None

        return self._read_measure_frame(timeout=cfg['timeout'])

    def _continuous_loop(self):
        """연속 측정 수신 루프 — 연속 실패 시 자동 재초기화"""
        logger.debug("연속 루프 진입")
        fail_count = 0
        cfg = self.MEASURE_MODES[self.measure_mode]

        # 연속 측정 명령 전송
        with self._lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cfg['cont'])
            except serial.SerialException as exc:
                logger.error("연속 명령 전송 실패: %s", exc)
                self.is_continuous = False
                return

        while not self._stop_event.is_set():
            frame = self._read_measure_frame(timeout=cfg['timeout'])

            if frame is None:
                # 타임아웃 — 연속 실패 카운트
                fail_count += 1
                logger.warning("연속 측정 타임아웃 (%d/%d)",
                               fail_count, self.CONT_MAX_FAIL)

                if fail_count >= self.CONT_MAX_FAIL:
                    logger.warning("연속 측정 연속 실패 — 자동 재초기화 시도")
                    if self._reinitialize() and self.is_powered and not self._stop_event.is_set():
                        # 연속 측정 재시작
                        with self._lock:
                            try:
                                self.ser.reset_input_buffer()
                                self.ser.write(cfg['cont'])
                                fail_count = 0
                                logger.info("연속 측정 재개")
                            except serial.SerialException as exc:
                                logger.error("연속 재시작 실패: %s", exc)
                                break
                    else:
                        logger.error("재초기화 후 연속 측정 재개 불가 — 루프 종료")
                        break
            else:
                fail_count = 0
                if self._data_callback:
                    try:
                        self._data_callback(frame)
                    except Exception as exc:
                        logger.error("콜백 오류: %s", exc)

        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        logger.debug("연속 루프 종료")

    # ── 쓰기 명령 (재초기화 포함) ─────────────────────────────────────

    def _write_cmd_with_retry(self, cmd: bytes, label: str) -> dict:
        """
        쓰기 명령 전송. 타임아웃(응답 없음) 시 자동 재초기화 후 1회 재시도.
        """
        if not self.ser or not self.ser.is_open:
            return _err(f'[{label}] 포트 닫힘')

        result = self._try_write_cmd(cmd, label)

        if not result['success'] and '응답 없음' in result.get('error', ''):
            logger.warning("[%s] 응답 없음 — 자동 재초기화 후 재시도", label)
            if self._reinitialize():
                result = self._try_write_cmd(cmd, label)
            else:
                return _err(f'[{label}] 재초기화 실패')

        return result

    def _try_write_cmd(self, cmd: bytes, label: str) -> dict:
        """쓰기 명령 1회 전송 + 에코 응답 확인 (재시도 없음)"""
        with self._lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cmd)
                resp = self._read_nbytes(self.RESP_LEN_ACK, timeout=self.ACK_TIMEOUT)
            except serial.SerialException as exc:
                return _err(f'[{label}] 시리얼 오류: {exc}')

        if not resp:
            return _err(f'[{label}] 응답 없음 (타임아웃)')

        # 에코 확인: 전송한 프레임과 동일한지
        if resp[:len(cmd)] == cmd:
            logger.info("[%s] 성공 (에코 확인)", label)
            return {'success': True, 'raw': resp.hex()}

        # 0xEE 오류 프레임 확인
        if resp[0] == 0xEE and len(resp) >= 8:
            err_code = self._parse_error_code(resp)
            msg = STATUS_CODES.get(err_code, f'오류 코드 0x{err_code:04X}')
            return _err(f'[{label}] 센서 오류: {msg}')

        logger.info("[%s] 응답 수신 (raw: %s)", label, resp.hex())
        return {'success': True, 'raw': resp.hex()}

    # ── 수신 헬퍼 ────────────────────────────────────────────────────

    def _read_measure_frame(self, timeout: float) -> Optional[dict]:
        """
        측정 응답 프레임(13바이트) 또는 오류 프레임 수신 후 파싱.
        헤더(0xAA/0xEE)를 찾을 때까지 잡음 바이트를 건너뜀.
        타임아웃 시 None 반환.

        JRT U81 오류 프레임 구조 (두 종류):
          단형(9B):  [EE][addr][regH][regL][cntH=00][cntL=01][errH][errL][CS]
                     — count=1: 거리 없음, errH/errL 에 상태 코드
          장형(13B): [EE][addr][regH][regL][cntH=00][cntL=04][D3][D2][D1][D0][errH][errL][CS]
                     — count=4: bytes 6-9 에 거리(mm), bytes 10-11 에 상태 코드
          count 필드(bytes 4-5)로 어느 형식인지 판단.
        """
        try:
            header = self._read_until_header(timeout=timeout)
            if header is None:
                return None

            if header == 0xEE:
                # 최소 8바이트 수신 → count 필드 + 에러코드 파악
                rest = self._read_nbytes(8, timeout=0.5)
                frame = bytes([0xEE]) + rest
                logger.debug("오류 프레임 수신 (%dB): %s", len(frame), frame.hex())

                # count 필드 (bytes 4-5) 로 프레임 형식 판단
                cnt = ((frame[4] << 8) | frame[5]) if len(frame) >= 6 else 1

                if cnt == 4:
                    # 장형 오류 프레임 — 나머지 4바이트 추가 수신
                    extra = self._read_nbytes(4, timeout=0.3)
                    frame = frame + extra
                    logger.debug("장형 오류 프레임 전체 (%dB): %s", len(frame), frame.hex())

                    if len(frame) >= 13:
                        dist_mm  = struct.unpack('>I', frame[6:10])[0]
                        err_code = (frame[10] << 8) | frame[11]
                        msg = STATUS_CODES.get(err_code, f'오류 코드 0x{err_code:04X}')
                        logger.warning(
                            "센서 오류 프레임(13B): %s (0x%04X) — 측정값: %d mm (저신뢰도)",
                            msg, err_code, dist_mm)

                        # 유효 범위 내이면 저신뢰도 거리 결과로 반환
                        if 30 <= dist_mm <= 25000:
                            return {
                                'success':        True,
                                'mm':             dist_mm,
                                'm':              dist_mm / 1000.0,
                                'signal_quality': 9999,  # SQ 불량 (오류 프레임)
                                'low_confidence': True,
                                'timestamp':      time.time(),
                            }

                # 단형(9B) 오류 프레임 — 거리 데이터 없음
                err_code = self._parse_error_code(frame)
                msg = STATUS_CODES.get(err_code, f'오류 코드 0x{err_code:04X}')
                logger.warning("센서 오류 프레임: %s (0x%04X)", msg, err_code)
                return {
                    'success':   False,
                    'error':     f'센서 오류: {msg}',
                    'timestamp': time.time(),
                }

            rest = self._read_nbytes(12, timeout=0.5)
            if len(rest) < 12:
                logger.warning("응답 프레임 불완전: 1+%dB 수신", len(rest))
                return None

            return self._parse_measure_frame(bytes([header]) + rest)

        except serial.SerialException as exc:
            logger.error("수신 오류: %s", exc)
            return None

    def _read_until_header(self, timeout: float) -> Optional[int]:
        """
        0xAA 또는 0xEE 헤더 바이트가 나올 때까지 스트림을 소비.
        ser.timeout = SERIAL_BYTE_TIMEOUT(0.1s) → _stop_event를 0.1s마다 확인.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return None
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] in (0xAA, 0xEE):
                return b[0]
            logger.debug("헤더 동기화 스킵: 0x%02X", b[0])
        return None

    def _read_nbytes(self, n: int, timeout: float = 0.5) -> bytes:
        """
        정확히 n 바이트 수신.
        ser.timeout이 짧으므로 루프로 재시도하며 지정된 timeout 내 수신 시도.
        """
        buf = b''
        deadline = time.monotonic() + timeout
        while len(buf) < n and time.monotonic() < deadline:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf += chunk
        return buf

    def _flush_input(self):
        """입력 버퍼 비우기"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.reset_input_buffer()
        except Exception:
            pass

    # ── 파싱 헬퍼 ────────────────────────────────────────────────────

    @staticmethod
    def _parse_error_code(frame: bytes) -> int:
        """
        오류 응답 프레임에서 상태 코드 추출.

        프레임 구조 (PDF 6.4.16):
          [0xEE][addr][regH][regL][cntH][cntL][errH][errL][checksum]
           idx0   1    2     3     4     5     6     7      8

        → err_code = (frame[6] << 8) | frame[7]
          ※ 수정 전 버그: (frame[7]<<8)|frame[8] (checksum을 오류 코드로 오파싱)
        """
        if len(frame) < 8:
            return 0xFFFF
        return (frame[6] << 8) | frame[7]

    @staticmethod
    def _parse_measure_frame(frame: bytes) -> dict:
        """
        13바이트 측정 응답 프레임 파싱.

        프레임 구조 (PDF 6.4.10절):
          [AA][addr][regH][regL][cntH][cntL][D3][D2][D1][D0][SQH][SQL][CS]
           0    1     2     3    4     5    6   7   8   9   10   11   12

          Distance = big-endian uint32 (bytes 6~9), 단위: mm
          SQ       = big-endian uint16 (bytes 10~11)
          CS       = sum(bytes[1:12]) & 0xFF
        """
        if len(frame) < 13:
            return _err(f'프레임 길이 부족: {len(frame)}B')

        # 체크섬 검증
        expected_cs = sum(frame[1:12]) & 0xFF
        if frame[12] != expected_cs:
            logger.debug("체크섬 불일치: 수신 0x%02X, 계산 0x%02X (데이터는 사용)",
                         frame[12], expected_cs)

        dist_mm = struct.unpack('>I', frame[6:10])[0]
        sq      = struct.unpack('>H', frame[10:12])[0]

        return {
            'success':        True,
            'mm':             dist_mm,
            'm':              dist_mm / 1000.0,
            'signal_quality': sq,
            'timestamp':      time.time(),
        }


def _err(message: str) -> dict:
    """에러 결과 딕셔너리 생성"""
    logger.warning(message)
    return {'success': False, 'error': message}
