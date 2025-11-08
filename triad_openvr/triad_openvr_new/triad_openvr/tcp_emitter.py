import triad_openvr
import time
import sys
import struct
import socket
import threading

class TCPEmitter:
    def __init__(self, host='10.0.1.48', port=8051):
        self.host = host
        self.port = port
        self.sock = None
        self.connected = False
        self.running = True
        self.v = triad_openvr.triad_openvr()
        self.v.print_discovered_objects()
        
    def connect(self):
        """TCP 서버에 연결 시도"""
        while self.running and not self.connected:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5.0)  # 5초 타임아웃
                print(f"Connecting to TCP server at {self.host}:{self.port}...")
                self.sock.connect((self.host, self.port))
                self.connected = True
                print(f"Connected to TCP server at {self.host}:{self.port}")
                return True
            except socket.error as e:
                print(f"Connection failed: {e}. Retrying in 3 seconds...")
                if self.sock:
                    self.sock.close()
                time.sleep(3)
        return False
    
    def send_data(self, data):
        """데이터 전송 (재연결 로직 포함)"""
        if not self.connected:
            if not self.connect():
                return False
        
        try:
            # 데이터 크기를 먼저 전송 (4바이트)
            data_bytes = struct.pack('d'*len(data), *data)
            size_bytes = struct.pack('I', len(data_bytes))
            
            # 크기와 데이터 전송
            self.sock.sendall(size_bytes + data_bytes)
            return True
        except (socket.error, BrokenPipeError) as e:
            print(f"Send failed: {e}. Reconnecting...")
            self.connected = False
            if self.sock:
                self.sock.close()
            return False
    
    def run(self, interval=1/250):
        """메인 실행 루프"""
        print(f"Starting TCP emitter with interval: {interval:.4f} seconds")
        
        # 초기 연결
        if not self.connect():
            print("Failed to establish initial connection. Exiting.")
            return
        
        try:
            while self.running:
                start = time.time()
                
                # tracker_1의 포즈 데이터 가져오기 (Euler 각도 사용)
                try:
                    data = self.v.devices["tracker_1"].get_pose_quaternion()
                    if data:
                        # 데이터 전송 (x, y, z, pitch, yaw, roll) - 원래 순서
                        if self.send_data(data):
                            # 디버그 출력: 위치와 회전 정보 표시
                            print(f"\rSent: Pos({data[0]:.2f}, {data[1]:.2f}, {data[2]:.2f}) Rot(w:{data[3]:.1f} x:{data[4]:.1f} y:{data[5]:.1f} z{data[6]:.1f})", end="")
                        else:
                            print("\rFailed to send data, attempting reconnection...", end="")
                except KeyError:
                    print("\rTracker_1 not found. Waiting...", end="")
                except Exception as e:
                    print(f"\rError getting pose: {e}", end="")
                
                # 인터벌 유지
                sleep_time = interval - (time.time() - start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            print("\nShutting down TCP emitter...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """정리 작업"""
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        print("TCP emitter closed.")

def main():
    # 커맨드라인 인자 처리
    if len(sys.argv) == 1:
        interval = 1/250  # 기본값 250Hz
    elif len(sys.argv) == 2:
        try:
            interval = 1/float(sys.argv[1])
        except ValueError:
            print("Invalid frequency argument. Using default 250Hz.")
            interval = 1/250
    else:
        print("Usage: python tcp_emitter.py [frequency_hz]")
        print("Example: python tcp_emitter.py 250")
        return
    
    # TCP Emitter 실행
    emitter = TCPEmitter(host='10.0.1.48', port=8051)
    emitter.run(interval)

if __name__ == "__main__":
    main()