import sys
import rclpy
import std_msgs.msg as std_msgs
import tf2_ros
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, QToolTip, QGridLayout, QLabel, QLineEdit, QSlider,
                             QVBoxLayout, QHBoxLayout)

from gt3_rainbow.cobot import *

# 전역 RCL 초기화 상태 추적
_rcl_initialized = False

class pose_Subscriber(Node):

    def __init__(self):
        super().__init__('pose_rcv')
        
        self.sub_pose = self.create_subscription(std_msgs.String, '/robot_pose', self.cmd_Callback, 10)
        self.pose_pub = self.create_publisher(std_msgs.String,"/current_pose", 1)
        self.cnt = 0
        self.current_pose = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self, 10)

        self.msg = std_msgs.String()
        self.robot_connect = False
        self.pose_connect = False

        self.prev_z = 0
        self.current_z = 0
    
    def cmd_Callback(self, msg):
        print(msg.data)
        received_data = msg.data.split(';')
        if received_data[0] == 'rainbow_start':
            ToCB('192.168.1.13')
            self.robot_connect = True
        elif received_data[0] == 'rainbow_stop':
            DisConnectToCB()
        elif received_data[0] == 'ik' and received_data[1] == 'L':
            pose = eval(received_data[2])
            MoveL(float(pose[0]), float(pose[1]), float(pose[2]), float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6]), float(pose[7]))
            
        elif received_data[0] == 'iks' and received_data[1] == 'L':
            pose = eval(received_data[2])
            MoveITPL_Add(float(pose[0]), float(pose[1]), float(pose[2]), float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6]), self.cnt)   
            self.cnt += 1
            
        elif received_data[0] == 'iksr':
            MoveITPL_Run(float(received_data[1]), ITPL_RTYPE.CA_INTENDED, eval(received_data[2])) 
            time.sleep(0.1)
            MoveITPL_Clear() 
            self.cnt = 0
            
        elif received_data[0] == 'pb_add' and received_data[1] == 'L':
            pose = eval(received_data[2])
            MovePB_Add(float(pose[0]), float(pose[1]), float(pose[2]), float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6]), BLEND_OPTION.RATIO, 0.6)   
            
        elif received_data[0] == 'pb_run':
            MovePB_Run(float(received_data[1]), BLEND_RTYPE.INTENDED) 
            time.sleep(0.1)
            MovePB_Clear()        
            
        elif received_data[0] == 'fk' and received_data[1] == 'J':
            pose = eval(received_data[2])
            MoveJ(float(pose[0]), float(pose[1]), float(pose[2]), float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6]), float(pose[7]))
        
        elif received_data[0] == 'servo' and received_data[1] == 'J':
            pose = eval(received_data[2])
            ServoJ(float(pose[0]), float(pose[1]), float(pose[2]), float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6]), float(pose[7]), float(pose[8]), float(pose[9]))
        
        elif received_data[0] == 'servo' and received_data[1] == 'L':
            pose = eval(received_data[2])
            ServoL(float(pose[0]), float(pose[1]), float(pose[2]), float(pose[3]), float(pose[4]), float(pose[5]), float(pose[6]), float(pose[7]), float(pose[8]), float(pose[9]))
        
        elif received_data[0] == 'gripper':
            if received_data[1] == '0':
                CBDigitalOut(0.0, DOUT_SET.LOW)
            else:
                CBDigitalOut(0.0, DOUT_SET.HIGH)

class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        grid = QGridLayout()
        self.setLayout(grid)
        global spd, setspd_sld
        global ip_get, ip

        # Connect button and IP input
        global btn
        btn = QPushButton('Connect', self)
        ip = QLineEdit('192.168.1.13')
        init_btn = QPushButton('Initialized')
        ip_hgrid = QHBoxLayout()
        ip_hgrid.addWidget(ip)
        ip_hgrid.addWidget(init_btn)
        grid.addLayout(ip_hgrid, 0, 1)
        grid.addWidget(btn, 0, 0)
        btn.setToolTip('This is a <b>QPushButton</b> widget')
        btn.resize(btn.sizeHint())
        btn.setStyleSheet("background-color: red")
        btn.clicked.connect(lambda: ToCB(ip.text()))
        init_btn.clicked.connect(CobotInit)

        # Timer for data updates
        timer = QTimer(self)
        timer.setInterval(10)
        timer.timeout.connect(self.datareset)
        timer.start()

        # Status display
        status_hgrid = QHBoxLayout()
        mode_btn = QPushButton('Mode')
        global mode_lb, robot_lb
        mode_lb = QLabel('Mode : ')
        robot_lb = QLabel('Robot : ')
        grid.addWidget(mode_btn, 1, 0)
        status_hgrid.addWidget(mode_lb)
        status_hgrid.addWidget(robot_lb)
        grid.addLayout(status_hgrid, 1, 1)
        global mode
        mode = PG_MODE.SIMULATION
        mode_btn.clicked.connect(lambda: SetProgramMode(mode))

        # Speed control
        setspd_hgrid = QHBoxLayout()
        setspd_btn = QPushButton('Speed Bar')
        setspd_sld = QSlider(Qt.Horizontal)
        setspd_sld.setTickPosition(1)
        setspd_sld.setMaximum(100)
        setspd_sld.setMinimum(0)
        spd = float(setspd_sld.value()) * 0.01
        global spd_lb
        spd_lb = QLabel(str(float(setspd_sld.value()) * 0.01))
        setspd_hgrid.addWidget(spd_lb)
        setspd_hgrid.addWidget(setspd_sld)
        grid.addLayout(setspd_hgrid, 2, 1)
        grid.addWidget(setspd_btn, 2, 0)
        setspd_btn.clicked.connect(lambda: SetBaseSpeed(float(setspd_sld.value()) * 0.01))

        # Joint control
        movej_hgrid1 = QHBoxLayout()
        movej_hgrid2 = QHBoxLayout()
        movej_vgrid = QVBoxLayout()
        j0_lb = QLabel('j0 : ')
        j1_lb = QLabel('j1 : ')
        j2_lb = QLabel('j2 : ')
        j3_lb = QLabel('j3 : ')
        j4_lb = QLabel('j4 : ')
        j5_lb = QLabel('j5 : ')
        spdj_lb = QLabel('speed : ')
        accj_lb = QLabel('accel : ')
        j0_le = QLineEdit()
        j1_le = QLineEdit()
        j2_le = QLineEdit()
        j3_le = QLineEdit()
        j4_le = QLineEdit()
        j5_le = QLineEdit()
        spdj_le = QLineEdit()
        accj_le = QLineEdit()
        movej_btn = QPushButton('Move J')
        movej_hgrid1.addWidget(j0_lb)
        movej_hgrid1.addWidget(j0_le)
        movej_hgrid1.addWidget(j1_lb)
        movej_hgrid1.addWidget(j1_le)
        movej_hgrid1.addWidget(j2_lb)
        movej_hgrid1.addWidget(j2_le)
        movej_hgrid2.addWidget(j3_lb)
        movej_hgrid2.addWidget(j3_le)
        movej_hgrid2.addWidget(j4_lb)
        movej_hgrid2.addWidget(j4_le)
        movej_hgrid2.addWidget(j5_lb)
        movej_hgrid2.addWidget(j5_le)
        movej_hgrid1.addWidget(spdj_lb)
        movej_hgrid1.addWidget(spdj_le)
        movej_hgrid2.addWidget(accj_lb)
        movej_hgrid2.addWidget(accj_le)
        movej_vgrid.addLayout(movej_hgrid1)
        movej_vgrid.addLayout(movej_hgrid2)
        grid.addWidget(movej_btn, 3, 0)
        grid.addLayout(movej_vgrid, 3, 1)
        movej_btn.clicked.connect(
            lambda: MoveJ(float(j0_le.text()), float(j1_le.text()), float(j2_le.text()), float(j3_le.text()),
                          float(j4_le.text()),
                          float(j5_le.text()), float(spdj_le.text()), float(accj_le.text())))

        # Cartesian control
        movel_hgrid1 = QHBoxLayout()
        movel_hgrid2 = QHBoxLayout()
        movel_vgrid = QVBoxLayout()
        x_lb = QLabel('x : ')
        y_lb = QLabel('y : ')
        z_lb = QLabel('z : ')
        rx_lb = QLabel('rx : ')
        ry_lb = QLabel('ry : ')
        rz_lb = QLabel('rz : ')
        spdl_lb = QLabel('speed : ')
        accl_lb = QLabel('accel : ')
        x_le = QLineEdit()
        y_le = QLineEdit()
        z_le = QLineEdit()
        rx_le = QLineEdit()
        ry_le = QLineEdit()
        rz_le = QLineEdit()
        spdl_le = QLineEdit()
        accl_le = QLineEdit()
        movel_btn = QPushButton('Move L')
        movel_hgrid1.addWidget(x_lb)
        movel_hgrid1.addWidget(x_le)
        movel_hgrid1.addWidget(y_lb)
        movel_hgrid1.addWidget(y_le)
        movel_hgrid1.addWidget(z_lb)
        movel_hgrid1.addWidget(z_le)
        movel_hgrid2.addWidget(rx_lb)
        movel_hgrid2.addWidget(rx_le)
        movel_hgrid2.addWidget(ry_lb)
        movel_hgrid2.addWidget(ry_le)
        movel_hgrid2.addWidget(rz_lb)
        movel_hgrid2.addWidget(rz_le)
        movel_hgrid1.addWidget(spdl_lb)
        movel_hgrid1.addWidget(spdl_le)
        movel_hgrid2.addWidget(accl_lb)
        movel_hgrid2.addWidget(accl_le)
        movel_vgrid.addLayout(movel_hgrid1)
        movel_vgrid.addLayout(movel_hgrid2)
        grid.addWidget(movel_btn, 4, 0)
        grid.addLayout(movel_vgrid, 4, 1)
        movel_btn.clicked.connect(
            lambda: MoveL(float(x_le.text()), float(y_le.text()), float(z_le.text()), float(rx_le.text()),
                          float(ry_le.text()),
                          float(rz_le.text()), float(spdl_le.text()), float(accl_le.text())))

        # Manual script
        mscript_btn = QPushButton('Manual Script')
        mscript_le = QLineEdit()
        grid.addWidget(mscript_btn, 5, 0)
        grid.addWidget(mscript_le, 5, 1)
        mscript_btn.clicked.connect(lambda: ManualScript(mscript_le.text()))

        # Tooltips
        QToolTip.setFont(QFont('SansSerif', 10))
        self.setToolTip('This is a <b>QWidget</b> widget')

        self.setWindowTitle('Rainbow Robotics')
        self.setWindowIcon(QIcon('robotics-ci-1.png'))
        self.setGeometry(300, 300, 300, 300)
        self.show()

    def datareset(self):
        global mode, mode_lb, robot_lb
        global spd_lb
        global btn

        setspd_f2 = round(float(setspd_sld.value()) * 0.01, 2)
        spd_lb.setText(str(setspd_f2))

        if IsRobotReal() == True:
            mode_lb.setText('Mode : REAL')
            mode = PG_MODE.SIMULATION
        elif IsRobotReal() == False:
            mode_lb.setText('Mode : SIMULATION')
            mode = PG_MODE.REAL

        if IsIdle() == True:
            robot_lb.setText('ROBOT : IDLE')
        elif IsIdle() == False:
            robot_lb.setText('ROBOT : RUN')

        if IsCommandSockConnect() == True & IsDataSockConnect() == True:
            btn.setStyleSheet("background-color: green")
            btn.setText('Connect')
        else:
            btn.setStyleSheet("background-color: red")
            btn.setText('Disconnect')
            ToCB('192.168.1.13')


def main(args=None):
    """
    ROS2 노드를 초기화하고 실행하는 메인 함수.
    RCL 중복 초기화 방지 로직 추가.
    """
    global _rcl_initialized
    
    # RCL 초기화 상태 확인
    if not _rcl_initialized:
        try:
            rclpy.init(args=args)
            _rcl_initialized = True
        except Exception as e:
            print(f"RCL already initialized or error: {e}")
            return
    
    ps = pose_Subscriber()
    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(ps)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f"Executor error: {e}")
    finally:
        # 안전한 종료 처리
        try:
            ps.destroy_node()
        except Exception as e:
            print(f"Node destruction error: {e}")
        
        # RCL 종료 - 중복 종료 방지
        if _rcl_initialized:
            try:
                rclpy.shutdown()
                _rcl_initialized = False
            except Exception as e:
                print(f"RCL shutdown error: {e}")


if __name__ == '__main__':
    main()