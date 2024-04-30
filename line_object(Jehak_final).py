#!/usr/bin/env python

import cv2
import numpy as np
import threading
import time
import rospy
import math

from sensor_msgs.msg import LaserScan
from pymycobot.myagv import MyAgv
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

if sys.platform == 'win32': #플랫폼에 따라 키보드임력 모듈 가져오기
    import msvcrt
else:
    import termios
    import tty

TwistMsg = Twist
agv = MyAgv("/dev/ttyAMA2", 115200)
cx_pit, cx, pi = 0, 0, 0.2
detect_distance = 5

def process_frame(frame):
    global cx_pit, cx
    height, width, _ = frame.shape              # 프레임의 높이, 너비 및 채널 정보를 가져옵니다.
    roi_height = int(height / 5)                # 관심 영역(ROI)의 높이를 계산합니다. 전체 높이의 5분의 1로 설정합니다.
    roi_top = height - roi_height               # 관심 영역(ROI)의 맨 위 위치를 계산합니다.
    roi = frame[roi_top:, :]                    # 관심 영역(ROI)을 추출합니다.
    center_line = width // 2                    # 관심 영역(ROI)의 중앙선 위치를 계산합니다.
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)  # 관심 영역(ROI)을 HSV 색 공간으로 변환합니다.

    lower_line_color = np.array([30, 100, 70], dtype=np.uint8)        # 라인을 감지하기 위한 HSV 임계값을 설정합니다.
    upper_line_color = np.array([69, 255, 255], dtype=np.uint8)
    line_mask = cv2.inRange(hsv, lower_line_color, upper_line_color) # HSV 이미지에서 라인에 대한 마스크를 생성합니다.
    line_contours, _ = cv2.findContours(line_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # 파란색 마스크에서 외곽선을 찾습니다.

    
    if line_contours: # 파란색 외곽선이 존재하는 경우
        max_contour = max(line_contours, key=cv2.contourArea) # 외곽선 중 가장 큰 것을 선택합니다.
        M = cv2.moments(max_contour)                          # 외곽선의 모멘트를 계산합니다.

        if M["m00"] != 0: # 외곽선의 면적이 0이 아닌 경우
            cx = int(M["m10"] / M["m00"]) # 외곽선의 중심을 계산합니다.
            cx_pit = int(cx - pi*(cx-cx_pit)) #p제어를 이용해서 중심 따라가기
            cv2.drawContours(roi, [max_contour], -1, (255, 0, 0), 2) # 외곽선과 중앙선을 그립니다.
            cv2.line(roi, (cx, 0), (cx, roi_height), (255, 0, 0), 2) 
            cv2.line(roi, (cx_pit, 0), (cx_pit, roi_height), (255, 0, 0), 2)

    else: # 외곽선의 면적이 0인 경우, 정지 상태로 간주합니다.
        cx = int(width // 2)
        cx_pit = int(cx - pi*(cx-cx_pit))
        cv2.line(roi, (cx, 0), (cx, roi_height), (255, 0, 0), 2) 
        cv2.line(roi, (cx_pit, 0), (cx_pit, roi_height), (255, 0, 0), 2)
        return "STOP", cx_pit, False
    
    # 중심선 위치에 따라 이동 방향을 결정합니다.
    if cx_pit < center_line: 
        return "LEFT", cx_pit, True
    
    elif cx_pit > center_line: 
        return "RIGHT", cx_pit, True
    
    else: 
        return "GO", cx_pit, True

def traffic_frame(frame):
    height, width, _ = frame.shape
    roi_height = int(height / 5)
    roi_top = height - roi_height
    signal_roi = frame[:roi_top, :]
    
    # 색상 변화 및 이진화 
    hsv = cv2.cvtColor(signal_roi, cv2.COLOR_BGR2HSV)

    # 빨간색 신호등을 감지
    lower_red = np.array([0, 0, 200])
    upper_red = np.array([150, 150, 255])
    red_mask = cv2.inRange(signal_roi, lower_red, upper_red)
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if red_contours:
        # 빨간색 신호등이 감지되면 여기에 대한 처리를 추가하세요
        pub_thread.update(0,0,0,0,0,0)
        # 물체의 중심 계산 및 결과 출력 
        M = cv2.moments(red_contours[0])        
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cv2.drawContours(signal_roi, [red_contours[0]], -1, (0, 0, 255), 2)
            return

def lidar_callback(scan):
    global detect_distance
    count=int(scan.scan_time/scan.time_increment)
    M_min_degree = -160
    M_max_degree = -180
    P_min_degree = 160
    P_max_degree = 180

    for i in range(count):
        degree=math.degrees(scan.angle_min+i*scan.angle_increment)

        if (M_min_degree <= degree <= M_max_degree) or (P_min_degree <= degree <= P_max_degree):
            distance = scan.ranges[i]
            if distance != 0.0:
                detect_distance = distance
    

def camera_thread():
    global pub_thread
    pub_thread = PublishThread(repeat)
    cap = cv2.VideoCapture(0) # 카메라 장치를 초기화합니다.
    while True:
        ret, frame = cap.read()   # 프레임을 읽어옵니다.
        _, width, _ = frame.shape # 프레임의 너비를 가져옵니다.
        
        if not ret: # 읽기가 실패한 경우 에러를 출력하고 반복문을 종료합니다.
            print("Camera error")
            break

        result, cx, _ = process_frame(frame) # process_frame 함수를 사용하여 프레임을 처리합니다.
        dx = abs((cx - width / 2) / (width / 2)) * 0.5  # 중심선과 객체의 거리를 계산하여 dx를 얻습니다.
        red_detected = traffic_frame(frame)

        if red_detected:
            stop_agv()
        else:
            action_order(result, x)
        
        cv2.imshow("Frame", frame) # 프레임을 화면에 표시합니다.

        if cv2.waitKey(1) & 0xFF == ord('q'): # 'q' 키를 눌러서 종료할 수 있습니다.
            break
    cap.release() # 카메라를 해제하고 창을 닫습니다.
    cv2.destroyAllWindows()

def stop_agv():
    print("Red Light!! Stop Car!!!")
    pub_thread.update(0, 0, 0, 0, 0, 0)

def action_order(result, dx):
    global detect_distance
    if 0<= detect_distance < 0.28:
        print("DETECT")
        pub_thread.update(0, 0, 0, 0, 0, 0)
        time.sleep(0.5)
        pub_thread.update(0, 1, 0, 0, 0.08, 0)
        time.sleep(3)
        pub_thread.update(0, 0, 0, 0, 0, 0)
        time.sleep(0.5)
        pub_thread.update(1, 0, 0, 0, 0.08, 0)
        time.sleep(6.5)
        pub_thread.update(0, 0, 0, 0, 0, 0)
        time.sleep(0.5)
        pub_thread.update(0, -1, 0, 0, 0.08, 0)
        time.sleep(3)
        pub_thread.update(0, 0, 0, 0, 0, 0)
        time.sleep(0.5)
    elif result and detect_distance >= 0.28: # 결과가 있는 경우 동작을 수행합니다.
        print(result)
        if result == "LEFT": 
            pub_thread.update(1, 0, 0, 1, 0.08, dx)
        elif result == "RIGHT": 
            pub_thread.update(1, 0, 0, -1, 0.08, dx)
        elif result == "GO": 
            pub_thread.update(1, 0, 0, 0, 0.08, 0.00)
        elif result == "STOP": 
            pub_thread.update(0, 0, 0, 0, 0.00, 0.00)
    else:
        pub_thread.update(0,0,0,0,0.0,0)
    

class PublishThread(threading.Thread):
    def __init__(self, rate):
        # 부모 클래스의 초기화 함수를 호출합니다.
        super(PublishThread, self).__init__()
        # ROS 토픽에 메시지를 발행할 파이썬 ROS 퍼블리셔를 만듭니다.
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size=1)
        # 로봇의 상태를 저장할 변수들을 초기화합니다.
        self.x, self.y, self.z, self.th, self.speed, self.turn = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        # 스레드 조건 변수와 종료 상태를 관리하는 변수를 설정합니다.
        self.condition, self.done = threading.Condition(), False
        # 발행 주기에 따라 스레드 타임아웃을 설정합니다.
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None
        
        # 스레드를 시작합니다.
        self.start()

    def wait_for_subscribers(self): # 서브스크라이버가 연결될 때까지 대기합니다.
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn): # 로봇 상태를 업데이트하고 발행 스레드에 알립니다.
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        self.condition.notify()
        self.condition.release()

    def stop(self): # 스레드를 종료하고 로봇을 정지시킵니다.
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self): # 발행 스레드의 실행 메소드입니다.
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg

        while not self.done:
            self.condition.acquire()
            self.condition.wait(self.timeout)

            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame

            # 로봇 상태를 메시지에 복사합니다.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # 메시지를 발행합니다.
            self.publisher.publish(twist_msg)

        # 스레드 종료 시 정지 메시지를 발행합니다.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)

def getKey(settings, timeout):
    # 플랫폼에 따라 사용자 입력을 받는 방식이 다릅니다.
    if sys.platform == 'win32': # Windows에서는 getwch() 함수를 사용하여 문자를 직접 입력 받습니다.
        key = msvcrt.getwch()
    else: # Linux에서는 raw 모드로 전환하여 키 입력을 처리합니다.
        tty.setraw(sys.stdin.fileno())
        # 지정된 시간 동안 키 입력을 기다립니다.
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            # 사용자가 입력한 키를 읽습니다.
            key = sys.stdin.read(1)
        else:
            key = ''
        # 키 입력을 처리한 후에는 터미널 설정을 복원합니다.
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    # 플랫폼에 따라 터미널 설정을 저장합니다.
    if sys.platform == 'win32':
        return None
    # Linux에서는 현재 터미널 설정을 저장하고 반환합니다.
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    # 플랫폼에 따라 터미널 설정을 복원합니다.
    if sys.platform == 'win32':
        return
    # Linux에서는 이전에 저장한 터미널 설정을 복원합니다.
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    # 현재 로봇의 속도와 회전 값을 문자열로 포맷팅하여 반환합니다.
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = saveTerminalSettings() # 터미널 설정을 저장합니다.
    rospy.init_node('line_agv')       # ROS 노드를 초기화합니다.
    rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size=1)
    # ROS 파라미터로부터 속도, 회전, 속도 제한, 회전 제한, 반복 주기 등을 가져옵니다.
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 0.5)
    speed_limit = rospy.get_param("~speed_limit", 1.0)
    turn_limit = rospy.get_param("~turn_limit", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.52)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    
    # 메시지가 타임스탬프와 함께 발행되어야 하는지 확인하고 메시지 타입을 선택합니다.
    if stamped:
        TwistMsg = TwistStamped

    # 발행 스레드를 생성합니다.
    pub_thread = PublishThread(repeat)

    # 로봇의 초기 상태를 설정합니다.
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        # 구독자가 연결될 때까지 대기합니다.
        pub_thread.wait_for_subscribers()
        # 초기 로봇 상태를 발행 스레드에 업데이트합니다.
        pub_thread.update(x, y, z, th, speed, turn)
        # 사용자에게 지침을 출력합니다.
        print("Please Press 'T' to Continue... ")
        print("Init Linear Speed, Angular Speed")
        print(vels(speed,turn))
        print("To Stop the Process Please Press 'v' or 'V'.")

        while(1):
            # 사용자 입력을 받습니다.
            key = getKey(settings, key_timeout)
            
            # 'T'를 입력받으면 무한 루프를 실행합니다.
            if key == 't' or key == 'T':
                while(1):
                    # 메인 스레드에서 카메라 스레드 실행
                    camera_thread = threading.Thread(target=camera_thread)
                    camera_thread.start()
                    
                    # 카메라 스레드가 종료될 때까지 대기
                    camera_thread.join()
                    
                    # 사용자 입력을 받습니다.
                    quit_key = getKey(settings, key_timeout)
                    
                    # 0.25초 대기 후 사용자 입력을 출력합니다.
                    time.sleep(0.25)
                    print(key)
                    
                    # 'v'를 입력받으면 무한 루프를 종료합니다.
                    if quit_key == 'v' or quit_key == "V":
                        break

            else:
                # 키 입력이 없거나 로봇이 이미 정지된 경우 cmd_vel을 업데이트하지 않습니다.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x, y, z, th = 0, 0, 0, 0
                if (key == '\x03'):
                    break

            # 로봇 상태를 업데이트합니다.
            pub_thread.update(x, y, z, th, speed, turn)
    except Exception as e:
        # 예외가 발생하면 오류 메시지를 출력합니다.
        print(e)
    finally:
        # 스레드를 중지하고 터미널 설정을 복원합니다.
        pub_thread.stop()
        restoreTerminalSettings(settings)