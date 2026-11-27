#!/usr/bin/env python3
# 中文数据面板：调试图 + detection + track + target + 参数
import sys, threading, time
import rospy, cv_bridge, numpy as np, cv2
from sensor_msgs.msg import Image
from ft_msgs.msg import DetectionArray, Track, TargetSelect
from PyQt5.QtWidgets import (QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
                             QGroupBox, QGridLayout, QPushButton)
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtCore import QTimer, Qt

lock = threading.Lock()
state = {'img': None, 'det': None, 'trk': None, 'tgt': None, 'stamp': None}
bridge = cv_bridge.CvBridge()

def img_cb(msg):
    try: im = bridge.imgmsg_to_cv2(msg, 'bgr8')
    except Exception: return
    with lock: state['img'] = im

def det_cb(msg):
    with lock: state['det'] = msg

def trk_cb(msg):
    with lock: state['trk'] = msg

def tgt_cb(msg):
    with lock: state['tgt'] = msg

class Panel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('视觉调试面板')
        self.setStyleSheet('QWidget{background:#f6f1e7;color:#2b2b2b;font-size:15px;}'
                           'QGroupBox{border:2px solid #1f8a7a;border-radius:10px;margin-top:10px;padding:10px;}'
                           'QGroupBox::title{subcontrol-origin:margin;left:12px;padding:0 6px;color:#1f8a7a;font-weight:bold;}'
                           'QLabel#big{font-size:22px;font-weight:bold;}'
                           'QLabel#warn{color:#d9674b;font-weight:bold;}')
        self.img_label = QLabel('等待 /ft/debug/image ...')
        self.img_label.setMinimumSize(820, 615)
        self.img_label.setAlignment(Qt.AlignCenter)
        self.img_label.setStyleSheet('background:#111;color:#aaa;border:2px solid #555;')
        self.det_box = QGroupBox('检测 /ft/detection')
        self.det_text = QLabel('无数据')
        self.det_text.setFont(QFont('Noto Sans CJK SC', 14))
        self.det_text.setAlignment(Qt.AlignTop)
        self.det_box.setLayout(QVBoxLayout()); self.det_box.layout().addWidget(self.det_text)
        self.trk_box = QGroupBox('跟踪 /ft/track')
        self.trk_text = QLabel('无数据'); self.trk_text.setAlignment(Qt.AlignTop)
        self.trk_box.setLayout(QVBoxLayout()); self.trk_box.layout().addWidget(self.trk_text)
        self.tgt_box = QGroupBox('目标 /ft/target')
        self.tgt_text = QLabel('无数据'); self.tgt_text.setAlignment(Qt.AlignTop)
        self.tgt_box.setLayout(QVBoxLayout()); self.tgt_box.layout().addWidget(self.tgt_text)
        self.par_box = QGroupBox('参数（实时）')
        self.par_text = QLabel('-'); self.par_text.setAlignment(Qt.AlignTop)
        self.par_box.setLayout(QVBoxLayout()); self.par_box.layout().addWidget(self.par_text)
        right = QVBoxLayout(); right.addWidget(self.det_box); right.addWidget(self.trk_box)
        right.addWidget(self.tgt_box); right.addWidget(self.par_box)
        left = QVBoxLayout(); left.addWidget(self.img_label)
        root = QHBoxLayout(); root.addLayout(left, 3); root.addLayout(right, 2)
        self.setLayout(root)
        self.timer = QTimer(); self.timer.timeout.connect(self.refresh); self.timer.start(150)

    def refresh(self):
        with lock:
            img = None if state['img'] is None else state['img'].copy()
            det, trk, tgt = state['det'], state['trk'], state['tgt']
        if img is not None:
            h, w = img.shape[:2]
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            q = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
            self.img_label.setPixmap(QPixmap.fromImage(q).scaled(820, 615, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        if det is not None:
            lines = [f'数量：{len(det.detections)}']
            for d in det.detections[:4]:
                lines.append(f'ID {d.id}  {"红" if d.is_red else "蓝"}  {"大" if d.is_large_armor else "小"}')
                lines.append(f'  x={d.pose.position.x:+.2f}  y={d.pose.position.y:+.2f}  z={d.pose.position.z:.2f} m')
            self.det_text.setText('\n'.join(lines))
        if trk is not None:
            self.trk_text.setText(
                f'跟踪：{"是" if trk.tracking else "否（仅预测）"}\n'
                f'位置 z：{trk.position.z:.2f} m\n'
                f'速度 z：{trk.velocity.z:+.2f} m/s\n'
                f'预测 z：{trk.position_predict.z:.2f} m')
        if tgt is not None:
            self.tgt_text.setText(f'有效：{"是" if tgt.valid else "否"}\n目标 ID：{tgt.target_id}\n'
                                  f'位置 z：{tgt.position.z:.2f} m\n预测 z：{tgt.position_predict.z:.2f} m')
        try:
            self.par_text.setText(
                f'max_body_v：{rospy.get_param("/ft_detector/max_body_v", "-")}\n'
                f'min_area：{rospy.get_param("/ft_detector/min_area", "-")}\n'
                f'gate：{rospy.get_param("/ft_tracker/gate", "-")}\n'
                f'T：{rospy.get_param("/ft_tracker/T", "-")}')
        except Exception: pass

if __name__ == '__main__':
    rospy.init_node('vision_panel', anonymous=True)
    rospy.Subscriber('/ft/debug/image', Image, img_cb, queue_size=1)
    rospy.Subscriber('/ft/detection', DetectionArray, det_cb, queue_size=10)
    rospy.Subscriber('/ft/track', Track, trk_cb, queue_size=10)
    rospy.Subscriber('/ft/target', TargetSelect, tgt_cb, queue_size=10)
    app = QApplication(sys.argv)
    w = Panel(); w.resize(1450, 900); w.show()
    app.exec_()
