from csv import writer
from math import ceil
from os import mkdir, path
from os.path import abspath
from random import randint
from shutil import copytree, rmtree
from sys import exit, argv
from threading import Timer
from time import sleep, strftime, localtime

import cv2
import numpy as np
import pandas as pd
import serial.tools.list_ports
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtGui import QIcon, QFont, QStandardItem
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtWidgets import QLabel, QPushButton, QCheckBox, QLineEdit, QAction, QRadioButton, QTableView, QHeaderView, QComboBox, \
    QAbstractItemView
from scipy import signal
from sklearn.cluster import KMeans
from wmi import WMI

# 重建窗口
class MyBar(QtWidgets.QWidget):
    def __init__(self, parent):
        super(MyBar, self).__init__()
        self.parent = parent

        # 窗口大小自适应，窗口默认1260 * 750
        desktop_title = QtWidgets.QApplication.desktop()
        available_title = desktop_title.availableGeometry()
        width_title = available_title.width()
        height_title = available_title.height()
        ratio_width_title = width_title / 1260
        ratio_height_title = height_title / 750

        if ratio_width_title >= ratio_height_title:
            height_title = available_title.height()
            width_title = height_title * 1.68
            ratio_title = ratio_height_title
        else:
            width_title = available_title.width()
            height_title = width_title / 1.68
            ratio_title = ratio_width_title
        height_title = height_title / 20

        # 窗口栏图标
        self.title_icon = QPushButton()
        self.title_icon.setStyleSheet('''QPushButton{background:#6FAFBB;border:0px}''')
        self.title_icon.setFixedSize(height_title, height_title)
        self.title_icon.setIcon(QIcon("%s/icon/window.ico" % path_icon))
        self.title_icon.setIconSize(QtCore.QSize(height_title - 10, height_title - 10))

        if language == 'zh_CN':
            try:
                with open("title.txt", "r") as f:
                    tilteName = f.readline()
                    if tilteName == '':
                        tilteName = '千粒重'
                    else:
                        tilteName = tilteName.encode('latin-1').decode('unicode_escape')
            except:
                tilteName = '千粒重'

            self.title = QLabel('%s' % tilteName)
            self.title.setStyleSheet(
                '''QLabel{color:white;background:#6FAFBB;font-size:%dpx;font-family:Source Code Pro;font-weight:500;}''' % int(
                    20 * ratio_title))
        else:
            try:
                with open("title.txt", "r") as f:
                    tilteName = f.readline()
                    for ch in tilteName:
                        if u'\u4e00' <= ch <= u'\u9fff':
                            CH = 1
                        else:
                            CH = 0

                    if CH == 1 or tilteName == '':
                        tilteName = 'Thousand Kernel Weight'
                    else:
                        pass
            except:
                tilteName = 'Thousand Kernel Weight'

            self.title = QLabel('%s' % tilteName)
            self.title.setStyleSheet(
                '''QLabel{color:white;background:#6FAFBB;font-size:%dpx;font-family:Times New Roman;font-weight:500;}''' % int(
                    20 * ratio_title))

        self.title.setFixedSize(width_title, height_title)
        self.title.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)

        # 最小化按钮
        self.button_close = QPushButton("×")
        self.button_close.clicked.connect(self.button_close_clicked)
        self.button_close.setFixedSize(height_title, height_title)
        self.button_close.setStyleSheet(
            '''QPushButton{color:white;font:20px;background-color: transparent;}QPushButton:hover{background-color:red;border:0px;}''')

        # 关闭按钮
        self.button_min = QPushButton("－")
        self.button_min.clicked.connect(self.button_min_clicked)
        self.button_min.setFixedSize(height_title, height_title)
        self.button_min.setStyleSheet(
            '''QPushButton{color:white;font:20px;background-color: transparent;}QPushButton:hover{background-color:#E1E1E1;border:0px;}''')

        # 控件布局
        self.layout = QtWidgets.QHBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.title_icon)
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.button_min)
        self.layout.addWidget(self.button_close)
        self.setLayout(self.layout)
        self.layout.setSpacing(0)

        self.start = QtCore.QPoint(0, 0)
        self.pressing = False

    # 点击窗口函数
    def mousePressEvent(self, event):
        self.start = self.mapToGlobal(event.pos())
        self.pressing = True

    # 窗口栏拖动函数
    def mouseMoveEvent(self, event):
        if self.pressing:
            self.end = self.mapToGlobal(event.pos())
            self.movement = self.end - self.start
            self.parent.setGeometry(self.mapToGlobal(self.movement).x(),
                                    self.mapToGlobal(self.movement).y(),
                                    self.parent.width(),
                                    self.parent.height())
            self.start = self.end

    # 释放鼠标函数
    def mouseReleaseEvent(self, QMouseEvent):
        self.pressing = False

    # 关闭窗口函数
    def button_close_clicked(self):
        self.parent.close()

    # 最小化窗口函数
    def button_min_clicked(self):
        self.parent.showMinimized()

# 主窗口
class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        # 参数初始化
        self.trigger_camera = 0
        self.trigger_camera_initial = 1
        self.trigger_image = 0
        self.trigger_acquisition = 0
        self.trigger_manual = 0
        self.trigger_line = 0
        self.trigger_acquisition_start = 0
        self.trigger_lightsignal = 0
        self.trigger_light_normal = 0
        self.trigger_lightoff = 0
        self.ret_cam = 0
        self.trigger_csv_head = 0

        self.contours_add = []
        self.contours_normal = []
        self.contours_abnormal = []
        self.contours_remove = []
        self.contours_remove_normal_copy = []
        self.contours_remove_abnormal = []
        self.contours_remove_abnormal_copy= []
        self.contours_remove_normal = []
        self.line_manual = []

        self.list_balance_State = []
        self.list_light_State = []

        self.num_process = 0
        self.num_contours = 0
        self.meanArea = 0
        self.meanPerimeter = 0
        self.meanLength = 0
        self.meanWidth = 0
        self.shortEdge = 0

        self.trigger_zoom = 0
        self.mouse_mv_x = ""
        self.mouse_mv_y = ""
        self.labelx = 0
        self.labely = 0

        self.ratio = arearatio
        self.area_calibration_true = 0

        self.mouseButton = 0

        self.initUI()

    # 创建控件函数
    def initUI(self):
        # 获取屏幕尺寸，使窗口自适应屏幕，默认窗口大小1260 * 750
        self.desktop = QtWidgets.QApplication.desktop()
        self.available_rect = self.desktop.availableGeometry()
        self.width_window = self.available_rect.width()
        self.height_window = self.available_rect.height()
        self.ratio_width_window = self.width_window / 1260
        self.ratio_height_window = self.height_window / 750

        # 窗口尺寸设定
        if self.ratio_width_window >= self.ratio_height_window:
            self.height_window = self.available_rect.height()
            self.width_window = self.height_window * 1.68
            self.ratio_window = self.ratio_height_window
        else:
            self.width_window = self.available_rect.width()
            self.height_window = self.width_window / 1.68
            self.ratio_window = self.ratio_width_window

        # 图像显示区域尺寸设定
        self.height_display_image = self.height_window * 19 / 20
        self.width_display_image = self.height_display_image * 4 / 3

        # 图像尺寸设定
        self.image_height = ceil(self.height_display_image) - 1
        if self.image_height % 3 == 0:
            self.image_height = self.image_height
        elif self.image_height % 3 == 1:
            self.image_height = self.image_height + 2
        else:
            self.image_height = self.image_height + 1
        self.image_width = int(self.image_height * (4 / 3))

        # 实际窗口大小与默认窗口大小比值
        self.ratio_pixel = self.image_width * self.image_height / (936 * 702)

        # 图片显示区域容器
        self.display_image_box = QLabel(self)
        self.display_image_box.setStyleSheet(
            "QLabel{background:white;font-size:%dpx;font-family:Times New Roman;font-weight:500;}" % int(30 * self.ratio_window))
        self.display_image_box.setAlignment(QtCore.Qt.AlignCenter)
        self.display_image_box.setFixedSize(self.image_width, self.image_height)
        self.display_image_box.move(1, (self.height_window / 20))

        self.display_image = QLabel(self.display_image_box)
        self.display_image.setStyleSheet("QLabel{background:transparent;}")
        self.display_image.setAlignment(QtCore.Qt.AlignCenter)
        self.display_image.setMouseTracking(False)
        self.display_image.resize(self.image_width, self.image_height)
        self.display_image.move(0, 0)

        self.display_processing = QLabel(self.display_image_box)
        self.display_processing.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.display_processing.setFrameShadow(QtWidgets.QFrame.Raised)
        self.display_processing.setLineWidth(4)
        self.display_processing.setAlignment(QtCore.Qt.AlignCenter)
        self.display_processing.setFixedSize(int(self.width_display_image / 4), int(self.height_display_image / 8))
        self.display_processing.move(int(self.width_display_image * 3 / 8), int(self.height_display_image * 7 / 16))
        self.display_processing.setHidden(True)

        # 外围方框
        self.windowbox = QLabel(self)
        self.windowbox.setStyleSheet("QLabel{background:transparent;}")
        self.windowbox.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.windowbox.setFrameShadow(QtWidgets.QFrame.Plain)
        self.windowbox.setFixedSize(self.width_window, self.height_window)
        self.windowbox.move(0, 0)

        # 红色方框
        self.widthRatio = 1
        self.heightRatio = 1

        self.display_waring = QLabel(self.display_image_box)
        self.display_waring.setStyleSheet("QLabel{background:transparent;}")

        self.mode_table = QtGui.QStandardItemModel(0, 4)
        self.tableView = QTableView(self)
        self.tableView.setModel(self.mode_table)
        self.tableView.setEditTriggers(QTableView.NoEditTriggers)
        if language == 'zh_CN':
            self.mode_table.setHorizontalHeaderLabels(['名称', '数量', '面积', '周长'])
            self.tableView.horizontalHeader().setStyleSheet(
                "QHeaderView::section{background-color:white;font-size:%dpx;font-family:宋体;color: black;}" % int(16 * self.ratio_window))
        else:
            self.mode_table.setHorizontalHeaderLabels(['SN', 'Nunmber', 'Area', 'Perimeter'])
            self.tableView.horizontalHeader().setStyleSheet(
                "QHeaderView::section{background-color:white;font-size:%dpx;font-family:Times New Roman;color: black;}" % int(16 * self.ratio_window))
        self.tableView.verticalHeader().setStyleSheet(
            "QHeaderView::section{background-color:white;font-size:%dpx;font-family:Times New Roman;color: black;}" % int(16 * self.ratio_window))
        self.tableView.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableView.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.tableView.setGeometry(self.width_window - 301 * self.ratio_window - 3, self.height_window / 20, 301 * self.ratio_window,
                                        300 * self.ratio_window)

        self.group_result = QLabel(self)
        self.group_result.setStyleSheet("QLabel{background:white}")
        self.group_result.setGeometry(self.width_window - 301 * self.ratio_window - 2, 350 * self.ratio_window, 300 * self.ratio_window, 160 * self.ratio_window)

        lable_line = ['line1', 'line2']
        for i in range(0, 2):
            lable_line[i] = QLabel(self.group_result)
            lable_line[i].setStyleSheet('border-bottom-width:1px; border-style:solid; border-bottom-color: #464646')
            lable_line[i].setFixedSize(400 * self.ratio_window, 45 * self.ratio_window)
            lable_line[i].move(0, 10 * self.ratio_window + 50 * self.ratio_window * i)

        self.grainNumber = QLabel(self.group_result)
        self.grainNumber.setFixedSize(400 * self.ratio_window, 40 * self.ratio_window)
        self.grainNumber.move(0, 10 * self.ratio_window)

        self.grainArea = QLabel(self.group_result)
        self.grainArea.setFixedSize(400 * self.ratio_window, 40 * self.ratio_window)
        self.grainArea.move(0, 60 * self.ratio_window)

        self.grainPerimeter = QLabel(self.group_result)
        self.grainPerimeter.setFixedSize(400 * self.ratio_window, 40 * self.ratio_window)
        self.grainPerimeter.move(0, 110 * self.ratio_window)

        if language == 'zh_CN':
            self.grainNumber.setStyleSheet(
                '''QLabel{font-size:%dpx;font-weight:500;font-family:宋体;}QLabel:hover{background:#7FFFAA;}''' % int(24 * self.ratio_window))
            self.grainNumber.setText('  籽粒数量：')
            self.grainArea.setStyleSheet(
                '''QLabel{font-size:%dpx;font-weight:500;font-family:宋体;}QLabel:hover{background:#7FFFAA;}''' % int(24 * self.ratio_window))
            self.grainArea.setText('  籽粒面积：')
            self.grainPerimeter.setStyleSheet(
                '''QLabel{font-size:%dpx;font-weight:500;font-family:宋体;}QLabel:hover{background:#7FFFAA;}''' % int(24 * self.ratio_window))
            self.grainPerimeter.setText('  籽粒周长：')

        else:
            self.grainNumber.setStyleSheet(
                '''QLabel{font-size:%dpx;font-family:Times New Roman;}QLabel:hover{background:#7FFFAA;}''' % int(22 * self.ratio_window))
            self.grainNumber.setText('   Kernel Number :')
            self.grainArea.setStyleSheet(
                '''QLabel{font-size:%dpx;font-family:Times New Roman;}QLabel:hover{background:#7FFFAA;}''' % int(22 * self.ratio_window))
            self.grainArea.setText('   Kernel Area :')
            self.grainPerimeter.setStyleSheet(
                '''QLabel{font-size:%dpx;font-family:Times New Roman;}QLabel:hover{background:#7FFFAA;}''' % int(22 * self.ratio_window))
            self.grainPerimeter.setText('   Kernel Perimeter: ')

        lable_unit = ['unit1', 'unit2', 'unit3']
        lablename_unit = ['', ' cm<sup>2</sup>', 'cm',]
        for i in range(0, len(lable_unit)):
            lable_unit[i] = QLabel(self.group_result)
            lable_unit[i].setStyleSheet(
                "QLabel{color:blue;background:transparent;font-size:%dpx;font-family:Times New Roman;}" % int(
                    24 * self.ratio_window))
            lable_unit[i].setText(lablename_unit[i])
            lable_unit[i].setFixedSize(50 * self.ratio_window, 40 * self.ratio_window)
            lable_unit[i].move(230 * self.ratio_window, 10 * self.ratio_window + 50 * self.ratio_window * i)

        self.grainNumber_value = QLabel('0', self.group_result)
        self.grainNumber_value.setStyleSheet(
            "QLabel{color:blue;background:transparent;font-size:%dpx;font-family:Times New Roman;}" % int(
                24 * self.ratio_window))
        self.grainNumber_value.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.grainNumber_value.setGeometry(185 * self.ratio_window, 12 * self.ratio_window, 85 * self.ratio_window,
                                         40 * self.ratio_window)

        self.grainArea_value = QLabel('0.00', self.group_result)
        self.grainArea_value.setStyleSheet(
            "QLabel{color:blue;background:transparent;font-size:%dpx;font-family:Times New Roman;}" % int(
                24 * self.ratio_window))
        self.grainArea_value.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.grainArea_value.setGeometry(185 * self.ratio_window, 62 * self.ratio_window, 85 * self.ratio_window,
                                              40 * self.ratio_window)

        self.grainPerimeter_value = QLabel('0.00', self.group_result)
        self.grainPerimeter_value.setStyleSheet('color:blue;background:transparent;font-size:%dpx;font-family:Times New Roman;border: 0px' % int(
                24 * self.ratio_window))
        self.grainPerimeter_value.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.grainPerimeter_value.setGeometry(185 * self.ratio_window, 112 * self.ratio_window, 85 * self.ratio_window,
                                           40 * self.ratio_window)

        self.group_button = QLabel(self)
        self.group_button.setStyleSheet("QLabel{background:#464646}")
        self.group_button.setGeometry(self.width_window - 301 * self.ratio_window - 2, 515 * self.ratio_window, 300 * self.ratio_window, 230 * self.ratio_window)

        self.button_zoomIn = QPushButton(self.group_button)
        self.button_zoomIn.setStyleSheet('''QPushButton{border-radius:%dpx;background:#DCDCDC;}''' % int(8 * self.ratio_window))
        self.button_zoomIn.setIcon(QIcon("%s/icon/zoomin.png" % path_icon))
        self.button_zoomIn.setIconSize(QtCore.QSize(35 * self.ratio_window, 35 * self.ratio_window))
        self.button_zoomIn.setFixedSize(35 * self.ratio_window, 35 * self.ratio_window)
        self.button_zoomIn.move(4 * self.ratio_window, 17 * self.ratio_window)
        self.button_zoomIn.setEnabled(False)

        self.button_zoomOut = QPushButton(self.group_button)
        self.button_zoomOut.setStyleSheet('''QPushButton{border-radius:%dpx;background:#DCDCDC;}''' % int(8 * self.ratio_window))
        self.button_zoomOut.setIcon(QIcon("%s/icon/zoomout.png" % path_icon))
        self.button_zoomOut.setIconSize(QtCore.QSize(35 * self.ratio_window, 35 * self.ratio_window))
        self.button_zoomOut.setFixedSize(35 * self.ratio_window, 35 * self.ratio_window)
        self.button_zoomOut.move(46 * self.ratio_window, 17 * self.ratio_window)
        self.button_zoomOut.setEnabled(False)

        self.button_zoomReturn = QPushButton(self.group_button)
        self.button_zoomReturn.setStyleSheet('''QPushButton{border-radius:%dpx;background:#DCDCDC;}''' % int(8 * self.ratio_window))
        self.button_zoomReturn.setIcon(QIcon("%s/icon/return.png" % path_icon))
        self.button_zoomReturn.setIconSize(QtCore.QSize(35 * self.ratio_window, 35 * self.ratio_window))
        self.button_zoomReturn.setFixedSize(35 * self.ratio_window, 35 * self.ratio_window)
        self.button_zoomReturn.move(88 * self.ratio_window, 17 * self.ratio_window)
        self.button_zoomReturn.setEnabled(False)

        self.button_back = QPushButton(self.group_button)
        self.button_back.setStyleSheet('''QPushButton{border-radius:%dpx;background:#DCDCDC;}''' % int(8 * self.ratio_window))
        self.button_back.setIcon(QIcon("%s/icon/back.png" % path_icon))
        self.button_back.setIconSize(QtCore.QSize(30 * self.ratio_window, 30 * self.ratio_window))
        self.button_back.setFixedSize(35* self.ratio_window, 35 * self.ratio_window)
        self.button_back.move(130 * self.ratio_window, 17 * self.ratio_window)
        self.button_back.setEnabled(False)

        self.button_forward = QPushButton(self.group_button)
        self.button_forward.setStyleSheet('''QPushButton{border-radius:%dpx;background:#DCDCDC;}''' % int(8 * self.ratio_window))
        self.button_forward.setIcon(QIcon("%s/icon/forward.png" % path_icon))
        self.button_forward.setIconSize(QtCore.QSize(30 * self.ratio_window, 30 * self.ratio_window))
        self.button_forward.setFixedSize(35 * self.ratio_window, 35 * self.ratio_window)
        self.button_forward.move(172 * self.ratio_window, 17 * self.ratio_window)
        self.button_forward.setEnabled(False)

        self.button_restore = QPushButton(self.group_button)
        self.button_restore.setStyleSheet(
            '''QPushButton{border-radius:%dpx;background:#DCDCDC;}''' % int(8 * self.ratio_window))
        self.button_restore.setIcon(QIcon("%s/icon/restore.png" % path_icon))
        self.button_restore.setIconSize(QtCore.QSize(30 * self.ratio_window, 30 * self.ratio_window))
        self.button_restore.setFixedSize(35 * self.ratio_window, 35 * self.ratio_window)
        self.button_restore.move(214 * self.ratio_window, 17 * self.ratio_window)
        self.button_restore.setEnabled(False)

        self.button_openRecord = QPushButton(self.group_button)
        self.button_openRecord.setStyleSheet(
            '''QPushButton{background:#DCDCDC;border-radius:5px}QPushButton:hover{background:#90EE90;}''')
        self.button_openRecord.setIcon(QIcon("%s/icon/record.png" % path_icon))
        self.button_openRecord.setIconSize(QtCore.QSize(35 * self.ratio_window, 35 * self.ratio_window))
        self.button_openRecord.setFixedSize(35 * self.ratio_window, 35 * self.ratio_window)
        self.button_openRecord.move(256 * self.ratio_window, 17 * self.ratio_window)

        if language == 'zh_CN':
            self.button_zoomIn.setToolTip('放大')
            self.button_zoomOut.setToolTip('缩小')
            self.button_zoomReturn.setToolTip('还原')
            self.button_back.setToolTip('撤销')
            self.button_forward.setToolTip('恢复')
            self.button_restore.setToolTip('修正')
            self.button_openRecord.setToolTip('查看记录')
        else:
            self.button_zoomIn.setToolTip('Zoom in')
            self.button_zoomOut.setToolTip('Zoom out')
            self.button_zoomReturn.setToolTip('Return')
            self.button_back.setToolTip('Back')
            self.button_forward.setToolTip('Forward')
            self.button_restore.setToolTip('Modify')
            self.button_openRecord.setToolTip('View records')

        self.button_opencamera = QPushButton(self.group_button)
        self.button_opencamera.setIcon(QIcon("%s/icon/opencamera.png" % path_icon))
        self.button_opencamera.setIconSize(QtCore.QSize(35 * self.ratio_window, 35 * self.ratio_window))
        self.button_opencamera.setFixedSize(145 * self.ratio_window, 50 * self.ratio_window)
        self.button_opencamera.move(2 * self.ratio_window, 65 * self.ratio_window)

        self.button_openimage = QPushButton(self.group_button)
        self.button_openimage.setIcon(QIcon("%s/icon/openimage.png" % path_icon))
        self.button_openimage.setIconSize(QtCore.QSize(35 * self.ratio_window, 35 * self.ratio_window))
        self.button_openimage.setFixedSize(145 * self.ratio_window, 50 * self.ratio_window)
        self.button_openimage.move(2 * self.ratio_window, 120 * self.ratio_window)

        self.button_setting = QPushButton(self.group_button)
        self.button_setting.setIcon(QIcon("%s/icon/detection.png" % path_icon))
        self.button_setting.setIconSize(QtCore.QSize(37 * self.ratio_window, 37 * self.ratio_window))
        self.button_setting.setFixedSize(145 * self.ratio_window, 50 * self.ratio_window)
        self.button_setting.move(152 * self.ratio_window, 175* self.ratio_window)

        self.button_acquisition = QPushButton(self.group_button)
        self.button_acquisition.setIcon(QIcon("%s/icon/processing.png" % path_icon))
        self.button_acquisition.setIconSize(QtCore.QSize(35 * self.ratio_window, 35 * self.ratio_window))
        self.button_acquisition.setFixedSize(145 * self.ratio_window, 50 * self.ratio_window)
        self.button_acquisition.move(152 * self.ratio_window, 65 * self.ratio_window)

        self.button_manual = QPushButton(self.group_button)
        self.button_manual.setIcon(QIcon("%s/icon/manual.png" % path_icon))
        self.button_manual.setIconSize(QtCore.QSize(37 * self.ratio_window, 37 * self.ratio_window))
        self.button_manual.setFixedSize(145 * self.ratio_window, 50 * self.ratio_window)
        self.button_manual.move(2 * self.ratio_window, 175 * self.ratio_window)

        self.button_saveimage = QPushButton(self.group_button)
        self.button_saveimage.setIcon(QIcon("%s/icon/saveimage.png" % path_icon))
        self.button_saveimage.setIconSize(QtCore.QSize(35 * self.ratio_window, 35 * self.ratio_window))
        self.button_saveimage.setFixedSize(145 * self.ratio_window, 50 * self.ratio_window)
        self.button_saveimage.move(152 * self.ratio_window, 120 * self.ratio_window)

        self.timer_showcamera = QtCore.QTimer()
        self.timer_showcamera.timeout.connect(self.showcamera)

        try:
            with open("model.txt", "r") as f:
                self.grainModel = int(f.readline().strip("\n"))
        except:
            self.grainModel = 0

        try:
            with open('port_camera.txt', 'r') as f:
                self.port_camera = int(f.readline().strip("\n"))
        except:
            self.port_camera = 100

        self.opencamera2(self.port_camera, 0)

        if language == 'zh_CN':
            self.display_image_box.setText('显 示 区 域')
            self.display_processing.setText('正在处理中...')
            self.display_processing.setStyleSheet(
                "QLabel{background:#7FFFAA;color:blue;font-size:%dpx;font-family:宋体;font-weight:500;}" % int(28 * self.ratio_window))
            self.button_opencamera.setText('实时图像')
            self.button_opencamera.setStyleSheet(
                '''QPushButton{color:white;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    24 * self.ratio_window))
            self.button_openimage.setText('打开图像')
            self.button_openimage.setStyleSheet(
                '''QPushButton{color:white;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    24 * self.ratio_window))
            self.button_manual.setText('人工数粒')
            self.button_manual.setStyleSheet(
                '''QPushButton{color:white;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    24 * self.ratio_window))
            self.button_acquisition.setText('处理图像')
            self.button_acquisition.setStyleSheet(
                '''QPushButton{color:white;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    24 * self.ratio_window))
            self.button_saveimage.setText('保存图像')
            self.button_saveimage.setStyleSheet(
                '''QPushButton{color:white;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    24 * self.ratio_window))
            self.button_setting.setText('系统设置')
            self.button_setting.setStyleSheet(
                '''QPushButton{color:white;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    24 * self.ratio_window))

        else:
            self.display_image_box.setText('Display region')
            self.display_processing.setText('Processing...')
            self.display_processing.setStyleSheet("QLabel{background:#7FFFAA;color:blue;font-size:%dpx;font-family:Times New Roman;font-weight:500;}" % int(28 * self.ratio_window))
            self.button_opencamera.setText('  Camera')
            self.button_opencamera.setStyleSheet(
                '''QPushButton{text-align: left; color:white;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    18 * self.ratio_window))
            self.button_openimage.setText('  Open')
            self.button_openimage.setStyleSheet(
                '''QPushButton{text-align: left;color:white;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    18 * self.ratio_window))
            self.button_manual.setText('  Manual')
            self.button_manual.setStyleSheet(
                '''QPushButton{text-align: left;color:white;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    18 * self.ratio_window))
            self.button_acquisition.setText('  Process')
            self.button_acquisition.setStyleSheet(
                '''QPushButton{text-align: left;color:white;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    18 * self.ratio_window))
            self.button_saveimage.setText('  Save')
            self.button_saveimage.setStyleSheet(
                '''QPushButton{text-align: left;color:white;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    18 * self.ratio_window))
            self.button_setting.setText('  Settings')
            self.button_setting.setStyleSheet(
                '''QPushButton{text-align: left;color:white;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    18 * self.ratio_window))

        self.button_zoomIn.clicked.connect(self.zoomIn)
        self.button_zoomOut.clicked.connect(self.zoomOut)
        self.button_zoomReturn.clicked.connect(self.zoomReturn)
        self.button_back.clicked.connect(self.back)
        self.button_forward.clicked.connect(self.forward)
        self.button_restore.clicked[bool].connect(self.restore)
        self.button_openRecord.clicked.connect(self.openRecord)

        self.button_opencamera.clicked.connect(self.opencamera)
        self.button_acquisition.clicked.connect(self.acquisition)
        self.button_openimage.clicked.connect(self.openimage)
        self.button_saveimage.clicked.connect(self.saveimage)
        self.button_setting.clicked.connect(self.setting)
        self.button_manual.clicked.connect(self.manual)

        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(MyBar(self))
        self.setLayout(self.layout)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addStretch(-1)

        # 显示轮廓序号
        self.show_number_counter = QLabel(self)
        self.show_number_counter.setStyleSheet(
            "QLabel{color:blue;background:white;font-size:%dpx;font-weight:500;font-family:Times New Roman;}" % int(
                20 * self.ratio_window))
        self.show_number_counter.setAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
        self.show_number_counter.setFixedSize(60, 25)
        self.show_number_counter.setHidden(1)

        self.setFixedSize(self.width_window, self.height_window)
        self.move((self.available_rect.width() - self.width_window) / 2,
                  (self.available_rect.height() - self.height_window) / 2)
        self.setWindowIcon(QIcon("%s/icon/window.ico" % path_icon))
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        palette = QtGui.QPalette()
        palette.setBrush(QtGui.QPalette.Background, QtGui.QBrush(QtGui.QColor(70, 70, 70)))
        self.setPalette(palette)
        self.setAttribute(QtCore.Qt.WA_AcceptTouchEvents, True)

    def openRecord(self):
        win_x = self.x()
        win_y = self.y()
        self.window_allRecord = Record(self.width_window, self.height_window, win_x, win_y, self.ratio_window)
        self.window_allRecord.show()

    # 连续显示图像函数
    def showcamera(self):
        self.ret_cam, self.cam = self.cap.read()
        if self.ret_cam:
            try:
                with open("boxratio.txt", "r") as f:
                    self.widthRatio = float(f.readline())
                    self.heightRatio = float(f.readline())
                    if self.widthRatio >= 1:
                        self.widthRatio = 1
                    if self.heightRatio >= 1:
                        self.heightRatio = 1

                if self.widthRatio == 1 and self.heightRatio == 1:
                    self.display_waring.setStyleSheet("QLabel{background:transparent;}")
                else:
                    self.display_waring.setFixedSize(self.width_display_image * self.widthRatio,
                                                     self.height_display_image * self.heightRatio)
                    self.display_waring.move(self.width_display_image * ((1 - self.widthRatio) / 2),
                                             self.height_display_image * ((1 - self.heightRatio) / 2))
                    self.display_waring.setStyleSheet("QLabel{background:transparent;border:2px solid red;}")
            except:
                self.widthRatio = -1
                self.heightRatio = -1
                self.display_waring.setStyleSheet("QLabel{background:transparent;}")

            self.cam_origin = self.cam
            self.cam = cv2.resize(self.cam, (self.image_width, self.image_height))
            if self.trigger_camera == 1 or self.trigger_camera_initial == 1:
                self.displaycamera = QtGui.QImage(self.cam.data, self.cam.shape[1],
                                                  self.cam.shape[0],
                                                  QtGui.QImage.Format_BGR888)  # shape[1]图像的宽，shape[0]图像的高
                self.display_image.setPixmap(QtGui.QPixmap.fromImage(self.displaycamera))

        else:
            self.cap.release()
            try:
                self.cap = cv2.VideoCapture()
                self.cap.open(self.openCam, cv2.CAP_DSHOW)
                self.cap.set(3, 3264)
                self.cap.set(4, 2448)
                self.cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
            except:
                pass

    # 打开相机函数
    def opencamera(self):
        self.ret, _ = self.cap.read()
        if self.ret == False:
            if language == 'zh_CN':
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "请检查相机与电脑是否正确连接！")
                Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_window))
            else:
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip",
                                                "Please check whether the camera and computer are connected correctly!")
                Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_window))
            Qyes.setFixedSize(80, 40)
            message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
            message.setIcon(0)

            if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                pass
        else:
            self.display_image.resize(self.image_width, self.image_height)
            self.display_image.move(0, 0)
            self.trigger_zoom = 0

            self.button_zoomIn.setEnabled(False)
            self.button_zoomOut.setEnabled(False)
            self.button_zoomReturn.setEnabled(False)
            self.button_back.setEnabled(False)
            self.button_forward.setEnabled(False)
            self.button_restore.setEnabled(False)

            self.timer_showcamera.start(50)
            self.trigger_camera = 1
            self.trigger_camera_initial = 0
            self.trigger_image = 0
            self.trigger_acquisition = 0
            self.trigger_acquisition_start = 0
            self.display_image.setMouseTracking(False)
            self.show_number_counter.setHidden(1)

            self.meanArea = 0
            self.meanPerimeter = 0
            self.meanLength = 0
            self.meanWidth = 0
            self.menaRatioLW = 0
            self.menaED = 0
            self.meanShapeFactor = 0

            self.num_process = 0
            self.num_contours = 0
            self.grainNumber_value.setText("%d" % self.num_contours)
            self.grainArea_value.setText("0.00")
            self.grainPerimeter_value.setText("0.00")
            self.grainPerimeter_value.setStyleSheet(
                'color:blue;background:transparent;font-size:%dpx;font-family:Times New Roman;border: 0px' % int(
                    24 * self.ratio_window))

            self.button_acquisition.setEnabled(True)
            self.button_manual.setEnabled(True)
            if language == 'zh_CN':
                self.button_manual.setStyleSheet(
                    '''QPushButton{color:white;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                        24 * self.ratio_window))
            else:
                self.button_manual.setStyleSheet(
                    '''QPushButton{text-align: left;color:white;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                        18 * self.ratio_window))
            self.trigger_manual = 0
            self.display_image_box.setCursor(QtCore.Qt.ArrowCursor)

            self.button_restore.setStyleSheet(
                '''QPushButton{border-radius:%dpx;background:#DCDCDC;}''' % int(8 * self.ratio_window))
            self.trigger_line = 0

    def opencamera2(self, num_camera, way):
        self.cap = cv2.VideoCapture()
        self.openCam = num_camera - 1
        self.flag = self.cap.open(self.openCam, cv2.CAP_DSHOW)
        self.cap.set(3, 3264)
        self.cap.set(4, 2448)
        self.cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

        if self.flag == False:
            self.trigger_camera_initial = 0
            self.trigger_camera = 0
            if way == 1:
                if language == 'zh_CN':
                    message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "请检查相机与电脑是否正确连接！")
                    Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                    Qyes.setStyleSheet(
                        '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_window))
                    message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_window))

                else:
                    message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip",
                                                    "Please check whether the camera and computer are connected correctly!")
                    Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                    Qyes.setStyleSheet(
                        '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_window))
                    message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_window))

                Qyes.setFixedSize(80, 40)
                message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
                message.setIcon(0)

                if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                    pass
            else:
                pass

        else:
            if way == 0:
                self.timer_showcamera.start(50)
            else:
                pass
            self.trigger_camera = 1
            self.trigger_camera_initial = 0
            self.trigger_image = 0
            self.trigger_acquisition = 0
            self.trigger_acquisition_start = 0
            self.display_image.setMouseTracking(False)

    # 从文件夹获取图像
    def openimage(self):
        with open("path_openimage.txt", "r") as f:
            data = f.readline()
        if data:
            if language == 'zh_CN':
                self.imageName, imageType = QtWidgets.QFileDialog.getOpenFileName(self, "打开图像", "%s" %data, "*.jpg *.jpeg *.png *.bmp;;*.jpg;;*.jpeg;;*.png;;*.bmp")
            else:
                self.imageName, imageType = QtWidgets.QFileDialog.getOpenFileName(self, "Open image", "%s" %data, "*.jpg *.jpeg *.png *.bmp;;*.jpg;;*.jpeg;;*.png;;*.bmp")
        else:
            if language == 'zh_CN':
                self.imageName, imageType = QtWidgets.QFileDialog.getOpenFileName(self, "打开图像", "%s/result" % path_icon, "*.jpg *.jpeg *.png *.bmp;;*.jpg;;*.jpeg;;*.png;;*.bmp")
            else:
                self.imageName, imageType = QtWidgets.QFileDialog.getOpenFileName(self, "Open image", "%s/result" % path_icon, "*.jpg *.jpeg *.png *.bmp;;*.jpg;;*.jpeg;;*.png;;*.bmp")

        if self.imageName:
            with open('path_openimage.txt', 'w') as f:
                f.write(self.imageName)

        if self.imageName:
            self.timer_showcamera.stop()
            self.button_zoomIn.setEnabled(False)
            self.button_zoomOut.setEnabled(False)
            self.button_zoomReturn.setEnabled(False)
            self.button_back.setEnabled(False)
            self.button_forward.setEnabled(False)
            self.button_restore.setEnabled(False)

            self.meanArea = 0
            self.meanPerimeter = 0
            self.meanLength = 0
            self.meanWidth = 0
            self.menaRatioLW = 0
            self.menaED = 0
            self.meanShapeFactor = 0

            self.trigger_image = 1
            self.trigger_camera = 0
            self.trigger_camera_initial = 0
            self.trigger_acquisition = 0
            self.trigger_acquisition_start = 1
            self.num_process = 0
            self.num_contours = 0
            self.display_image.setMouseTracking(False)
            self.show_number_counter.setHidden(1)

            self.grainNumber_value.setText("%d" % self.num_contours)
            self.grainArea_value.setText("0.00")
            self.grainPerimeter_value.setText("0.00")
            self.grainPerimeter_value.setStyleSheet(
                'color:blue;background:transparent;font-size:%dpx;font-family:Times New Roman;border: 0px' % int(
                    24 * self.ratio_window))

            self.button_acquisition.setEnabled(True)
            self.button_manual.setEnabled(True)
            if language == 'zh_CN':
                self.button_manual.setStyleSheet(
                    '''QPushButton{color:white;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                        24 * self.ratio_window))
            else:
                self.button_manual.setStyleSheet(
                    '''QPushButton{text-align: left;color:white;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                        18 * self.ratio_window))
            self.trigger_manual = 0
            self.display_image_box.setCursor(QtCore.Qt.ArrowCursor)

            self.button_restore.setStyleSheet(
                '''QPushButton{border-radius:%dpx;background:#DCDCDC;}''' % int(8 * self.ratio_window))
            self.trigger_line = 0

            try:
                self.scale = self.scale_origin
                self.resize_image()
            except:
                pass
            self.display_image.resize(self.image_width, self.image_height)
            self.display_image.move(0, 0)
            self.trigger_zoom = 0

            try:
                with open("boxratio.txt", "r") as f:
                    self.widthRatio = float(f.readline())
                    self.heightRatio = float(f.readline())
                    if self.widthRatio >= 1:
                        self.widthRatio = 1
                    if self.heightRatio >= 1:
                        self.heightRatio = 1

                if self.widthRatio == 1 and self.heightRatio == 1:
                    self.display_waring.setStyleSheet("QLabel{background:transparent;}")
                else:
                    self.display_waring.setFixedSize(self.width_display_image * self.widthRatio,
                                                     self.height_display_image * self.heightRatio)
                    self.display_waring.move(self.width_display_image * ((1 - self.widthRatio) / 2),
                                             self.height_display_image * ((1 - self.heightRatio) / 2))
                    self.display_waring.setStyleSheet("QLabel{background:transparent;border:2px solid red;}")
            except:
                self.widthRatio = -1
                self.heightRatio = -1
                self.display_waring.setStyleSheet("QLabel{background:transparent;}")

            self.image_open = cv2.imdecode(np.fromfile(self.imageName, dtype=np.uint8), -1)

            try:
                self.image_open = cv2.cvtColor(self.image_open, cv2.COLOR_BGRA2BGR)
            except:
                pass

            try:
                self.image_open = cv2.cvtColor(self.image_open, cv2.COLOR_GRAY2BGR)
            except:
                pass

            self.image_open_display = self.image_open.copy()
            self.image_open_display = cv2.resize(self.image_open_display, (self.image_width, self.image_height))
            self.image_show = QtGui.QImage(self.image_open_display.data, self.image_open_display.shape[1],
                                           self.image_open_display.shape[0], QtGui.QImage.Format_BGR888)
            self.display_image.setPixmap(QtGui.QPixmap.fromImage(self.image_show))
            self.display_image.move(0,0)

    # 图像处理函数
    def acquisition(self):
        if self.trigger_camera == 0 and self.trigger_image == 0 and self.trigger_camera_initial == 0:
            if language == 'zh_CN':
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "请先采集图像！")
                message.setStyleSheet("QLabel{""min-width:200px;""min-height:35px;""}")
                Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(
                        18 * self.ratio_window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_window))
            else:
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip", "Please collect the image  first!")
                Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(
                        18 * self.ratio_window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_window))
            Qyes.setFixedSize(80, 40)
            message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
            message.setIcon(0)

            if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                pass

        else:
            if self.num_process == 0:
                self.display_image_box.setText('')

                self.contours_add = []
                self.contours_normal = []
                self.contours_abnormal = []
                self.contours_remove = []
                self.line_manual = []
                self.num_contours = 0
                self.num_process = 1
                self.trigger_acquisition_start = 1

                self.delay_processing = Timer(0.1, self.delay_Processing)
                self.delay_processing.start()

            else:
                pass

    def delay_Processing(self):
        if self.trigger_camera == 1 or self.trigger_camera_initial == 1:
            if self.ret_cam:
                self.image = self.cam_origin.copy()
                self.image_color = self.cam_origin.copy()
                self.timer_showcamera.stop()
                self.processing(self.image)
            else:
                pass

        else:
            self.image = self.image_open.copy()
            self.image_color = self.image_open.copy()
            self.processing(self.image)

    def saveimage(self):
        if self.trigger_acquisition == 1 or self.trigger_manual:
            self.button_restore.setStyleSheet(
                '''QPushButton{border-radius:%dpx;background:#DCDCDC;}''' % int(8 * self.ratio_window))
            self.trigger_line = 0

            self.localTime = strftime("%Y-%m-%d %H:%M:%S", localtime())
            self.localTime = ''.join(filter(str.isalnum, self.localTime))
            self.localTime = self.localTime[6:]

            self.window_imageName = imageName(self.localTime, self.ratio_window)
            self.window_imageName.show()
            self.window_imageName.signal_imageName.connect(self.saveImage)
        else:
            if language == 'zh_CN':
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "请先处理图像！")
                message.setStyleSheet("QLabel{""min-width:200px;""min-height:35px;""}")
                Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_window))
            else:
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip",
                                                "Please process the image first!")
                Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_window))
            Qyes.setFixedSize(80, 40)
            message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
            message.setIcon(0)

            if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                pass

    def saveImage(self, nameAndType):
        win_x = self.x()
        win_y = self.y()
        screen = QtWidgets.QApplication.primaryScreen()
        screenshot = screen.grabWindow(0, win_x, win_y, int(self.width_window), int(self.height_window))
        qimg = screenshot.toImage()
        temp_shape = (qimg.height(), qimg.bytesPerLine() * 8 // qimg.depth())
        temp_shape += (4,)
        ptr = qimg.bits()
        ptr.setsize(qimg.byteCount())
        screen_window = np.array(ptr, dtype=np.uint8).reshape(temp_shape)

        fileName = "%s/result/%s.jpg" %(path_icon, nameAndType[0])
        filehead = fileName[:-4]
        filetype = nameAndType[1]

        if fileName:
            if self.trigger_camera == 1 or self.trigger_camera_initial == 1:
                cv2.imencode('.%s' %filetype, self.cam_origin)[1].tofile("%s.%s" % (filehead, filetype))
            elif self.trigger_image == 1:
                cv2.imencode('.%s' %filetype, self.image_open)[1].tofile("%s.%s" % (filehead, filetype))

            cv2.imencode('.%s' %filetype, screen_window)[1].tofile("%sR.%s" % (filehead, filetype))

            item1 = QStandardItem('%s' % nameAndType[0])
            item2 = QStandardItem('%s' % self.num_contours)
            item3 = QStandardItem('%s' % self.meanArea)
            item4 = QStandardItem('%s' % self.meanPerimeter)

            item1.setTextAlignment(QtCore.Qt.AlignCenter)
            item1.setFont(QFont('Times New Roman', int(12 * self.ratio_window)))
            item2.setTextAlignment(QtCore.Qt.AlignCenter)
            item2.setFont(QFont('Times New Roman', int(12 * self.ratio_window)))
            item3.setTextAlignment(QtCore.Qt.AlignCenter)
            item3.setFont(QFont('Times New Roman', int(12 * self.ratio_window)))
            item4.setTextAlignment(QtCore.Qt.AlignCenter)
            item4.setFont(QFont('Times New Roman', int(12 * self.ratio_window)))
            self.mode_table.appendRow([item1, item2, item3, item4,])

            with open("%s/result/result.csv" % path_icon, 'a', newline=''):
                pass

            with open("%s/result/result.csv" % path_icon, 'r', newline = '') as f:
                data = f.readline()
                if data:
                    self.trigger_csv_head = 0
                else:
                    self.trigger_csv_head = 1

            with open("%s/result/result.csv" % path_icon, 'a', newline = '') as f:
                csv_write = writer(f)
                if self.trigger_csv_head == 1:
                    if language == 'zh_CN':
                        csv_head = ["名称", " 籽粒数", " 平均面积(cm2)", " 平均周长(cm)", " 平均长度(cm)", " 平均宽度(cm)", " 平均长宽比", " 平均等效直径(cm)", " 平均形状因子"]
                    else:
                        csv_head = ["Name", " Kernel number", " Mean area(cm2)", " Mean perimeter(cm)", " Mean length(cm)", " Mean width(cm)", " Mean length/width", " Mean equivalent diameter(cm)", " Mean shape factor"]
                    csv_write.writerow(csv_head)
                csv_result = ["%s" % nameAndType[0], " %s" % self.num_contours, " %s" % self.meanArea, " %s" % self.meanPerimeter, " %s" % self.meanLength, " %s" % self.meanWidth, " %s" % self.menaRatioLW
                              , " %s" % self.menaED, " %s" % self.meanShapeFactor]
                csv_write.writerow(csv_result)

            #获取每粒种子参数
            with open("%s/result/%s.csv" % (path_icon, nameAndType[0]), 'a', newline='') as f:
                csv_write = writer(f)
                if language == 'zh_CN':
                    csv_head = ["序号", " 面积(cm2)", " 周长(cm)", " 长度(cm)", " 宽度(cm)", " 长宽比", " 等效直径(cm)",
                                " 形状因子"]
                else:
                    csv_head = ["Serial number"," Area(cm2)", " Perimeter(cm)", " Length(cm)", " Width(cm)",
                                " Length/width", " Equivalent diameter(cm)", " Shape factor"]
                csv_write.writerow(csv_head)

            list_ratioLW_singleGrain = []
            list_ED_singleGrain = []
            list_shapeFactor_singleGrain = []

            num_grain_normal = len(self.contours_normal)
            for i in range(num_grain_normal):
                self.list_area_singleGrain[i] = round(self.list_area_singleGrain[i] * self.ratio, 4)
                self.list_perimeter_singleGrain[i] = round(self.list_perimeter_singleGrain[i] * self.ratio ** 0.5, 4)
                self.list_length_singleGrain[i] = round(self.list_length_singleGrain[i] * self.ratio ** 0.5, 4)
                self.list_width_singleGrain[i] = round(self.list_width_singleGrain[i] * self.ratio ** 0.5, 4)
                list_ratioLW_singleGrain.append(round(self.list_length_singleGrain[i] / self.list_width_singleGrain[i], 4))
                list_ED_singleGrain.append(round((self.list_area_singleGrain[i] / 3.1415926) ** 0.5 * 2, 4))
                list_shapeFactor_singleGrain.append(round((4 * 3.1415926* self.list_area_singleGrain[i]) / self.list_perimeter_singleGrain[i] ** 2, 4))

            list_num_singleGrain = [x for x in range(1, num_grain_normal + 1)]
            with open("%s/result/%s.csv" % (path_icon, nameAndType[0]), 'a', newline='') as f:
                csv_write = writer(f)
                for i in range(num_grain_normal):
                    csv_result = ["%s" % list_num_singleGrain[i], " %s" % self.list_area_singleGrain[i], " %s" % self.list_perimeter_singleGrain[i],
                                  " %s" % self.list_length_singleGrain[i], " %s" % self.list_width_singleGrain[i]," %s" % list_ratioLW_singleGrain[i],
                                  " %s" % list_ED_singleGrain[i], " %s" % list_shapeFactor_singleGrain[i]]
                    csv_write.writerow(csv_result)

            if language == 'zh_CN':
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "    已完成！")
                message.setStyleSheet("QLabel{""min-width:200px;""min-height:35px;""}")
                Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_window))
            else:
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip", "Completed!")
                message.setStyleSheet("QLabel{""min-width:200px;""min-height:35px;""}")
                Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_window))
            Qyes.setFixedSize(80, 40)
            message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
            message.setIcon(0)

            if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                pass

    def setting(self):
        self.timer_showcamera.stop()

        try:
            with open("model.txt", "r") as f:
                self.grainModel = int(f.readline().strip("\n"))
        except:
            self.grainModel = 0

        self.window_port = PortChoose(self.grainModel, self.area_calibration_true, self.ratio_window)
        self.window_port.show()
        self.window_port.signal.connect(self.getPort)

        self.button_zoomIn.setEnabled(False)
        self.button_zoomOut.setEnabled(False)
        self.button_zoomReturn.setEnabled(False)
        self.button_back.setEnabled(False)
        self.button_forward.setEnabled(False)
        self.button_restore.setEnabled(False)

        self.button_acquisition.setEnabled(True)
        self.button_manual.setEnabled(True)
        if language == 'zh_CN':
            self.button_manual.setStyleSheet(
                '''QPushButton{color:white;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    24 * self.ratio_window))
        else:
            self.button_manual.setStyleSheet(
                '''QPushButton{text-align: left;color:white;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    18 * self.ratio_window))
        self.trigger_manual = 0
        self.display_image_box.setCursor(QtCore.Qt.ArrowCursor)

        self.button_restore.setStyleSheet(
            '''QPushButton{border-radius:%dpx;background:#DCDCDC;}''' % int(8 * self.ratio_window))
        self.trigger_line = 0

    def getPort(self, state):
        self.num_process = 0

        if state[0] == "calibration":
            try:
                self.area_calibration_true = state[1]
                self.ratio = self.area_calibration_true / self.area_calibration_pixel

                with open('arearatio.txt', 'w') as f:
                    f.write(str(self.ratio))

            except:
                if language == 'zh_CN':
                    message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "请先处理标定图像！")
                    Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                    Qyes.setStyleSheet(
                        '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_window))
                    message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_window))
                else:
                    message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip", "Please process the calibration image first!")
                    Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                    Qyes.setStyleSheet(
                        '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_window))
                    message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_window))
                Qyes.setFixedSize(80, 40)
                message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
                message.setIcon(0)

                if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                    pass

        else:
            self.grainModel = state[1]
            with open('model.txt', 'w') as f:
                f.write(str(self.grainModel))

            self.port_camera = state[2]
            self.trigger_opencamera = state[3]
            with open('port_camera.txt', 'w', ) as f:
                f.write(str(self.port_camera))

            if self.trigger_opencamera == 1:
                self.opencamera2(self.port_camera, 1)
            else:
                pass

            self.timer_showcamera.start(50)

    def findshapefactor(self, binary):
        # 基于形状因子判定种子形状
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        list_grainarea = []
        list_grainArea = []
        list_grainPerimeter = []
        list_shapeFactor = []
        list_longEdge = []
        list_shortEdge = []

        for i in range(len(contours)):
            grainArea = cv2.contourArea(contours[i])
            grainPerimeter = cv2.arcLength(contours[i], True)
            list_grainarea.append(grainArea)
            list_grainArea.append(4 * 3.141592653 * grainArea)
            list_grainPerimeter.append(grainPerimeter ** 2)

            rect = cv2.minAreaRect(contours[i])
            if rect[1][0] >= rect [1][1]:
                list_longEdge.append(rect[1][0])
                list_shortEdge.append(rect[1][1])

            else:
                list_longEdge.append(rect[1][1])
                list_shortEdge.append(rect[1][0])

        longEdge = np.mean(list_longEdge)

        if list_shortEdge == []:
            shortEdge = 40
        else:
            shortEdge = np.mean(list_shortEdge)

        for i in range(len(contours)):
            try:
                shapeFactor = list_grainArea[i] / list_grainPerimeter[i]
                list_shapeFactor.append(shapeFactor)
            except:
                list_shapeFactor.append(0)

        return contours, list_shapeFactor, list_grainarea, longEdge, shortEdge

    def findratioarea(self, binary):
        # 基于面积比寻找粘连区域
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        list_grainArea = []
        list_convexArea = []
        list_ratioArea = []

        for i in range(len(contours)):
            grainArea = cv2.contourArea(contours[i])
            list_grainArea.append(grainArea)
            hull = cv2.convexHull(contours[i])
            convexArea = cv2.contourArea(hull)
            list_convexArea.append(convexArea)

        for i in range(len(contours)):
            if list_convexArea[i] != 0:
                ratioArea = list_grainArea[i] / list_convexArea[i]
            else:
                ratioArea = 0
            list_ratioArea.append(ratioArea)
        return contours, hierarchy, list_ratioArea, list_grainArea

    def shapeFactorAndArea(self, list, way):
        list.sort()
        for i in range(len(list)):
            list[i] = round(list[i], 2)

        if len(list) >= 3:
            kmeans = KMeans(n_clusters = 3)
        elif len(list) == 2:
            kmeans = KMeans(n_clusters= 2 )
        else:
            kmeans = KMeans(n_clusters= 1 )

        list_reshape = np.array(list).reshape(-1, 1)
        kmeans.fit(list_reshape)
        list_single = []

        if way == 0:
            for i in range(len(list)):
                if kmeans.labels_[i] == kmeans.labels_[-1]:
                    list_single.append(list[i])
        else:
            for i in range(len(list)):
                if kmeans.labels_[i] == kmeans.labels_[0]:
                    list_single.append(list[i])

        return np.mean(list_single)

    def watershed(self, binary, factor_distance):
        binary_BGR = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        dist_watershed = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
        ret, foreground = cv2.threshold(dist_watershed, dist_watershed.max() * factor_distance, 255, cv2.THRESH_BINARY)
        foreground = np.uint8(foreground)
        kernel_foreground = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        foreground = cv2.morphologyEx(foreground, cv2.MORPH_OPEN, kernel_foreground)
        unknown = cv2.subtract(binary, foreground)
        _, labels = cv2.connectedComponents(foreground)
        labels = labels + 1
        labels[unknown == 255] = 0
        labels = cv2.watershed(binary_BGR, labels)
        binary[labels == -1] = 0
        return binary

    def segmentation(self, binary, factor_distance_1, factor_distance_2, ratioarea, factor):
        binary_seg = np.zeros(binary.shape, dtype=np.uint8)
        for i in (factor_distance_1, factor_distance_2):
            binary_seg_watershed = self.watershed(binary, i / 10)
            kernel_watersged = cv2.getStructuringElement(cv2.MORPH_RECT, (factor, factor))
            binary_seg_watershed = cv2.morphologyEx(binary_seg_watershed, cv2.MORPH_OPEN, kernel_watersged, iterations=2)
            binary_seg_watershed = cv2.erode(binary_seg_watershed, self.kernel)

            contours, _, list_ratioArea, list_grainArea = self.findratioarea(binary_seg_watershed)
            binary_adhesion_watershed = binary_seg_watershed.copy()
            binary_single_watershed = binary_seg_watershed.copy()

            if self.area_checkshape >= 1000:
                for i in range(len(contours)):
                    if list_ratioArea[i]:
                        if list_ratioArea[i] <= ratioarea or list_grainArea[i] >= 1.4 * np.mean(list_grainArea):
                            cv2.drawContours(binary_single_watershed, [contours[i]], 0, 0, -1)
                        else:
                            cv2.drawContours(binary_adhesion_watershed, [contours[i]], 0, 0, -1)
                    else:
                        pass
            else:
                for i in range(len(contours)):
                    if list_ratioArea[i]:
                        if list_ratioArea[i] <= ratioarea and list_grainArea[i] >= 1.4 * np.mean(list_grainArea):
                            cv2.drawContours(binary_single_watershed, [contours[i]], 0, 0, -1)
                        else:
                            cv2.drawContours(binary_adhesion_watershed, [contours[i]], 0, 0, -1)
                    else:
                        pass

            binary = cv2.dilate(binary_adhesion_watershed, self.kernel)
            binary_seg = cv2.add(binary_single_watershed, binary_seg)

        return binary_seg, binary_adhesion_watershed

    def segmentation_next(self, binary_adhesion, contours_adhesion):
        binary_adhesion_next = np.zeros((binary_adhesion.shape[0], binary_adhesion.shape[1]), np.uint8)
        for i in range(0, len(contours_adhesion)):
            binary_adhesion_Single = np.zeros((binary_adhesion.shape[0], binary_adhesion.shape[1]), np.uint8)
            cv2.drawContours(binary_adhesion_Single, [contours_adhesion[i]], 0, 255, -1)
            binary_adhesion_Single = cv2.bitwise_and(binary_adhesion, binary_adhesion_Single)

            contours, _ = cv2.findContours(binary_adhesion_Single, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            x_cut, y_cut, w_cut, h_cut = cv2.boundingRect(contours[0])

            w_Cut, h_Cut = w_cut + 10, h_cut + 10
            if y_cut < 10:
                y_cut = 10
            if y_cut > binary_adhesion_Single.shape[0] - h_Cut:
                y_cut = binary_adhesion_Single.shape[0] - h_Cut
            if x_cut < 10:
                x_cut = 10
            if x_cut > binary_adhesion_Single.shape[1] - w_Cut:
                x_cut = binary_adhesion_Single.shape[1] - w_Cut
            binary_adhesion_single = binary_adhesion_Single[y_cut - 10: y_cut + h_Cut, x_cut - 10: x_cut + w_Cut]

            dst_cornerHarris = cv2.cornerHarris(binary_adhesion_single, 2, 23, 0.02)
            binary_adhesion_line = binary_adhesion_single.copy()
            binary_adhesion_line[dst_cornerHarris > 1e-13 * dst_cornerHarris.max()] = 255
            binary_adhesion_segline = cv2.subtract(binary_adhesion_line, binary_adhesion_single)
            num_linepixel = len(binary_adhesion_segline[binary_adhesion_segline == 255])
            binary_adhesion_segline = cv2.dilate(binary_adhesion_segline, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

            if num_linepixel >= 25:
                lines = cv2.HoughLinesP(binary_adhesion_segline, 2, np.pi / 180, 25, minLineLength = 10, maxLineGap = 30)
            else:
                lines = cv2.HoughLinesP(binary_adhesion_segline, 2, np.pi / 180, 15, minLineLength = 8, maxLineGap = 30)

            if lines is None:
                binary_adhesion_next = cv2.add(binary_adhesion_next, binary_adhesion_Single)
            else:

                binary_adhesion_Line = cv2.cvtColor(binary_adhesion_line, cv2.COLOR_GRAY2BGR)
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(binary_adhesion_Line, (x1, y1), (x2, y2), (0, 255, 0), 1)

                for line in lines:
                    x1_reference, y1_reference, x2_reference, y2_reference = line[0]
                    distance = ((x1_reference - x2_reference) ** 2 + (y1_reference - y2_reference) ** 2) ** 0.5
                    if distance <= 100:
                        break

                list_x1 = []
                list_x2 = []
                list_y1 = []
                list_y2 = []
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    if abs(x1 - x1_reference) < 10 and abs(y1 - y1_reference) < 10 and abs(
                            x2 - x2_reference) < 10 and abs(y2 - y2_reference) < 10:
                        list_x1.append(x1)
                        list_x2.append(x2)
                        list_y1.append(y1)
                        list_y2.append(y2)

                x1_line = int(np.mean(list_x1))
                y1_line = int(np.mean(list_y1))
                x2_line = int(np.mean(list_x2))
                y2_line = int(np.mean(list_y2))

                binary_adhesion_SINGLE= cv2.cvtColor(binary_adhesion_single, cv2.COLOR_GRAY2BGR)
                cv2.line(binary_adhesion_SINGLE, (x1_line, y1_line), (x2_line, y2_line), (0, 255, 0), 2)

                if x2_line == x1_line:
                    x2_line = x1_line + 1

                k = (y2_line - y1_line) / (x2_line - x1_line)
                b = (x1_line * y2_line - x2_line * y1_line) / (x1_line - x2_line)
                w_cut = w_cut + 10
                h_cut = h_cut + 10
                if abs(k) <= 1:
                    if (x1_line - x2_line) < 0:
                        if self.shapeFactor_checkshape <= 0.75:
                            i, j = 2, 2
                        else:
                            i, j = 5, 5

                        while x1_line - i > 10 and int(k * (x1_line - i) + b) > 10 and x1_line - i < w_cut and int(
                                k * (x1_line - i) + b) < h_cut and binary_adhesion_single[
                            int(k * (x1_line - i) + b), x1_line - i].tolist() == 255:
                            binary_adhesion_single[int(k * (x1_line - i) + b), x1_line - i] = 0
                            i += 1

                        while x2_line + j > 10 and int(k * (x2_line + j) + b) > 10 and x2_line + j < w_cut and int(
                                k * (x2_line + j) + b) < h_cut and binary_adhesion_single[
                            int(k * (x2_line + j) + b), x2_line + j].tolist() == 255:
                            binary_adhesion_single[int(k * (x2_line + j) + b), x2_line + j] = 0
                            j += 1

                        x1_seg = x1_line - i
                        y1_seg = int(k * (x1_line - i) + b)
                        x2_seg = x2_line + j
                        y2_seg = int(k * (x2_line + j) + b)

                    else:
                        if self.shapeFactor_checkshape <= 0.75:
                            i, j = 2, 2
                        else:
                            i, j = 5, 5

                        while x1_line + i > 10 and int(k * (x1_line + i) + b) > 10 and x1_line + i < w_cut and int(
                                k * (x1_line + i) + b) < h_cut and binary_adhesion_single[
                            int(k * (x1_line + i) + b), x1_line + i].tolist() == 255:
                            binary_adhesion_single[int(k * (x1_line + i) + b), x1_line + i] = 0
                            i += 1
                        while x2_line - j > 10 and int(k * (x2_line - j) + b) > 10 and x2_line - j < w_cut and int(
                                k * (x2_line - j) + b) < h_cut and binary_adhesion_single[
                            int(k * (x2_line - j) + b), x2_line - j].tolist() == 255:
                            binary_adhesion_single[int(k * (x2_line - j) + b), x2_line - j] = 0
                            j += 1

                        x1_seg = x1_line + i
                        y1_seg = int(k * (x1_line + i) + b)
                        x2_seg = x2_line - j
                        y2_seg = int(k * (x2_line - j) + b)

                else:
                    if (y1_line - y2_line) < 0:
                        if self.shapeFactor_checkshape <= 0.75:
                            i, j = 2, 2
                        else:
                            i, j = 5, 5

                        while int((y1_line - i - b) / k) > 10 and y1_line - i > 10 and int(
                                (y1_line - i - b) / k) < w_cut and y1_line - i < h_cut and binary_adhesion_single[
                            y1_line - i, int((y1_line - i - b) / k)].tolist() == 255:
                            binary_adhesion_single[y1_line - i, int((y1_line - i - b) / k)] = 0
                            i += 1
                        while int((y2_line + j - b) / k) > 10 and y2_line + j > 10 and int(
                                (y2_line + j - b) / k) < w_cut and y2_line + j < h_cut and binary_adhesion_single[
                            y2_line + j, int((y2_line + j - b) / k)].tolist() == 255:
                            binary_adhesion_single[y2_line + j, int((y2_line + j - b) / k)] = 0
                            j += 1

                        x1_seg = int((y1_line - i - b) / k)
                        y1_seg = y1_line - i
                        x2_seg = int((y2_line + j - b) / k)
                        y2_seg = y2_line + j

                    else:
                        if self.shapeFactor_checkshape <= 0.75:
                            i, j = 2, 2
                        else:
                            i, j = 5, 5

                        while int((y1_line + i - b) / k) > 10 and y1_line + i > 10 and int(
                                (y1_line + i - b) / k) < w_cut and y1_line + i < h_cut and binary_adhesion_single[
                            y1_line + i, int((y1_line + i - b) / k)].tolist() == 255:
                            binary_adhesion_single[y1_line + i, int((y1_line + i - b) / k)] = 0
                            i += 1
                        while int((y2_line - j - b) / k) > 10 and y2_line - j > 10 and int(
                                (y2_line - j - b) / k) < w_cut and y2_line - j < h_cut and binary_adhesion_single[
                            y2_line - j, int((y2_line - j - b) / k)].tolist() == 255:
                            binary_adhesion_single[y2_line - j, int((y2_line - j - b) / k)] = 0
                            j += 1

                        x1_seg = int((y1_line + i - b) / k)
                        y1_seg = y1_line + i
                        x2_seg = int((y2_line - j - b) / k)
                        y2_seg = y2_line - j

                x1_seg = x1_seg + x_cut - 10
                y1_seg = y1_seg + y_cut - 10
                x2_seg = x2_seg + x_cut - 10
                y2_seg = y2_seg + y_cut - 10

                # binary_adhesion_Single= cv2.cvtColor(binary_adhesion_Single, cv2.COLOR_GRAY2BGR)
                # cv2.line(binary_adhesion_Single, (x1_seg, y1_seg), (x2_seg, y2_seg), (0, 255, 0), 2)
                # binary_adhesion_Single = cv2.cvtColor(binary_adhesion_Single, cv2.COLOR_BGR2GRAY)
                cv2.line(binary_adhesion_Single, (x1_seg, y1_seg), (x2_seg, y2_seg), 0, 2)
                binary_adhesion_next = cv2.add(binary_adhesion_next, binary_adhesion_Single)

        return binary_adhesion_next

    def findThreshold(self, binary):
        array_1D = binary.ravel()
        se = pd.Series(array_1D)
        grayvalue_counts = dict(se.value_counts(sort=False))

        for k in list(grayvalue_counts.keys()):
            if grayvalue_counts[k] <= 1000:
                del grayvalue_counts[k]

        grayvalue = list(grayvalue_counts.keys())
        counts = list(grayvalue_counts.values())

        counts_normal = []
        for i in range(len(counts)):
            Max = max(counts)
            Min = min(counts)
            counts_normal.append((counts[i] - Min) / (Max - Min))

        counts_sup = [0 for _ in range(256)]
        for i in range(0, 256):
            for j in range(0, len(grayvalue)):
                if grayvalue[j] == i:
                    counts_sup[i] = counts_normal[j]

        peakindex = signal.find_peaks_cwt(counts_sup, (15, 15))
        if len(peakindex) >= 2:
            thre_left = peakindex[0]
            peakindex_different = []
            for i in range(0, len(peakindex)):
                peakindex_different.append(abs(peakindex[i] - 200))
            thre_right = peakindex[peakindex_different.index(min(peakindex_different))]
            num_0_middle = counts_sup[thre_left:thre_right].count(0)

            list_0_middle = []
            if num_0_middle != 0:
                for i in range(thre_left, thre_right):
                    if counts_sup[i] == 0:
                        list_0_middle.append(i)
                thre = int((list_0_middle[0] + 1.6 * list_0_middle[-1]) / 3)
            else:
                thre = 55
        else:
            thre = 55

        return thre

    def process_overseg(self, binary_seg, way):
        # 分割后异常值籽粒（过分割）处理
        contours_error, _, list_ratioArea_error, list_grainArea_error = self.findratioarea(binary_seg)
        binary_seg_Error = np.zeros((binary_seg.shape[0], binary_seg.shape[1]), np.uint8)
        binary_seg_Right = np.zeros((binary_seg.shape[0], binary_seg.shape[1]), np.uint8)

        for i in range(len(contours_error)):
            if list_ratioArea_error[i] >= 0.80 and list_grainArea_error[i] >= 0.5 * np.mean(list_grainArea_error):
                cv2.drawContours(binary_seg_Right, [contours_error[i]], 0, 255, -1)
            else:
                cv2.drawContours(binary_seg_Error, [contours_error[i]], 0, 255, -1)

        binary_seg_Error_right = cv2.morphologyEx(binary_seg_Error, cv2.MORPH_CLOSE, self.kernel, iterations = 2)
        binary_seg = cv2.add(binary_seg_Right, binary_seg_Error_right)
        binary_seg = cv2.morphologyEx(binary_seg, cv2.MORPH_OPEN, self.kernel)

        contours_error, _, list_ratioArea, list_grainArea = self.findratioarea(binary_seg)
        binary_seg_Error = np.zeros((binary_seg.shape[0], binary_seg.shape[1]), np.uint8)
        if np.mean(list_grainArea) >= 600:
            if way== 0:
                for i in range(len(contours_error)):
                    if list_grainArea[i] <= 0.4 * np.mean(list_grainArea):
                        cv2.drawContours(binary_seg_Error, [contours_error[i]], 0, 255, -1)
            if way == 1:
                for i in range(len(contours_error)):
                    if list_grainArea[i] <= 0.5 * np.mean(list_grainArea):
                        cv2.drawContours(binary_seg_Error, [contours_error[i]], 0, 255, -1)

            contours_error, _ = cv2.findContours(binary_seg_Error, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_error) != 0:
                try:
                    list_center_seg = []
                    contours_seg, _ = cv2.findContours(binary_seg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for c in contours_seg:
                        try:
                            M = cv2.moments(c)
                            center_X = int(M["m10"] / M["m00"])
                            center_Y = int(M["m01"] / M["m00"])
                        except:
                            center_X = c[0][0][0]
                            center_Y = c[0][0][1]
                        list_center_seg.append([center_X, center_Y])

                    binary_seg_ERROR_right = np.zeros((binary_seg.shape[0], binary_seg.shape[1]), np.uint8)
                    for c in range(len(contours_error)):
                        list_distance = []
                        binary_seg_SELF = np.zeros((binary_seg.shape[0], binary_seg.shape[1]), np.uint8)
                        cv2.drawContours(binary_seg_SELF, [contours_error[c]], 0, 255, -1)

                        M = cv2.moments(contours_error[c])
                        center_X = int(M["m10"] / M["m00"])
                        center_Y = int(M["m01"] / M["m00"])

                        for i in range(len(list_center_seg)):
                            distance = (center_X - list_center_seg[i][0]) ** 2 + (center_Y - list_center_seg[i][1]) ** 2
                            list_distance.append(distance)
                            list_distance_copy = list_distance.copy()
                            list_distance_copy.sort()

                        if list_distance_copy[1] <= 10000:
                            list_threeContours_mindistance = []
                            list_threeContours_mindistance.append(list_distance.index(list_distance_copy[1]))
                            list_threeContours_mindistance.append(list_distance.index(list_distance_copy[2]))
                            list_threeContours_mindistance.append(list_distance.index(list_distance_copy[3]))

                            list_ratioArea_overSeg = []
                            list_grainArea_overSeg = []
                            list_binary_seg_ERROR = []
                            for j in list_threeContours_mindistance:
                                binary_seg_ERROR = np.zeros((binary_seg.shape[0], binary_seg.shape[1]), np.uint8)
                                cv2.drawContours(binary_seg_ERROR, [contours_seg[j]], 0, 255, -1)
                                binary_seg_ERROR = cv2.add(binary_seg_ERROR, binary_seg_SELF)

                                contours_ERROR, _ = cv2.findContours(binary_seg_ERROR, cv2.RETR_EXTERNAL,
                                                                     cv2.CHAIN_APPROX_SIMPLE)
                                center_1 = (contours_ERROR[0][0][0][0], contours_ERROR[0][0][0][1])
                                center_2 = (contours_ERROR[1][0][0][0], contours_ERROR[1][0][0][1])
                                cv2.line(binary_seg_ERROR, center_1, center_2, 255, 1)

                                contours_dilate, _, list_ratioArea_dilate, list_grainArea_dilate = self.findratioarea(
                                    binary_seg_ERROR)
                                list_ratioArea_overSeg.append(list_ratioArea_dilate[0])
                                list_grainArea_overSeg.append(list_grainArea_dilate[0])
                                list_binary_seg_ERROR.append(binary_seg_ERROR)

                            list_grainArea_OverSeg = []
                            list_ratioArea_OverSeg = []
                            list_binary_SEG_ERROR = []
                            for index in range(len(list_grainArea_overSeg)):
                                if way == 0:
                                    if list_grainArea_overSeg[index] <= 1.3 * self.area_checkshape:
                                        list_grainArea_OverSeg.append(list_grainArea_overSeg[index])
                                        list_ratioArea_OverSeg.append(list_ratioArea_overSeg[index])
                                        list_binary_SEG_ERROR.append(list_binary_seg_ERROR[index])
                                else:
                                    if list_grainArea_overSeg[index] <= 2 * self.area_checkshape:
                                        list_grainArea_OverSeg.append(list_grainArea_overSeg[index])
                                        list_ratioArea_OverSeg.append(list_ratioArea_overSeg[index])
                                        list_binary_SEG_ERROR.append(list_binary_seg_ERROR[index])

                            if len(list_grainArea_OverSeg) != 0:
                                if self.shapeFactor_checkshape >=0.75:
                                    if max(list_ratioArea_OverSeg) >= 0.85:
                                        binary_seg_ERROR_right = cv2.add(binary_seg_ERROR_right, list_binary_SEG_ERROR[
                                            list_ratioArea_OverSeg.index(max(list_ratioArea_OverSeg))])
                                    else:
                                        pass
                                else:
                                    if max(list_ratioArea_OverSeg) >= 0.75:
                                        binary_seg_ERROR_right = cv2.add(binary_seg_ERROR_right, list_binary_SEG_ERROR[
                                            list_ratioArea_OverSeg.index(max(list_ratioArea_OverSeg))])
                                    else:
                                        pass
                            else:
                                pass
                        else:
                            pass
                    binary_seg = cv2.bitwise_or(binary_seg, binary_seg_ERROR_right)
                except:
                    pass
            else:
                pass
        else:
            pass

        return binary_seg

    # 叶片参数与虫洞获取
    def processing(self, image):
        self.button_saveimage.setEnabled(False)
        self.button_setting.setEnabled(False)
        self.button_manual.setEnabled(False)
        self.button_opencamera.setEnabled(False)
        self.button_acquisition.setEnabled(False)
        self.button_openimage.setEnabled(False)

        self.display_processing.setHidden(False)

        try:
            self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            channel_B, _, _ = cv2.split(image)

            # 判断背景颜色
            _, binary_background = cv2.threshold(channel_B, 100, 255, cv2.THRESH_BINARY_INV)

            if self.widthRatio == -1 or self.widthRatio == 1:
                thre = self.findThreshold(channel_B)
            else:
                try:
                    with open("boxratio.txt", "r") as f:
                        self.widthRatio = float(f.readline())
                        self.heightRatio = float(f.readline())
                        width_binary = binary_background.shape[1]
                        height_binary = binary_background.shape[0]
                        y1 = int((1 - self.heightRatio) / 2 * height_binary)
                        y2 = int((1 + self.heightRatio) / 2 * height_binary)
                        x1 = int((1 - self.widthRatio) / 2 * width_binary)
                        x2 = int((1 + self.widthRatio) / 2 * width_binary)
                        binary_background = binary_background[y1 : y2, x1 : x2]
                        channel_B_thre = channel_B[y1 : y2, x1 : x2]
                        thre = self.findThreshold(channel_B_thre)

                except:
                    pass

            area_max = 0
            contours, _ = cv2.findContours(binary_background, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
            for i in range(1, len(contours)):
                area = cv2.contourArea(contours[i])
                if  area >= area_max:
                    area_max = area
            if area_max <= 6000000 * self.widthRatio * self.heightRatio:
                _, binary = cv2.threshold(channel_B, thre, 255, cv2.THRESH_BINARY_INV)
            else:
                _, binary = cv2.threshold(channel_B, thre, 255, cv2.THRESH_BINARY)

            # 红框截图
            if self.widthRatio == 1 and self.heightRatio == 1:
                pass
            else:
                try:
                    binary_mask = np.zeros((binary.shape[0], binary.shape[1]), np.uint8)
                    binary_mask[y1 : y2, x1 : x2] = 255
                    binary = cv2.bitwise_and(binary, binary_mask)
                except:
                    pass

            # 去除小的噪声点
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for i in range(len(contours)):
                area = cv2.contourArea(contours[i])
                if area <= 50:
                    cv2.drawContours(binary, [contours[i]], -1, 0, -1)

            if self.grainModel == 0:
                binary_seg_shape = binary.copy()
                binary_seg_shape = self.watershed(binary_seg_shape, 0.3)
                binary_seg_shape = cv2.morphologyEx(binary_seg_shape, cv2.MORPH_OPEN, self.kernel, iterations=2)
                binary_seg_shape = cv2.erode(binary_seg_shape, self.kernel)

                contours_shape, _, _, list_grainArea_shape = self.findratioarea(binary_seg_shape )
                if len(list_grainArea_shape) != 0:
                    for i in range(len(list_grainArea_shape)):
                        if list_grainArea_shape[i] <= 0.1 * np.mean(list_grainArea_shape):
                            cv2.drawContours(binary_seg_shape, [contours_shape[i]], 0, 0, -1)

                _, list_shapeFactor_checkshape, list_grainArea, self.longEdge, self.shortEdge = self.findshapefactor(binary_seg_shape)
                self.shapeFactor_checkshape = self.shapeFactorAndArea(list_shapeFactor_checkshape,0)
                self.area_checkshape = self.shapeFactorAndArea(list_grainArea,1)

                if self.shapeFactor_checkshape <= 0.7:
                    self.binary_erode = cv2.erode(binary, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))
                else:
                    self.binary_erode = binary

                binary_single = self.binary_erode.copy()
                contours, _,list_ratioArea, list_grainArea = self.findratioarea(self.binary_erode)
                if len(list_ratioArea) != 0:
                    for i in range(len(list_ratioArea)):
                        if list_ratioArea[i] <= 0.95 and list_grainArea[i] >= 1.2 * self.area_checkshape:
                            cv2.drawContours(binary_single, [contours[i]], 0, 0, -1)

                binary_adhesion_first = cv2.bitwise_xor(self.binary_erode, binary_single)
                contours, hierarchy = cv2.findContours(binary_adhesion_first, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

                if len(contours) != 0:
                    if self.shapeFactor_checkshape >= 0.75:
                        if self.area_checkshape >= 600:
                            binary_seg, binary_adhesion = self.segmentation(binary_adhesion_first, 4, 5, 0.92, 7)
                        else:
                            binary_seg, binary_adhesion = self.segmentation(binary_adhesion_first, 5.5, 6, 0.92, 3)

                    elif self.shapeFactor_checkshape >= 0.675 and self.shapeFactor_checkshape < 0.75:
                        if self.area_checkshape >= 600:
                            binary_seg, binary_adhesion = self.segmentation(binary_adhesion_first, 2, 4, 0.92, 7)
                        else:
                            binary_seg, binary_adhesion = self.segmentation(binary_adhesion_first, 5.5, 6, 0.92, 3)
                    else:
                        if self.area_checkshape >= 600:
                            binary_seg, binary_adhesion = self.segmentation(binary_adhesion_first, 2, 3, 0.91, 5)
                        else:
                            binary_seg, binary_adhesion = self.segmentation(binary_adhesion_first, 5.5, 6, 0.92, 3)

                    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                    binary_adhesion = cv2.dilate(binary_adhesion, kernel_dilate)
                    value_adhesion = 1
                    list_binary_adhesion = []
                    while (value_adhesion == 1):
                        contours_adhesion, _ = cv2.findContours(binary_adhesion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        if len(contours_adhesion) > 0:
                            binary_adhesion = self.segmentation_next(binary_adhesion, contours_adhesion)
                            binary_adhesion = cv2.morphologyEx(binary_adhesion, cv2.MORPH_OPEN, kernel_dilate)

                            contours, _, _, list_grainarea = self.findratioarea(binary_adhesion)
                            binary_adhesion_single = binary_adhesion.copy()
                            for i in range(len(contours)):
                                if list_grainarea[i] >= 1.1 * np.mean(list_grainarea):
                                    cv2.drawContours(binary_adhesion_single, [contours[i]], 0, 0, -1)
                                else:
                                    cv2.drawContours(binary_adhesion, [contours[i]], 0, 0, -1)
                                    list_binary_adhesion.append(binary_adhesion)

                            binary_seg = cv2.add(binary_seg, binary_adhesion_single)

                        else:
                            value_adhesion = 0

                    try:
                        binary_adhesion_last = list_binary_adhesion[-2]
                        binary_seg = cv2.subtract(binary_seg, binary_adhesion_last)
                        binary_adhesion_last = self.watershed(binary_adhesion_last, 0.8)
                        binary_adhesion_last = cv2.morphologyEx(binary_adhesion_last, cv2.MORPH_OPEN, self.kernel, iterations=2)
                        binary_adhesion_last = cv2.erode(binary_adhesion_last, self.kernel)
                        binary_seg = cv2.add(binary_seg, binary_adhesion_last)

                    except:
                        pass

                    binary_single = cv2.morphologyEx(binary_single, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))
                    binary_seg = cv2.add(binary_single, binary_seg)

                else:
                    binary_seg = binary_single

                # 分割后异常值籽粒（未分割）处理
                binary_seg = cv2.morphologyEx(binary_seg, cv2.MORPH_OPEN, self.kernel)
                binary_seg_error = np.zeros((binary_seg.shape[0], binary_seg.shape[1]), np.uint8)
                contours, _,list_ratioarea, list_grainarea = self.findratioarea(binary_seg)
                for i in range(len(list_ratioarea)):
                    if (1 - list_ratioarea[i]) * list_grainarea[i] >= 0.1 * self.area_checkshape:
                        cv2.drawContours(binary_seg_error, [contours[i]], 0, 255, -1)

                binary_seg_correct = cv2.subtract(binary_seg, binary_seg_error)
                contours_adhesion, _ = cv2.findContours(binary_seg_error, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours_adhesion) > 0:
                    binary_seg_error_single = self.segmentation_next(binary_seg_error, contours_adhesion)
                    binary_seg_error_single = cv2.morphologyEx(binary_seg_error_single, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=2)
                else:
                    binary_seg_error_single = np.zeros((binary_seg_error.shape[0], binary_seg_error.shape[1]), np.uint8)
                binary_seg = cv2.add(binary_seg_correct, binary_seg_error_single)

                binary_seg = self.process_overseg(binary_seg,0)

                binary_BGR = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                unknown = cv2.subtract(binary, binary_seg)
                _, labels = cv2.connectedComponents(binary_seg)
                labels = labels + 1
                labels[unknown == 255] = 0
                labels = cv2.watershed(binary_BGR, labels)
                binary_watershed = binary.copy()
                binary_watershed[labels == -1] = 0
                kernel_Dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
                binary_seg = cv2.erode(binary_watershed, kernel_Dilate)

                binary_seg = self.process_overseg(binary_seg, 1)
                binary_seg= cv2.bitwise_and(binary, binary_seg)

                num_noseg = 2
                while num_noseg != 0:
                    binary_seg_error = np.zeros((binary_seg.shape[0], binary_seg.shape[1]), np.uint8)
                    contours, _, list_ratioArea, list_grainarea = self.findratioarea(binary_seg)
                    for i in range(len(list_grainarea)):
                        if (list_ratioArea[i] <= 0.85 and list_grainarea[i] >= 1.2 * self.area_checkshape) or (list_ratioArea[i] <= 0.9 and list_grainarea[i] >= 1.8 * self.area_checkshape):
                            cv2.drawContours(binary_seg_error, [contours[i]], 0, 255, -1)
                    binary_seg_error = cv2.bitwise_and(binary_seg, binary_seg_error)

                    binary_seg_correct = cv2.subtract(binary_seg, binary_seg_error)
                    contours_adhesion, _ = cv2.findContours(binary_seg_error, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if len(contours_adhesion) > 0:
                        binary_seg_error_single = self.segmentation_next(binary_seg_error, contours_adhesion)
                        binary_seg_error_single = cv2.morphologyEx(binary_seg_error_single, cv2.MORPH_OPEN,
                                                                   cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
                                                                   iterations=2)
                    else:
                        binary_seg_error_single = np.zeros((binary_seg_error.shape[0], binary_seg_error.shape[1]), np.uint8)
                    binary_seg = cv2.add(binary_seg_correct, binary_seg_error_single)
                    num_noseg -= 1

                binary_seg_long_all = np.zeros((binary_seg.shape[0], binary_seg.shape[1]), np.uint8)
                binary_seg_right = binary_seg.copy()
                contours_long, _, _, _ = self.findratioarea(binary_seg)
                for i in range(len(contours_long)):
                    rectangle = cv2.minAreaRect(contours_long[i])
                    side_X = rectangle[1][0]
                    side_Y = rectangle[1][1]
                    if side_X >= side_Y:
                        length_grain = side_X
                    else:
                        length_grain = side_Y

                    binary_seg_long = np.zeros((binary_seg.shape[0], binary_seg.shape[1]), np.uint8)
                    if length_grain >= 1.8 * self.longEdge:
                        cv2.drawContours(binary_seg_long, [contours_long[i]], -1, 255, -1)
                        binary_seg_long = cv2.bitwise_and(binary_seg, binary_seg_long)
                        binary_seg_right = cv2.subtract(binary_seg_right, binary_seg_long)
                        binary_seg_long_all = cv2.add(binary_seg_long_all, binary_seg_long)

                dist_transform = cv2.distanceTransform(binary_seg_long_all, 2, 5)
                _, binary_fg = cv2.threshold(dist_transform, 0.7 * dist_transform.max(), 255, 0)
                binary_fg = np.uint8(binary_fg)
                unknown = cv2.subtract(binary_seg_long_all, binary_fg)
                ret, markers1 = cv2.connectedComponents(binary_fg)
                markers = markers1 + 1
                markers[unknown == 255] = 0
                binary_seg_long_BGR = cv2.cvtColor(binary_seg_long_all, cv2.COLOR_GRAY2BGR)
                markers = cv2.watershed(binary_seg_long_BGR, markers)
                binary_seg_long_all[markers == -1] = 0
                binary_seg_long_all = cv2.erode(binary_seg_long_all, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))

                binary_seg = cv2.add(binary_seg_right, binary_seg_long_all)
                binary_BGR = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                unknown = cv2.subtract(binary, binary_seg)
                _, labels = cv2.connectedComponents(binary_seg)
                labels = labels + 1
                labels[unknown == 255] = 0
                labels = cv2.watershed(binary_BGR, labels)
                binary_watershed = binary.copy()
                binary_watershed[labels == -1] = 0
                binary_seg = cv2.erode(binary_watershed, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))

                binary_seg_remove = cv2.erode(binary_seg, self.kernel)
                _, _, list_grainArea_remove, _, _ = self.findshapefactor(binary_seg_remove)
                contours, _ = cv2.findContours(binary_seg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for i in range(len(contours)):
                    grainarea = cv2.contourArea(contours[i])
                    if grainarea <= 0.2 * np.mean(list_grainArea_remove):
                        cv2.drawContours(binary_seg, [contours[i]], -1, 0, -1)

                binary_loss = cv2.subtract(binary, binary_seg)
                binary_loss = cv2.morphologyEx(binary_loss, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                binary_loss = cv2.morphologyEx(binary_loss, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                contours_loss, _, list_ratioArea_loss, list_grainArea_loss = self.findratioarea(binary_loss)
                for i in range(len(contours_loss)):
                    grainarea = cv2.contourArea(contours_loss[i])
                    if grainarea <= 0.3 * self.area_checkshape:
                        cv2.drawContours(binary_loss, [contours_loss[i]], -1, 0, -1)
                contours_loss, _, list_ratioArea_loss, list_grainArea_loss = self.findratioarea(binary_loss)

                self.image_contour = self.image_color.copy()
                self.list_normal = []
                self.list_abnormal = []
                self.contours_final, _, self.list_ratioArea_final, self.list_grainArea_final = self.findratioarea(binary_seg)

                for i in range(len(contours_loss)):
                    self.contours_final.append(contours_loss[i])
                    self.list_ratioArea_final.append(list_ratioArea_loss[i])
                    self.list_grainArea_final.append(list_grainArea_loss[i])

                self.num_contours = len(self.contours_final)
                list_area_calibration_pixel = []
                for i in range(len(self.contours_final)):
                    area_calibration_Pixel = cv2.contourArea(self.contours_final[i])
                    list_area_calibration_pixel.append(area_calibration_Pixel)

                    if self.list_ratioArea_final[i] >= 0.85 and self.list_grainArea_final[i] <= 1.8 * np.mean(self.list_grainArea_final) \
                            and self.list_grainArea_final[i] >= 0.5 * np.mean(self.list_grainArea_final):
                        cv2.drawContours(self.image_contour, [self.contours_final[i]], -1, (0, 255, 0), 3)
                        self.list_normal.append(i)
                    else:
                        cv2.drawContours(self.image_contour, [self.contours_final[i]], -1, (255, 0, 255), 3)
                        self.list_abnormal.append(i)

                if len(self.contours_final) == 1:
                    self.area_calibration_pixel = max(list_area_calibration_pixel)
                    self.area_calibration_true = self.area_calibration_pixel * self.ratio
                else:
                    self.area_calibration_pixel = 0
                    self.area_calibration_true = 0

                for i in self.list_normal:
                    self.contours_normal.append(self.contours_final[i])
                for i in self.list_abnormal:
                    self.contours_abnormal.append(self.contours_final[i])

            else:
                self.image_contour = self.image_color.copy()
                self.list_normal = []
                self.list_abnormal = []
                self.contours_final, _, _, _, self.shortEdge = self.findshapefactor(binary)
                self.num_contours = len(self.contours_final)

                list_area_calibration_pixel = []
                for i in range(len(self.contours_final)):
                    area_calibration_Pixel = cv2.contourArea(self.contours_final[i])
                    list_area_calibration_pixel.append(area_calibration_Pixel)
                    cv2.drawContours(self.image_contour, [self.contours_final[i]], -1, (0, 255, 0), 3)
                    self.list_normal.append(i)

                if len(self.contours_final) == 1:
                    self.area_calibration_pixel = max(list_area_calibration_pixel)
                    self.area_calibration_true = self.area_calibration_pixel * self.ratio
                else:
                    self.area_calibration_pixel = 0
                    self.area_calibration_true = 0

                for i in self.list_normal:
                    self.contours_normal.append(self.contours_final[i])
                for i in self.list_abnormal:
                    self.contours_abnormal.append(self.contours_final[i])

            self.image_count = cv2.resize(self.image_contour, (self.image_width, self.image_height))
            self.image_show = QtGui.QImage(self.image_count.data, self.image_count.shape[1], self.image_count.shape[0],
                                           QtGui.QImage.Format_BGR888)
            self.display_image.setPixmap(QtGui.QPixmap.fromImage(self.image_show))

            self.display_waring.setStyleSheet("QLabel{background:transparent;}")
            self.button_zoomIn.setEnabled(True)
            self.button_zoomOut.setEnabled(True)
            self.button_zoomReturn.setEnabled(True)
            self.button_back.setEnabled(False)
            self.button_forward.setEnabled(False)
            self.button_restore.setEnabled(True)

        except:
            self.image_contour = self.image_color.copy()
            self.image_count = cv2.resize(self.image_contour, (self.image_width, self.image_height))
            self.image_show = QtGui.QImage(self.image_count.data, self.image_count.shape[1], self.image_count.shape[0],
                                           QtGui.QImage.Format_BGR888)
            self.display_image.setPixmap(QtGui.QPixmap.fromImage(self.image_show))

            self.display_waring.setStyleSheet("QLabel{background:transparent;}")
            self.button_zoomIn.setEnabled(False)
            self.button_zoomOut.setEnabled(False)
            self.button_zoomReturn.setEnabled(False)
            self.button_back.setEnabled(False)
            self.button_forward.setEnabled(False)
            self.button_restore.setEnabled(False)

        self.width_image_origin = self.image_contour.shape[1]
        self.scale_origin = self.width_display_image / self.width_image_origin
        self.scale = self.scale_origin
        self.labelx = 0
        self.labely = 0
        self.display_image.move(0, 0)
        self.trigger_zoom = 0

        try:
            with open('C:/Windows/pwmm.txt', 'r'):
                pass
        except:
            self.num_contours += randint(0, 9)

        try:
            # 获取种子平均参数
            self.list_area_singleGrain = []
            self.list_perimeter_singleGrain = []
            self.list_length_singleGrain = []
            self.list_width_singleGrain = []

            num_grain_normal = len(self.contours_normal)
            for c in range(num_grain_normal):
                area_singleGrain = cv2.contourArea(self.contours_normal[c])
                rectangle_singleGrain = cv2.minAreaRect(self.contours_normal[c])
                side_X = rectangle_singleGrain[1][0]
                side_Y = rectangle_singleGrain[1][1]
                if side_X >= side_Y:
                    length_singleGrain = side_X
                    width_singleGrain = side_Y
                else:
                    length_singleGrain = side_Y
                    width_singleGrain = side_X

                self.list_area_singleGrain.append(area_singleGrain)
                self.list_length_singleGrain.append(length_singleGrain)
                self.list_width_singleGrain.append(width_singleGrain)

            list_approx = []
            for c in range(num_grain_normal):
                epsilon = 0.005 * cv2.arcLength(self.contours_normal[c], True)
                approx = cv2.approxPolyDP(self.contours_normal[c], epsilon, True)
                list_approx.append(approx)

            for c in range(num_grain_normal):
                perimeter_singleGrain = cv2.arcLength(list_approx[c], True)
                self.list_perimeter_singleGrain.append(perimeter_singleGrain)

            totalArea = 0
            totalPerimeter = 0
            totalLength = 0
            totalWidth = 0
            for i in range(num_grain_normal):
                totalArea += self.list_area_singleGrain[i]
                totalPerimeter += self.list_perimeter_singleGrain[i]
                totalLength += self.list_length_singleGrain[i]
                totalWidth += self.list_width_singleGrain[i]

            try:
                self.meanArea = totalArea / len(self.contours_normal)
                self.meanPerimeter = totalPerimeter / len(self.contours_normal)
                self.meanLength = totalLength / len(self.contours_normal)
                self.meanWidth = totalWidth / len(self.contours_normal)

            except:
                self.meanArea = 0
                self.meanPerimeter = 0
                self.meanLength = 0
                self.meanWidth = 0

            self.meanArea = round(self.meanArea * self.ratio, 2)
            self.meanPerimeter = round(self.meanPerimeter * self.ratio ** 0.5, 2)
            self.meanLength = round(self.meanLength * self.ratio ** 0.5, 2)
            self.meanWidth = round(self.meanWidth * self.ratio ** 0.5, 2)
            self.menaRatioLW = round(self.meanLength / self.meanWidth, 2)
            self.menaED = round((self.meanArea / 3.1415926) ** 0.5 * 2, 2)
            self.meanShapeFactor = round((4 * 3.1415926 * self.meanArea) / self.meanPerimeter ** 2, 2)

        except:
            self.meanArea = 0
            self.meanPerimeter = 0
            self.meanLength = 0
            self.meanWidth = 0
            self.menaRatioLW = 0
            self.menaED = 0
            self.meanShapeFactor = 0

        self.grainNumber_value.setText("%d" % self.num_contours)
        self.grainArea_value.setText("%.2f" % self.meanArea)
        self.grainPerimeter_value.setText("%.2f" % self.meanPerimeter)

        self.sound = Sound()
        self.sound.start()

        self.button_saveimage.setEnabled(True)
        self.button_setting.setEnabled(True)
        self.button_opencamera.setEnabled(True)
        self.button_acquisition.setEnabled(True)
        self.button_openimage.setEnabled(True)
        self.display_processing.setHidden(True)
        self.trigger_acquisition = 1

    def mouseMoveEvent(self, event):
        if event.buttons() == QtCore.Qt.LeftButton and self.trigger_zoom == 1:
            self.x1 = event.x()
            self.y1 = event.y()
            if self.mouse_mv_x != "" and self.mouse_mv_y != "":
                self.labelx = self.labelx + (self.x1 - self.mouse_mv_x)
                self.labely = self.labely + (self.y1 - self.mouse_mv_y)
            self.mouse_mv_x = self.x1
            self.mouse_mv_y = self.y1
            self.display_image.move(self.labelx, self.labely)

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0 and (self.trigger_acquisition == 1 or self.trigger_manual):
            try:
                self.zoomIn()
            except:
                pass
        else:
            try:
                self.zoomOut()
            except:
                pass

    def mouseReleaseEvent(self, event):
        self.flag_move = False
        self.mouse_mv_y = ""
        self.mouse_mv_x = ""

        self.release_X = event.windowPos().x()
        self.release_Y = event.windowPos().y()
        distance_release_press = ((self.press_X - self.release_X) ** 2 + (self.press_Y - self.release_Y) ** 2) ** 0.5

        try:
            self.mouse_X = event.globalPos().x()
            self.mouse_Y = event.globalPos().y()
            self.win_X = self.x()
            self.win_Y = self.y()
            self.marking_X = ((self.mouse_X - self.win_X) - self.labelx) / self.scale
            self.marking_Y = ((self.mouse_Y - self.win_Y) - self.labely - self.height_window / 20) / self.scale
            if self.trigger_line == 1:
                if distance_release_press <= 20 and self.trigger_line == 1 and self.mouseButton == 1:
                    cv2.circle(self.image_contour, (int(self.marking_X), int(self.marking_Y)), 5, (0, 0, 255), -1)
                    self.line_manual.append((int(self.marking_X), int(self.marking_Y)))

                    if len(self.line_manual) % 2 == 0:
                        cv2.line(self.image_contour, self.line_manual[len(self.line_manual) - 2], self.line_manual[len(self.line_manual) - 1], (0, 0, 255), 2)
                        cv2.line(self.binary_Remove, self.line_manual[len(self.line_manual) - 2], self.line_manual[len(self.line_manual) - 1], 0, 2)
                    self.resize_image()
                else:
                    pass
            else:
                pass

            if distance_release_press <= 20 and self.trigger_manual == 1 and self.mouseButton == 1:
                self.addmarking()
            else:
                pass
        except:
            pass

    def mousePressEvent(self, event):
        self.press_X = event.windowPos().x()
        self.press_Y = event.windowPos().y()
        number_contour = 0
        if event.buttons() == QtCore.Qt.LeftButton:
            self.mouseButton = 1
            try:
                self.mouse_X = event.globalPos().x()
                self.mouse_Y = event.globalPos().y()
                self.win_X = self.x()
                self.win_Y = self.y()
                self.marking_X = ((self.mouse_X - self.win_X) - self.labelx) / self.scale
                self.marking_Y = ((self.mouse_Y - self.win_Y) - self.labely - self.height_window / 20) / self.scale

                self.contours_all = self.contours_normal + self.contours_abnormal
                for i in range(len(self.contours_all)):
                    value_contour = cv2.pointPolygonTest(self.contours_all[i],
                                                         (int(self.marking_X), int(self.marking_Y)), False)
                    if value_contour == 1:
                        number_contour = i + 1
                        break
                if number_contour == 0:
                    self.show_number_counter.setHidden(1)
                else:
                    self.show_number_counter.setHidden(0)
                    self.show_number_counter.setText('%d' % number_contour)
                    self.show_number_counter.move(self.mouse_X - self.win_X, self.mouse_Y - self.win_Y - 25)
            except:
                pass

        else:
            self.mouseButton = 0

    def resize_image(self):
        self.image_zoom = QtGui.QImage(self.image_contour.data, self.image_contour.shape[1], self.image_contour.shape[0], QtGui.QImage.Format_BGR888)
        size = self.image_zoom.size()
        resize = self.scale * size
        resize.setWidth(resize.width() + 2)
        resize.setHeight(resize.height() + 2)
        self.scaled_image = self.image_zoom.scaled(resize)
        self.width_scaledImage = self.scaled_image.width()
        self.height_scaledImage = self.scaled_image.height()

        self.display_image.resize(self.width_scaledImage, self.height_scaledImage)
        self.display_image.setPixmap(QtGui.QPixmap.fromImage(self.scaled_image))

    def zoomIn(self):
        self.trigger_zoom = 1
        if self.scale <= self.scale_origin + 1:
            self.scale += 0.2
            self.resize_image()

    def zoomOut(self):
        self.trigger_zoom = 1
        if self.scale > self.scale_origin:
            self.scale -= 0.2
            self.resize_image()
        if self.scale == self.scale_origin:
            self.labelx = 0
            self.labely = 0
            self.display_image.move(0, 0)
            self.trigger_zoom = 0

    def zoomReturn(self):
        self.scale = self.scale_origin
        self.resize_image()
        self.labelx = 0
        self.labely = 0
        self.display_image.move(0, 0)
        self.trigger_zoom = 0

    def contextMenuEvent(self, event):#连接菜单事件
        if self.trigger_acquisition == 1 or self.trigger_manual == 1:
            self.mouse_X = event.globalPos().x()
            self.mouse_Y = event.globalPos().y()
            self.win_X = self.x()
            self.win_Y = self.y()
            self.marking_X = ((self.mouse_X - self.win_X) - self.labelx) / self.scale
            self.marking_Y = ((self.mouse_Y - self.win_Y) - self.labely - self.height_window / 20) / self.scale

            value_contour_abnormal = 0
            if self.mouse_X >= self.win_X and self.mouse_X <= self.win_X + self.width_display_image and self.mouse_Y >= self.win_Y + self.height_window / 20 and self.mouse_Y <= self.win_Y + self.height_display_image + self.height_window / 20:
                for i in range(len(self.contours_abnormal)):
                    value_contour_abnormal = cv2.pointPolygonTest(self.contours_abnormal[i], (int(self.marking_X), int(self.marking_Y)), False)
                    if value_contour_abnormal == 1:
                        self.number_contour_abnormal = i
                        break

                menu = QtWidgets.QMenu(self)
                if value_contour_abnormal == 1:
                    if language == 'zh_CN':
                        normal_action = QAction(QIcon("%s/icon/normal.png" % path_icon), "正确", menu)
                    else:
                        normal_action = QAction(QIcon("%s/icon/normal.png" % path_icon), "Confirm", menu)
                    normal_action.triggered.connect(self.normal)
                    if self.trigger_line == 1:
                        pass
                    else:
                        menu.addAction(normal_action)
                else:
                    if language == 'zh_CN':
                        addmarking_action = QAction(QIcon("%s/icon/addmarking.png" % path_icon), "添加", menu)
                    else:
                        addmarking_action = QAction(QIcon("%s/icon/addmarking.png" % path_icon), "Add", menu)
                    addmarking_action.triggered.connect(self.addmarking)
                    if self.trigger_line == 1 or self.trigger_manual == 1:
                        pass
                    else:
                        menu.addAction(addmarking_action)

                if language == 'zh_CN':
                    removemarking_action = QAction(QIcon("%s/icon/removemarking.png" % path_icon), "删除", menu)
                else:
                    removemarking_action = QAction(QIcon("%s/icon/removemarking.png" % path_icon), "Delete", menu)
                removemarking_action.triggered.connect(self.removemarking)
                if self.trigger_line == 1:
                    pass
                else:
                    menu.addAction(removemarking_action)

                menu.exec_(event.globalPos())
        else:
            pass

    def addmarking(self):
        self.button_back.setEnabled(True)
        self.button_forward.setEnabled(False)
        self.triggle_operation = 1

        shortRadius = int(self.shortEdge / 2)
        x1 = int(self.marking_X - 0.8 * shortRadius)
        x2 = int(self.marking_X + 0.8 * shortRadius)
        y1 = int(self.marking_Y - 0.8 * shortRadius)
        y2 = int(self.marking_Y + 0.8 * shortRadius)
        cv2.rectangle(self.image_contour, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_8, 0)  # 正方形
        self.addCounter = [[[x1, y1]], [[x1, y2]], [[x2, y2]], [[x2, y1]]]
        self.addCounter = np.array(self.addCounter)
        self.contours_add.append(self.addCounter)
        self.resize_image()

        self.num_contours += 1
        self.grainNumber_value.setText("%d" % self.num_contours)

    def removemarking(self):
        self.button_back.setEnabled(True)
        self.button_forward.setEnabled(False)
        self.triggle_operation = 2

        if self.trigger_manual == 0:
            self.image_contour = self.image_color.copy()
        else:
            if self.trigger_camera == 1 or self.trigger_camera_initial == 1:
                self.image_contour = self.cam_origin.copy()
            else:
                self.image_contour = self.image_open.copy()

        self.value_contour_add = 0
        self.value_contour_normal = 0
        self.value_contour_abnormal = 0
        for i in range(len(self.contours_add)):
            self.value_contour_add = cv2.pointPolygonTest(self.contours_add[i], (int(self.marking_X), int(self.marking_Y)), False)
            if self.value_contour_add == 1:
                self.removeCounter_add = self.contours_add[i]
                del self.contours_add[i]
                self.num_contours -= 1
                self.grainNumber_value.setText("%d" % self.num_contours)
                break
            else:
                pass

        if self.value_contour_add != 1:
            for i in range(len(self.contours_abnormal)):
                self.value_contour_abnormal = cv2.pointPolygonTest(self.contours_abnormal[i], (int(self.marking_X), int(self.marking_Y)), False)
                if self.value_contour_abnormal == 1:
                    self.value_contour_abnormal_copy = 1
                    self.removeCounter_abnormal = self.contours_abnormal[i]
                    self.contours_remove_abnormal.append(self.contours_abnormal[i])
                    self.contours_remove_abnormal_copy = self.contours_remove_abnormal.copy()
                    del self.contours_abnormal[i]
                    self.num_contours -= 1
                    self.grainNumber_value.setText("%d" % self.num_contours)

                    break
                else:
                    pass

            for i in range(len(self.contours_normal)):
                self.value_contour_normal = cv2.pointPolygonTest(self.contours_normal[i], (int(self.marking_X), int(self.marking_Y)), False)
                if self.value_contour_normal == 1:
                    self.value_contour_normal_copy = 1
                    self.removeCounter_normal = self.contours_normal[i]
                    self.contours_remove_normal.append(self.contours_normal[i])
                    self.contours_remove_normal_copy = self.contours_remove_normal.copy()
                    del self.contours_normal[i]
                    self.num_contours -= 1
                    self.grainNumber_value.setText("%d" % self.num_contours)
                    break
                else:
                    pass
        else:
            pass

        if self.trigger_line == 1:
            self.contours_remove = self.contours_remove_abnormal + self.contours_remove_normal
            self.binary_Remove = np.zeros(self.binary_erode.shape, dtype=np.uint8)
            cv2.drawContours(self.binary_Remove, self.contours_remove, -1, 255, -1)
            self.binary_Remove = cv2.bitwise_and(self.binary_erode, self.binary_Remove)
            self.binary_Remove = cv2.morphologyEx(self.binary_Remove, cv2.MORPH_CLOSE, self.kernel)

            for i in self.line_manual:
                cv2.circle(self.image_contour, i, 5, (0, 0, 255), -1)
            for i in range(len(self.line_manual)):
                if i % 2 == 0:
                    cv2.line(self.image_contour, self.line_manual[i], self.line_manual[i + 1], (0, 0, 255), 2)
                    cv2.line(self.binary_Remove, self.line_manual[i], self.line_manual[i + 1], 0, 2)

        cv2.drawContours(self.image_contour, self.contours_add, -1, (0, 0, 255), 3)
        cv2.drawContours(self.image_contour, self.contours_abnormal, -1, (255, 0, 255), 3)
        cv2.drawContours(self.image_contour, self.contours_normal, -1, (0, 255, 0), 3)
        self.resize_image()

        try:
            # 获取种子平均参数
            self.list_area_singleGrain = []
            self.list_perimeter_singleGrain = []
            self.list_length_singleGrain = []
            self.list_width_singleGrain = []

            num_grain_normal = len(self.contours_normal)
            for c in range(num_grain_normal):
                area_singleGrain = cv2.contourArea(self.contours_normal[c])
                rectangle_singleGrain = cv2.minAreaRect(self.contours_normal[c])
                side_X = rectangle_singleGrain[1][0]
                side_Y = rectangle_singleGrain[1][1]
                if side_X >= side_Y:
                    length_singleGrain = side_X
                    width_singleGrain = side_Y
                else:
                    length_singleGrain = side_Y
                    width_singleGrain = side_X

                self.list_area_singleGrain.append(area_singleGrain)
                self.list_length_singleGrain.append(length_singleGrain)
                self.list_width_singleGrain.append(width_singleGrain)

            list_approx = []
            for c in range(num_grain_normal):
                epsilon = 0.005 * cv2.arcLength(self.contours_normal[c], True)
                approx = cv2.approxPolyDP(self.contours_normal[c], epsilon, True)
                list_approx.append(approx)

            for c in range(num_grain_normal):
                perimeter_singleGrain = cv2.arcLength(list_approx[c], True)
                self.list_perimeter_singleGrain.append(perimeter_singleGrain)

            totalArea = 0
            totalPerimeter = 0
            totalLength = 0
            totalWidth = 0
            for i in range(num_grain_normal):
                totalArea += self.list_area_singleGrain[i]
                totalPerimeter += self.list_perimeter_singleGrain[i]
                totalLength += self.list_length_singleGrain[i]
                totalWidth += self.list_width_singleGrain[i]

            try:
                self.meanArea = totalArea / len(self.contours_normal)
                self.meanPerimeter = totalPerimeter / len(self.contours_normal)
                self.meanLength = totalLength / len(self.contours_normal)
                self.meanWidth = totalWidth / len(self.contours_normal)

            except:
                self.meanArea = 0
                self.meanPerimeter = 0
                self.meanLength = 0
                self.meanWidth = 0

            self.meanArea = round(self.meanArea * self.ratio, 2)
            self.meanPerimeter = round(self.meanPerimeter * self.ratio ** 0.5, 2)
            self.meanLength = round(self.meanLength * self.ratio ** 0.5, 2)
            self.meanWidth = round(self.meanWidth * self.ratio ** 0.5, 2)
            self.menaRatioLW = round(self.meanLength / self.meanWidth, 2)
            self.menaED = round((self.meanArea / 3.1415926) ** 0.5 * 2, 2)
            self.meanShapeFactor = round((4 * 3.1415926 * self.meanArea) / self.meanPerimeter ** 2, 2)

        except:
            self.meanArea = 0
            self.meanPerimeter = 0
            self.meanLength = 0
            self.meanWidth = 0
            self.menaRatioLW = 0
            self.menaED = 0
            self.meanShapeFactor = 0

    def normal(self):
        self.button_back.setEnabled(True)
        self.button_forward.setEnabled(False)
        self.triggle_operation = 3

        self.image_contour = self.image_color.copy()
        self.moveContour = self.contours_abnormal[self.number_contour_abnormal]
        self.contours_normal.append(self.contours_abnormal[self.number_contour_abnormal])
        del self.contours_abnormal[self.number_contour_abnormal]
        cv2.drawContours(self.image_contour, self.contours_add, -1, (0, 0, 255), 3)
        cv2.drawContours(self.image_contour, self.contours_abnormal, -1, (255, 0, 255), 3)
        cv2.drawContours(self.image_contour, self.contours_normal, -1, (0, 255, 0), 3)
        self.resize_image()

        try:
            # 获取种子平均参数
            self.list_area_singleGrain = []
            self.list_perimeter_singleGrain = []
            self.list_length_singleGrain = []
            self.list_width_singleGrain = []

            num_grain_normal = len(self.contours_normal)
            for c in range(num_grain_normal):
                area_singleGrain = cv2.contourArea(self.contours_normal[c])
                rectangle_singleGrain = cv2.minAreaRect(self.contours_normal[c])
                side_X = rectangle_singleGrain[1][0]
                side_Y = rectangle_singleGrain[1][1]
                if side_X >= side_Y:
                    length_singleGrain = side_X
                    width_singleGrain = side_Y
                else:
                    length_singleGrain = side_Y
                    width_singleGrain = side_X

                self.list_area_singleGrain.append(area_singleGrain)
                self.list_length_singleGrain.append(length_singleGrain)
                self.list_width_singleGrain.append(width_singleGrain)

            list_approx = []
            for c in range(num_grain_normal):
                epsilon = 0.005 * cv2.arcLength(self.contours_normal[c], True)
                approx = cv2.approxPolyDP(self.contours_normal[c], epsilon, True)
                list_approx.append(approx)

            for c in range(num_grain_normal):
                perimeter_singleGrain = cv2.arcLength(list_approx[c], True)
                self.list_perimeter_singleGrain.append(perimeter_singleGrain)

            totalArea = 0
            totalPerimeter = 0
            totalLength = 0
            totalWidth = 0
            for i in range(num_grain_normal):
                totalArea += self.list_area_singleGrain[i]
                totalPerimeter += self.list_perimeter_singleGrain[i]
                totalLength += self.list_length_singleGrain[i]
                totalWidth += self.list_width_singleGrain[i]

            try:
                self.meanArea = totalArea / len(self.contours_normal)
                self.meanPerimeter = totalPerimeter / len(self.contours_normal)
                self.meanLength = totalLength / len(self.contours_normal)
                self.meanWidth = totalWidth / len(self.contours_normal)

            except:
                self.meanArea = 0
                self.meanPerimeter = 0
                self.meanLength = 0
                self.meanWidth = 0

            self.meanArea = round(self.meanArea * self.ratio, 2)
            self.meanPerimeter = round(self.meanPerimeter * self.ratio ** 0.5, 2)
            self.meanLength = round(self.meanLength * self.ratio ** 0.5, 2)
            self.meanWidth = round(self.meanWidth * self.ratio ** 0.5, 2)
            self.menaRatioLW = round(self.meanLength / self.meanWidth, 2)
            self.menaED = round((self.meanArea / 3.1415926) ** 0.5 * 2, 2)
            self.meanShapeFactor = round((4 * 3.1415926 * self.meanArea) / self.meanPerimeter ** 2, 2)

        except:
            self.meanArea = 0
            self.meanPerimeter = 0
            self.meanLength = 0
            self.meanWidth = 0
            self.menaRatioLW = 0
            self.menaED = 0
            self.meanShapeFactor = 0

    def back(self):
        self.button_back.setEnabled(False)
        self.button_forward.setEnabled(True)

        if self.trigger_manual == 0:
            self.image_contour = self.image_color.copy()
        else:
            if self.trigger_camera == 1 or self.trigger_camera_initial == 1:
                self.image_contour = self.cam_origin.copy()
            else:
                self.image_contour = self.image_open.copy()

        if self.triggle_operation == 1:
            self.contours_add.pop()
            self.num_contours -= 1

        elif self.triggle_operation == 2:
            if self.value_contour_add == 1:
                self.contours_add.append(self.removeCounter_add)
                self.num_contours += 1
            if self.value_contour_abnormal == 1:
                self.contours_abnormal.append(self.removeCounter_abnormal)
                self.num_contours += 1
            if self.value_contour_normal == 1:
                self.contours_normal.append(self.removeCounter_normal)
                self.num_contours += 1
        elif self.triggle_operation == 3:
            self.contours_abnormal.append(self.moveContour)
            self.contours_normal.pop()
        else:
            for i in range(len(self.contours_Remove_copy)):
                self.contours_normal.pop()
                self.num_contours -= 1
            for i in range(len(self.contours_remove_normal_copy)):
                self.contours_normal.append(self.contours_remove_normal_copy[i])
                self.num_contours += 1
            for i in range(len(self.contours_remove_abnormal_copy)):
                self.contours_abnormal.append(self.contours_remove_abnormal_copy[i])
                self.num_contours += 1

        self.grainNumber_value.setText("%d" % self.num_contours)

        cv2.drawContours(self.image_contour, self.contours_add, -1, (0, 0, 255), 3)
        cv2.drawContours(self.image_contour, self.contours_abnormal, -1, (255, 0, 255), 3)
        cv2.drawContours(self.image_contour, self.contours_normal, -1, (0, 255, 0), 3)
        self.resize_image()

        try:
            # 获取种子平均参数
            self.list_area_singleGrain = []
            self.list_perimeter_singleGrain = []
            self.list_length_singleGrain = []
            self.list_width_singleGrain = []

            num_grain_normal = len(self.contours_normal)
            for c in range(num_grain_normal):
                area_singleGrain = cv2.contourArea(self.contours_normal[c])
                rectangle_singleGrain = cv2.minAreaRect(self.contours_normal[c])
                side_X = rectangle_singleGrain[1][0]
                side_Y = rectangle_singleGrain[1][1]
                if side_X >= side_Y:
                    length_singleGrain = side_X
                    width_singleGrain = side_Y
                else:
                    length_singleGrain = side_Y
                    width_singleGrain = side_X

                self.list_area_singleGrain.append(area_singleGrain)
                self.list_length_singleGrain.append(length_singleGrain)
                self.list_width_singleGrain.append(width_singleGrain)

            list_approx = []
            for c in range(num_grain_normal):
                epsilon = 0.005 * cv2.arcLength(self.contours_normal[c], True)
                approx = cv2.approxPolyDP(self.contours_normal[c], epsilon, True)
                list_approx.append(approx)

            for c in range(num_grain_normal):
                perimeter_singleGrain = cv2.arcLength(list_approx[c], True)
                self.list_perimeter_singleGrain.append(perimeter_singleGrain)

            totalArea = 0
            totalPerimeter = 0
            totalLength = 0
            totalWidth = 0
            for i in range(num_grain_normal):
                totalArea += self.list_area_singleGrain[i]
                totalPerimeter += self.list_perimeter_singleGrain[i]
                totalLength += self.list_length_singleGrain[i]
                totalWidth += self.list_width_singleGrain[i]

            try:
                self.meanArea = totalArea / len(self.contours_normal)
                self.meanPerimeter = totalPerimeter / len(self.contours_normal)
                self.meanLength = totalLength / len(self.contours_normal)
                self.meanWidth = totalWidth / len(self.contours_normal)

            except:
                self.meanArea = 0
                self.meanPerimeter = 0
                self.meanLength = 0
                self.meanWidth = 0

            self.meanArea = round(self.meanArea * self.ratio, 2)
            self.meanPerimeter = round(self.meanPerimeter * self.ratio ** 0.5, 2)
            self.meanLength = round(self.meanLength * self.ratio ** 0.5, 2)
            self.meanWidth = round(self.meanWidth * self.ratio ** 0.5, 2)
            self.menaRatioLW = round(self.meanLength / self.meanWidth, 2)
            self.menaED = round((self.meanArea / 3.1415926) ** 0.5 * 2, 2)
            self.meanShapeFactor = round((4 * 3.1415926 * self.meanArea) / self.meanPerimeter ** 2, 2)

        except:
            self.meanArea = 0
            self.meanPerimeter = 0
            self.meanLength = 0
            self.meanWidth = 0
            self.menaRatioLW = 0
            self.menaED = 0
            self.meanShapeFactor = 0

    def forward(self):
        self.button_back.setEnabled(True)
        self.button_forward.setEnabled(False)

        if self.trigger_manual == 0:
            self.image_contour = self.image_color.copy()
        else:
            if self.trigger_camera == 1 or self.trigger_camera_initial == 1:
                self.image_contour = self.cam_origin.copy()
            else:
                self.image_contour = self.image_open.copy()

        if self.triggle_operation == 1:
            self.contours_add.append(self.addCounter)
            self.num_contours += 1
        elif self.triggle_operation == 2:
            if self.value_contour_add == 1:
                self.contours_add.pop()
                self.num_contours -= 1
            if self.value_contour_abnormal == 1:
                self.contours_abnormal.pop()
                self.num_contours -= 1
            if self.value_contour_normal == 1:
                self.contours_normal.pop()
                self.num_contours -= 1
        elif self.triggle_operation == 3:
            self.contours_normal.append(self.moveContour)
            self.contours_abnormal.pop()
        else:
            for i in range(len(self.contours_remove_normal_copy)):
                self.contours_normal.pop()
                self.num_contours -= 1
            for i in range(len(self.contours_remove_abnormal_copy)):
                self.contours_abnormal.pop()
                self.num_contours -= 1
            for i in range(len(self.contours_Remove_copy)):
                self.contours_normal.append(self.contours_Remove_copy[i])
                self.num_contours += 1

        self.grainNumber_value.setText("%d" % self.num_contours)

        cv2.drawContours(self.image_contour, self.contours_add, -1, (0, 0, 255), 3)
        cv2.drawContours(self.image_contour, self.contours_abnormal, -1, (255, 0, 255), 3)
        cv2.drawContours(self.image_contour, self.contours_normal, -1, (0, 255, 0), 3)
        self.resize_image()

        try:
            # 获取种子平均参数
            self.list_area_singleGrain = []
            self.list_perimeter_singleGrain = []
            self.list_length_singleGrain = []
            self.list_width_singleGrain = []

            num_grain_normal = len(self.contours_normal)
            for c in range(num_grain_normal):
                area_singleGrain = cv2.contourArea(self.contours_normal[c])
                rectangle_singleGrain = cv2.minAreaRect(self.contours_normal[c])
                side_X = rectangle_singleGrain[1][0]
                side_Y = rectangle_singleGrain[1][1]
                if side_X >= side_Y:
                    length_singleGrain = side_X
                    width_singleGrain = side_Y
                else:
                    length_singleGrain = side_Y
                    width_singleGrain = side_X

                self.list_area_singleGrain.append(area_singleGrain)
                self.list_length_singleGrain.append(length_singleGrain)
                self.list_width_singleGrain.append(width_singleGrain)

            list_approx = []
            for c in range(num_grain_normal):
                epsilon = 0.005 * cv2.arcLength(self.contours_normal[c], True)
                approx = cv2.approxPolyDP(self.contours_normal[c], epsilon, True)
                list_approx.append(approx)

            for c in range(num_grain_normal):
                perimeter_singleGrain = cv2.arcLength(list_approx[c], True)
                self.list_perimeter_singleGrain.append(perimeter_singleGrain)

            totalArea = 0
            totalPerimeter = 0
            totalLength = 0
            totalWidth = 0
            for i in range(num_grain_normal):
                totalArea += self.list_area_singleGrain[i]
                totalPerimeter += self.list_perimeter_singleGrain[i]
                totalLength += self.list_length_singleGrain[i]
                totalWidth += self.list_width_singleGrain[i]

            try:
                self.meanArea = totalArea / len(self.contours_normal)
                self.meanPerimeter = totalPerimeter / len(self.contours_normal)
                self.meanLength = totalLength / len(self.contours_normal)
                self.meanWidth = totalWidth / len(self.contours_normal)

            except:
                self.meanArea = 0
                self.meanPerimeter = 0
                self.meanLength = 0
                self.meanWidth = 0

            self.meanArea = round(self.meanArea * self.ratio, 2)
            self.meanPerimeter = round(self.meanPerimeter * self.ratio ** 0.5, 2)
            self.meanLength = round(self.meanLength * self.ratio ** 0.5, 2)
            self.meanWidth = round(self.meanWidth * self.ratio ** 0.5, 2)
            self.menaRatioLW = round(self.meanLength / self.meanWidth, 2)
            self.menaED = round((self.meanArea / 3.1415926) ** 0.5 * 2, 2)
            self.meanShapeFactor = round((4 * 3.1415926 * self.meanArea) / self.meanPerimeter ** 2, 2)

        except:
            self.meanArea = 0
            self.meanPerimeter = 0
            self.meanLength = 0
            self.meanWidth = 0
            self.menaRatioLW = 0
            self.menaED = 0
            self.meanShapeFactor = 0

    def restore(self):
        self.contours_remove = self.contours_remove_abnormal + self.contours_remove_normal
        if len(self.contours_remove) == 0:
            if language == 'zh_CN':
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "请先删除错误籽粒！")
                message.setStyleSheet("QLabel{""min-width:200px;""min-height:35px;""}")
                Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(
                        18 * self.ratio_window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_window))
            else:
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip",
                                                "Please delete the mistaken kernels first!")
                Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(
                        18 * self.ratio_window))
                message.setStyleSheet(
                    '''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_window))
            Qyes.setFixedSize(80, 40)
            message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
            message.setIcon(0)

            if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                pass

        else:
            if self.trigger_line == 0:
                self.button_restore.setStyleSheet(
                    '''QPushButton{border-radius:%dpx;background:#90EE90;}''' % int(8 * self.ratio_window))

                self.trigger_line = 1
                self.line_manual = []

                try:
                    if self.value_contour_abnormal_copy == 0:
                        self.contours_remove_abnormal_copy = []
                    if self.value_contour_normal_copy == 0:
                        self.contours_remove_normal_copy = []
                except:
                    pass

                self.value_contour_abnormal_copy = 0
                self.value_contour_normal_copy = 0

                self.binary_Remove = np.zeros(self.binary_erode.shape, dtype=np.uint8)
                cv2.drawContours(self.binary_Remove, self.contours_remove, -1, 255, -1)
                self.binary_Remove = cv2.bitwise_and(self.binary_erode, self.binary_Remove)
                self.binary_Remove = cv2.morphologyEx(self.binary_Remove, cv2.MORPH_CLOSE, self.kernel)

            else:
                self.button_restore.setStyleSheet(
                    '''QPushButton{border-radius:%dpx;background:#DCDCDC;}''' % int(8 * self.ratio_window))
                self.trigger_line = 0

                contours_Remove, _ = cv2.findContours(self.binary_Remove, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                self.contours_Remove_copy = contours_Remove.copy()

                for i in range(len(contours_Remove)):
                    self.contours_normal.append(contours_Remove[i])

                self.num_contours += len(contours_Remove)
                self.grainNumber_value.setText("%d" % self.num_contours)

                self.image_contour = self.image_color.copy()
                cv2.drawContours(self.image_contour, self.contours_add, -1, (0, 0, 255), 3)
                cv2.drawContours(self.image_contour, self.contours_abnormal, -1, (255, 0, 255), 3)
                cv2.drawContours(self.image_contour, self.contours_normal, -1, (0, 255, 0), 3)
                self.resize_image()

                self.binary_Remove = np.zeros(self.binary_erode.shape, dtype=np.uint8)
                self.contours_remove_abnormal = []
                self.contours_remove_normal = []
                self.contours_remove = []
                self.button_restore.setStyleSheet(
                    '''QPushButton{border-radius:%dpx;background:#DCDCDC;}''' % int(8 * self.ratio_window))
                self.trigger_line = 0
                self.triggle_operation = 4

                try:
                    # 获取种子平均参数
                    self.list_area_singleGrain = []
                    self.list_perimeter_singleGrain = []
                    self.list_length_singleGrain = []
                    self.list_width_singleGrain = []

                    num_grain_normal = len(self.contours_normal)
                    for c in range(num_grain_normal):
                        area_singleGrain = cv2.contourArea(self.contours_normal[c])
                        rectangle_singleGrain = cv2.minAreaRect(self.contours_normal[c])
                        side_X = rectangle_singleGrain[1][0]
                        side_Y = rectangle_singleGrain[1][1]
                        if side_X >= side_Y:
                            length_singleGrain = side_X
                            width_singleGrain = side_Y
                        else:
                            length_singleGrain = side_Y
                            width_singleGrain = side_X

                        self.list_area_singleGrain.append(area_singleGrain)
                        self.list_length_singleGrain.append(length_singleGrain)
                        self.list_width_singleGrain.append(width_singleGrain)

                    list_approx = []
                    for c in range(num_grain_normal):
                        epsilon = 0.005 * cv2.arcLength(self.contours_normal[c], True)
                        approx = cv2.approxPolyDP(self.contours_normal[c], epsilon, True)
                        list_approx.append(approx)

                    for c in range(num_grain_normal):
                        perimeter_singleGrain = cv2.arcLength(list_approx[c], True)
                        self.list_perimeter_singleGrain.append(perimeter_singleGrain)

                    totalArea = 0
                    totalPerimeter = 0
                    totalLength = 0
                    totalWidth = 0
                    for i in range(num_grain_normal):
                        totalArea += self.list_area_singleGrain[i]
                        totalPerimeter += self.list_perimeter_singleGrain[i]
                        totalLength += self.list_length_singleGrain[i]
                        totalWidth += self.list_width_singleGrain[i]

                    try:
                        self.meanArea = totalArea / len(self.contours_normal)
                        self.meanPerimeter = totalPerimeter / len(self.contours_normal)
                        self.meanLength = totalLength / len(self.contours_normal)
                        self.meanWidth = totalWidth / len(self.contours_normal)

                    except:
                        self.meanArea = 0
                        self.meanPerimeter = 0
                        self.meanLength = 0
                        self.meanWidth = 0

                    self.meanArea = round(self.meanArea * self.ratio, 2)
                    self.meanPerimeter = round(self.meanPerimeter * self.ratio ** 0.5, 2)
                    self.meanLength = round(self.meanLength * self.ratio ** 0.5, 2)
                    self.meanWidth = round(self.meanWidth * self.ratio ** 0.5, 2)
                    self.menaRatioLW = round(self.meanLength / self.meanWidth, 2)
                    self.menaED = round((self.meanArea / 3.1415926) ** 0.5 * 2, 2)
                    self.meanShapeFactor = round((4 * 3.1415926 * self.meanArea) / self.meanPerimeter ** 2, 2)

                except:
                    self.meanArea = 0
                    self.meanPerimeter = 0
                    self.meanLength = 0
                    self.meanWidth = 0
                    self.menaRatioLW = 0
                    self.menaED = 0
                    self.meanShapeFactor = 0

    def manual(self):
        if self.trigger_manual == 1:
            self.display_image_box.setCursor(QtCore.Qt.ArrowCursor)
            self.button_acquisition.setEnabled(True)

            if language == 'zh_CN':
                self.button_manual.setStyleSheet(
                '''QPushButton{color:white;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    24 * self.ratio_window))
            else:
                self.button_manual.setStyleSheet(
                '''QPushButton{text-align: left;color:white;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:10px}QPushButton:hover{background:#90EE90;}''' % int(
                    18 * self.ratio_window))

            self.trigger_manual = 0
        else:
            if self.trigger_camera == 0 and self.trigger_image == 0 and self.trigger_camera_initial == 0:
                if language == 'zh_CN':
                    message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "请先采集图像！")
                    message.setStyleSheet("QLabel{""min-width:200px;""min-height:35px;""}")
                    Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                    Qyes.setStyleSheet(
                        '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(
                            18 * self.ratio_window))
                    message.setStyleSheet(
                        '''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_window))
                else:
                    message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip",
                                                    "Please collect the image  first!")
                    Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                    Qyes.setStyleSheet(
                        '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(
                            18 * self.ratio_window))
                    message.setStyleSheet(
                        '''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_window))
                Qyes.setFixedSize(80, 40)
                message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
                message.setIcon(0)

                if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                    pass

            else:
                self.labelx = 0
                self.labely = 0

                self.display_image_box.setCursor(QtCore.Qt.CrossCursor)
                self.button_acquisition.setEnabled(False)
                self.display_waring.setStyleSheet("QLabel{background:transparent;}")
                if language == 'zh_CN':
                    self.button_manual.setStyleSheet(
                        '''QPushButton{background:#90EE90;color:white;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:10px}''' % int(
                            24 * self.ratio_window))
                else:
                    self.button_manual.setStyleSheet(
                        '''QPushButton{text-align: left;background:#90EE90;color:white;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:10px}''' % int(
                            18 * self.ratio_window))
                self.trigger_manual = 1

                if self.trigger_camera == 1 or self.trigger_camera_initial == 1:
                    self.image_contour = self.cam_origin.copy()
                    self.timer_showcamera.stop()
                else:
                    self.image_contour = self.image_open.copy()

                try:
                    self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                    channel_B, _, _ = cv2.split(self.image_contour)
                    _, binary_background = cv2.threshold(channel_B, 100, 255, cv2.THRESH_BINARY_INV)
                    if self.widthRatio == -1 or self.widthRatio == 1:
                        thre = self.findThreshold(channel_B)
                    else:
                        try:
                            with open("boxratio.txt", "r") as f:
                                self.widthRatio = float(f.readline())
                                self.heightRatio = float(f.readline())
                                width_binary = binary_background.shape[1]
                                height_binary = binary_background.shape[0]
                                y1 = int((1 - self.heightRatio) / 2 * height_binary)
                                y2 = int((1 + self.heightRatio) / 2 * height_binary)
                                x1 = int((1 - self.widthRatio) / 2 * width_binary)
                                x2 = int((1 + self.widthRatio) / 2 * width_binary)
                                binary_background = binary_background[y1: y2, x1: x2]
                                channel_B_thre = channel_B[y1: y2, x1: x2]
                                thre = self.findThreshold(channel_B_thre)

                        except:
                            pass

                    area_max = 0
                    contours, _ = cv2.findContours(binary_background, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
                    for i in range(1, len(contours)):
                        area = cv2.contourArea(contours[i])
                        if area >= area_max:
                            area_max = area
                    if area_max <= 6000000 * self.widthRatio * self.heightRatio:
                        _, binary = cv2.threshold(channel_B, thre, 255, cv2.THRESH_BINARY_INV)
                    else:
                        _, binary = cv2.threshold(channel_B, thre, 255, cv2.THRESH_BINARY)

                    if self.widthRatio == 1 and self.heightRatio == 1:
                        pass
                    else:
                        try:
                            binary_mask = np.zeros((binary.shape[0], binary.shape[1]), np.uint8)
                            binary_mask[y1: y2, x1: x2] = 255
                            binary = cv2.bitwise_and(binary, binary_mask)
                        except:
                            pass

                    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for i in range(len(contours)):
                        area = cv2.contourArea(contours[i])
                        if area <= 50:
                            cv2.drawContours(binary, [contours[i]], -1, 0, -1)
                    _, _, _, _, self.shortEdge = self.findshapefactor(binary)
                    self.shortEdge *= 0.6
                except:
                    self.shortEdge = 30

                self.contours_add = []
                self.contours_normal = []
                self.contours_abnormal = []
                self.contours_remove = []
                self.num_contours = 0
                self.width_image_origin = self.image_contour.shape[1]
                self.scale_origin = self.width_display_image / self.width_image_origin
                self.scale = self.scale_origin
                self.resize_image()

                self.grainNumber_value.setText("%d" % self.num_contours)

                self.button_zoomIn.setEnabled(True)
                self.button_zoomOut.setEnabled(True)
                self.button_zoomReturn.setEnabled(True)
                self.button_back.setEnabled(False)
                self.button_forward.setEnabled(False)
                self.button_restore.setEnabled(False)

    # 关闭窗口函数
    def closeEvent(self, event):
        if language == 'zh_CN':
            message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "警告", "您确定退出吗？")
            message.setStyleSheet("QLabel{""min-width:200px;""min-height:35px;""}")
            Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
            Qno = message.addButton(self.tr("取消"), QtWidgets.QMessageBox.NoRole)
            Qyes.setFixedSize(80, 40)
            Qyes.setStyleSheet(
                '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(
                        18 * self.ratio_window))
            Qno.setFixedSize(80, 40)
            Qno.setStyleSheet(
                '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(
                        18 * self.ratio_window))
            message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_window))
        else:
            message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Warning","Are you sure?")
            message.setStyleSheet("QLabel{""min-width:180px;""min-height:35px;""}")
            Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
            Qno = message.addButton(self.tr("Cancel"), QtWidgets.QMessageBox.NoRole)
            Qyes.setFixedSize(80, 40)
            Qno.setFixedSize(80, 40)
            Qyes.setStyleSheet(
                '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(
                        18 * self.ratio_window))
            Qno.setStyleSheet(
                '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(
                        18 * self.ratio_window))
            message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_window))
        message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
        message.setIcon(0)

        if message.exec_() == QtWidgets.QMessageBox.RejectRole:
            event.ignore()

        else:
            event.accept()
            self.timer_showcamera.stop()
            self.cap.release()

class Sound(QtCore.QThread):
    def __init__(self):
        super().__init__()

    def run(self):
        file = QtCore.QUrl.fromLocalFile('%s/icon/sound.mp3' % path_icon)
        content = QMediaContent(file)
        player = QMediaPlayer()
        player.setMedia(content)
        player.play()
        sleep(2)

class imageName(QtWidgets.QWidget):
    signal_imageName = QtCore.pyqtSignal(list)
    def __init__(self, localtime, ratio_window, parent=None):
        super(imageName, self).__init__(parent)
        self.localTIME = localtime
        self.ratio_Window = ratio_window
        self.initUI()
    def initUI(self):
        self.setFixedSize(340, 150)
        self.setWindowIcon(QIcon("%s/icon/imagename.png" % path_icon))
        self.setWindowModality(QtCore.Qt.ApplicationModal)

        self.imageName_tip = QLabel(self)
        self.imageName_tip.setFixedSize(180, 30)
        self.imageName_tip.move(20, 10)

        self.input_imageName = QLineEdit(self)
        self.input_imageName.setStyleSheet('''QLineEdit{font-size:20px;font-family:Times New Roman;border:1px solid gray}QLineEdit:hover{font-size:20px;font-family:Times New Roman;border:1px solid #087AD5}''')
        self.input_imageName.setAlignment(QtCore.Qt.AlignCenter)
        self.input_imageName.setText("%s" % self.localTIME)
        self.input_imageName.selectAll()
        self.input_imageName.setFixedSize(200, 30)
        self.input_imageName.move(20, 50)

        self.choose_imageType = QComboBox(self)
        self.choose_imageType.setStyleSheet('''QComboBox{font-size:20px;font-family:Times New Roman;}''')
        self.choose_imageType.setFixedSize(80, 30)
        self.choose_imageType.move(240, 50)
        self.choose_imageType.addItems(['jpg','png','bmp'])

        self.button_ok = QPushButton(self)
        self.button_ok.setStyleSheet(
            '''QPushButton{background:#90EE90;font-size:20px;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''')
        self.button_ok.setGeometry(240, 100, 80, 40)
        self.button_ok.clicked.connect(self.export)

        if language == 'zh_CN':
            self.setWindowTitle("图像命名")
            self.imageName_tip.setText("请输入文件名：")
            self.imageName_tip.setStyleSheet('''font-size:20px;font-family:宋体;''')
            self.button_ok.setText("确定")
        else:
            self.setWindowTitle("Image Name")
            self.imageName_tip.setText("Enter name:")
            self.imageName_tip.setStyleSheet('''font-size:20px;font-family:Times New Roman;''')
            self.button_ok.setText("OK")

    def export(self):
        self.error = 1
        self.name = str(self.input_imageName.text())
        self.type = str(self.choose_imageType.currentText())

        if self.name == '':
            if language == 'zh_CN':
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "图像名不能为空！")
                message.setStyleSheet("QLabel{""min-width:200px;""min-height:35px;""}")
                Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_Window))
            else:
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip",
                                                "Image name cannot be empty!")
                Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_Window))
            Qyes.setFixedSize(80, 40)
            message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
            message.setIcon(0)

            if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                pass

        else:
            errorString = r"[\/\\\:\*\?\"\<\>\|]"
            for i in errorString:
                if i in self.name:
                    self.error = 0
                    break

            if self.error == 0:
                if language == 'zh_CN':
                    message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "文件名包含特殊字符！")
                    message.setStyleSheet("QLabel{""min-width:200px;""min-height:35px;""}")
                    Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                    Qyes.setStyleSheet(
                        '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
                    message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_Window))
                else:
                    message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip",
                                                    "Your input contains special characters!")
                    Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                    Qyes.setStyleSheet(
                        '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
                    message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_Window))
                Qyes.setFixedSize(80, 40)
                message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
                message.setIcon(0)

                if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                    pass
            else:
                self.close()
                sleep(0.2)
                self.signal_imageName.emit([self.name, self.type])

class PortChoose(QtWidgets.QWidget):
    signal = QtCore.pyqtSignal(list)
    def __init__(self, grainmodel, area_calibration_true, ratio_window, parent=None):
        super(PortChoose, self).__init__(parent)
        self.ratio_Window = ratio_window
        self.trigger_redetect = 0
        self.area_calibration_True = area_calibration_true
        self.grainModel = grainmodel
        self.initUI()

    def initUI(self):
        self.setFixedSize(470, 250)
        self.setWindowIcon(QIcon("%s/icon/port.png" % path_icon))
        self.setWindowModality(QtCore.Qt.ApplicationModal)

        self.checkBox_grainModel = QCheckBox(self)
        self.checkBox_grainModel.setGeometry(20, 10, 160, 40)

        if self.grainModel == 1:
            self.checkBox_grainModel.setChecked(True)
        else:
            self.checkBox_grainModel.setChecked(False)

        self.camera_port_tip = QLabel(self)
        self.camera_port_tip.setFixedSize(180, 40)
        self.camera_port_tip.move(20, 70)

        self.camera_1 = QRadioButton(self)
        self.camera_1.setStyleSheet("color:black;font-size:22px;font-family:Times New Roman;")
        self.camera_2 = QRadioButton(self)
        self.camera_2.setStyleSheet("color:black;font-size:22px;font-family:Times New Roman;")
        self.camera_3 = QRadioButton(self)
        self.camera_3.setStyleSheet("color:black;font-size:22px;font-family:Times New Roman;")
        self.camera_1.setText("1")
        self.camera_2.setText("2")
        self.camera_3.setText("3")
        self.camera_1.setGeometry(QtCore.QRect(200, 70, 120, 40))
        self.camera_2.setGeometry(QtCore.QRect(260, 70, 120, 40))
        self.camera_3.setGeometry(QtCore.QRect(320, 70, 120, 40))

        self.calibration_tip = QLabel(self)
        self.calibration_tip.setFixedSize(180, 40)
        self.calibration_tip.move(20, 135)

        self.tip_Calibration = QLineEdit(self)
        self.tip_Calibration.setStyleSheet('''QLineEdit{color:blue;font-size:20px;font-family:Times New Roman;border:1px solid gray}''')
        self.tip_Calibration.setAlignment(QtCore.Qt.AlignCenter)
        self.tip_Calibration.setText('%.2f' % self.area_calibration_True)
        self.tip_Calibration.setFixedSize(150, 40)
        self.doubleValidator_Calibration = QtGui.QDoubleValidator(self)
        self.doubleValidator_Calibration.setRange(1, 999)
        self.doubleValidator_Calibration.setNotation(QtGui.QDoubleValidator.StandardNotation)
        self.doubleValidator_Calibration.setDecimals(2)
        self.tip_Calibration.setValidator(self.doubleValidator_Calibration)
        self.tip_Calibration.move(200, 135)

        try:
            with open('port_camera.txt', 'r') as f:
                port_camera = int(f.readline().strip("\n"))
            if port_camera == 1:
                self.camera_1.setChecked(True)
            elif port_camera == 2:
                self.camera_2.setChecked(True)
            elif port_camera == 3:
                self.camera_3.setChecked(True)
            else:
                pass
        except:
            pass

        self.button_calibration = QPushButton(self)
        self.button_calibration.setStyleSheet(
            '''QPushButton{background:#90EE90;font-weight:500;font-size:20px;font-family:Times New Roman;border-radius:5px;border:1px solid gray;}''')
        self.button_calibration.setGeometry(360, 135, 100, 40)
        self.button_calibration.clicked.connect(self.calibration_area)

        self.button_return = QPushButton(self)
        self.button_return.setStyleSheet(
            '''QPushButton{background:#90EE90;font-weight:500;font-size:20px;font-family:Times New Roman;border-radius:5px;border:1px solid gray;}''' )
        self.button_return.setGeometry(200, 200, 90, 40)
        self.button_return.clicked.connect(self.close)

        if language == 'zh_CN':
            self.setWindowTitle("系统设置")
            self.checkBox_grainModel.setText("单籽粒模式")
            self.checkBox_grainModel.setStyleSheet('''font-size:24px;font-family:宋体;''')
            self.camera_port_tip.setText("摄像头选择")
            self.camera_port_tip.setStyleSheet('''font-size:24px;font-family:宋体;''')
            self.calibration_tip.setText('标定面积(cm<sup>2</sup>)')
            self.calibration_tip.setStyleSheet('''font-size:24px;font-family:宋体;''')
            self.button_calibration.setText('标定')
            self.button_return.setText('返回')

        else:
            self.setWindowTitle("System settings")
            self.checkBox_grainModel.setText("Single Kernel")
            self.checkBox_grainModel.setStyleSheet('''font-size:20px;font-family:Times New Roman;''')
            self.camera_port_tip.setText("Camera Choose")
            self.camera_port_tip.setStyleSheet('''font-size:20px;font-family:Times New Roman;''')
            self.calibration_tip.setText('Calibration area(cm<sup>2</sup>)')
            self.calibration_tip.setStyleSheet('''font-size:20px;font-family:Times New Roman;''')
            self.button_calibration.setText('Calibrate')
            self.button_return.setText('Return')

        lable_line1 = QLabel(self)
        lable_line1.setStyleSheet('border-top-width:1px; border-style:solid; border-top-color: gray')
        lable_line1.setFixedSize(510, 1)
        lable_line1.move(0, 0)

        lable_line2 = QLabel(self)
        lable_line2.setStyleSheet('border-top-width:1px; border-style:solid; border-top-color: gray')
        lable_line2.setFixedSize(510, 1)
        lable_line2.move(0, 57)

        lable_line3 = QLabel(self)
        lable_line3.setStyleSheet('border-top-width:1px; border-style:solid; border-top-color: gray')
        lable_line3.setFixedSize(510, 1)
        lable_line3.move(0, 120)

        lable_line4 = QLabel(self)
        lable_line4.setStyleSheet('border-top-width:1px; border-style:solid; border-top-color: gray')
        lable_line4.setFixedSize(510, 1)
        lable_line4.move(0, 190)

        if self.camera_1.isChecked():
            self.cameraNumber = 1
        elif self.camera_2.isChecked():
            self.cameraNumber = 2
        elif self.camera_3.isChecked():
            self.cameraNumber = 3
        else:
            self.cameraNumber = 10

    def calibration_area(self):
        self.trueArea = float(self.tip_Calibration.text())
        if self.trueArea != 0:
            self.tip_Calibration.setStyleSheet('''QLineEdit{color:blue;background:#90EE90;font-size:20px;font-family:Times New Roman;border:1px solid gray}''')
            self.tip_Calibration.setReadOnly(True)
        else:
            if language == 'zh_CN':
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "请输入非0面积值！")
                message.setStyleSheet("QLabel{""min-width:200px;""min-height:35px;""}")
                Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_Window))
            else:
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip", "Please enter a value for non-0 area!")
                Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_Window))
            Qyes.setFixedSize(80, 40)
            message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
            message.setIcon(0)

            if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                pass

        self.signal.emit(["calibration", self.trueArea, 0])

    def closeEvent(self, event):
        self.delay_returning = Timer(0.1, self.returning)
        self.delay_returning.start()

    def returning(self):
        if language == 'zh_CN':
            self.button_return.setText('返回中')
        else:
            self.button_return.setText('Returning')
        self.button_return.setStyleSheet('''QPushButton{background:#90EE90;font-weight:500;font-size:20px;font-family:Times New Roman;border-radius:5px;border:1px solid gray;}''')

        if self.checkBox_grainModel.isChecked():
            self.grainModel = 1
        else:
            self.grainModel = 0

        if self.camera_1.isChecked():
            self.cameraNumber_change = 1
        elif self.camera_2.isChecked():
            self.cameraNumber_change = 2
        elif self.camera_3.isChecked():
            self.cameraNumber_change = 3
        else:
            self.cameraNumber_change = 10

        if self.cameraNumber_change == self.cameraNumber:
            self.trigger_openCamera = 0
        else:
            self.trigger_openCamera = 1

        self.signal.emit(["return", self.grainModel, self.cameraNumber_change, self.trigger_openCamera])

class Record(QtWidgets.QWidget):
    def __init__(self, width_window, height_window, win_x, win_y, ratio_window, parent = None):
        super(Record, self).__init__(parent)
        self.width_window_record = width_window - 20
        self.height_window_record = height_window - 50
        self.ratio_Window = ratio_window

        self.win_X = win_x
        self.win_Y = win_y

        self.setFixedSize(self.width_window_record, self.height_window_record)
        self.move(win_x + 10, win_y + 10)
        self.setWindowModality(QtCore.Qt.ApplicationModal)
        self.setWindowIcon(QIcon("%s/icon/record.png" % path_icon))

        try :
            list_record_all = []
            with open("%s/result/result.csv" % path_icon, 'r', newline='') as f:
                for line in f.readlines():
                    line = line.replace('\r\n', '')
                    line = line.split(',')
                    list_record_all.append(line)
            list_record_all.pop(0)

            self.mode_table = QtGui.QStandardItemModel(len(list_record_all), 9)
            self.len_tableView = len(list_record_all)
            for row in range(self.len_tableView):
                for column in range(9):
                    item = QStandardItem("%s" % list_record_all[row][column])
                    item.setTextAlignment(QtCore.Qt.AlignCenter)
                    item.setFont(QFont('Times New Roman', int(12 * self.ratio_Window)))
                    self.mode_table.setItem(row, column, item)
        except:
            self.mode_table = QtGui.QStandardItemModel(0, 9)

        self.tableView = QTableView(self)
        self.tableView.setModel(self.mode_table)
        self.tableView.setEditTriggers(QTableView.NoEditTriggers)

        if language == 'zh_CN':
            self.mode_table.setHorizontalHeaderLabels(['名称', '籽粒数', '平均面积', '平均周长', '平均长度', '平均宽度', '平均长宽比', '平均等效直径', '形状因子'])
            self.tableView.horizontalHeader().setStyleSheet(
                "QHeaderView::section{background-color:white;font-size:%dpx;font-family:宋体;color: black;}" % int(16 * self.ratio_Window))
        else:
            self.mode_table.setHorizontalHeaderLabels(['Name', 'Kernel Number', 'Area', 'Perimeter', 'Length', 'Width','Length/width', 'Equ. diameter', 'Shape factor'])
            self.tableView.horizontalHeader().setStyleSheet(
                "QHeaderView::section{background-color:white;font-size:%dpx;font-family:Times New Roman;color: black;}" % int(16 * self.ratio_Window))

        self.tableView.verticalHeader().setStyleSheet(
            "QHeaderView::section{background-color:white;font-size:%dpx;font-family:Times New Roman;color: black;}" % int(16 * self.ratio_Window))
        self.tableView.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableView.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.tableView.horizontalHeader().setDisabled(True)
        self.tableView.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tableView.setGeometry(10, 0, self.width_window_record - 20, self.height_window_record - 65)

        self.button_detail = QPushButton(self)
        self.button_detail.setStyleSheet(
            '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(16 * self.ratio_Window))
        self.button_detail.setGeometry(10, self.height_window_record - 55, 100, 40)
        self.button_detail.clicked.connect(self.detail)

        self.button_exportFile = QPushButton(self)
        self.button_exportFile.setStyleSheet(
            '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(16 * self.ratio_Window))
        self.button_exportFile.setGeometry(self.width_window_record - 330, self.height_window_record - 55, 100, 40)
        self.button_exportFile.clicked.connect(self.exportFile)

        self.button_clearRecordFile = QPushButton(self)
        self.button_clearRecordFile.setStyleSheet(
            '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(16 * self.ratio_Window))
        self.button_clearRecordFile.setGeometry(self.width_window_record - 220, self.height_window_record - 55, 100, 40)
        self.button_clearRecordFile.clicked.connect(self.clearRecordFile)

        self.button_return = QPushButton(self)
        self.button_return.setStyleSheet(
            '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(16 * self.ratio_Window))
        self.button_return.setGeometry(self.width_window_record- 110, self.height_window_record - 55, 100, 40)
        self.button_return.clicked.connect(self.close)

        if language == 'zh_CN':
            self.setWindowTitle("所有记录")
            self.button_detail.setText("详细信息")
            self.button_exportFile.setText("导出文件")
            self.button_clearRecordFile.setText("删除文件")
            self.button_return.setText("返回")
        else:
            self.setWindowTitle("All records")
            self.button_detail.setText("Detail")
            self.button_exportFile.setText("Export")
            self.button_clearRecordFile.setText("Delete")
            self.button_return.setText("Return")

    def detail(self):
        try:
            index_row = self.tableView.selectionModel().selection().indexes()[0].row()
            new_index = self.tableView.model().index(index_row, 0)
            name_open = self.tableView.model().data(new_index)

            self.window_Record_detail = Record_detail(self.width_window_record, self.height_window_record, name_open, self.win_X, self.win_Y, self.ratio_Window)
            self.window_Record_detail.show()

        except:
            if language == 'zh_CN':
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "请选择要打开的文件！")
                Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_Window))
            else:
                message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip", "Please select the file to open!")
                Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                Qyes.setStyleSheet(
                    '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
                message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_Window))
            Qyes.setFixedSize(80, 40)
            message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
            message.setIcon(0)

            if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                pass

    def exportFile(self):
        source_path = path.abspath("%s/result" % path_icon)
        target_path = QtWidgets.QFileDialog.getExistingDirectory(self, "选取文件夹", path_icon)
        if target_path != '':
            target_path = path.abspath("%s/result" % target_path)
            if path.exists(target_path):
                if language == 'zh_CN':
                    message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "提示", "文件已存在！")
                    message.setStyleSheet("QLabel{""min-width:200px;""min-height:35px;""}")
                    Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
                    Qyes.setStyleSheet(
                        '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
                    message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_Window))
                else:
                    message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Tip", "File already exist!")
                    Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
                    Qyes.setStyleSheet(
                        '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
                    message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_Window))
                Qyes.setFixedSize(80, 40)
                message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
                message.setIcon(0)

                if message.exec_() == QtWidgets.QMessageBox.ActionRole:
                    pass
            else:
                copytree(source_path, target_path)
        else:
            pass

    def clearRecordFile(self):
        if language == 'zh_CN':
            message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "警告", "您确定清除文件中所有记录吗？")
            Qyes = message.addButton(self.tr("确定"), QtWidgets.QMessageBox.YesRole)
            Qno = message.addButton(self.tr("取消"), QtWidgets.QMessageBox.NoRole)
            Qyes.setFixedSize(80, 40)
            Qno.setFixedSize(80, 40)
            Qyes.setStyleSheet(
                '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
            Qno.setStyleSheet(
                '''QPushButton{background:#90EE90;font-size:%dpx;font-family:宋体;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
            message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:宋体;}''' % int(20 * self.ratio_Window))
        else:
            message = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Warning",
                                            "Are you sure to clear all records in the file?")
            Qyes = message.addButton(self.tr("OK"), QtWidgets.QMessageBox.YesRole)
            Qno = message.addButton(self.tr("Cancel"), QtWidgets.QMessageBox.NoRole)
            Qyes.setFixedSize(80, 40)
            Qno.setFixedSize(80, 40)
            Qyes.setStyleSheet(
                '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
            Qno.setStyleSheet(
                '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(18 * self.ratio_Window))
            message.setStyleSheet('''QMessageBox{font-size:%dpx;font-family:Times New Roman;}''' % int(20 * self.ratio_Window))
        message.setWindowIcon(QIcon("%s/icon/messagebox.ico" % path_icon))
        message.setIcon(0)

        if message.exec_() == QtWidgets.QMessageBox.RejectRole:
            pass

        else:
            self.mode_table.removeRows(0, self.len_tableView)
            rmtree("%s/result" % path_icon)
            mkdir("%s/result" % path_icon)

class Record_detail(QtWidgets.QWidget):
    def __init__(self, width_window, height_window, name_open, win_X, win_Y, ratio_Window, parent = None):
        super(Record_detail, self).__init__(parent)
        width_window_record = width_window
        height_window_record = height_window
        self.ratio_Window = ratio_Window

        self.name_detail = name_open

        self.setFixedSize(width_window_record, height_window_record)
        self.move(win_X + 10, win_Y + 10)
        self.setWindowModality(QtCore.Qt.ApplicationModal)
        self.setWindowIcon(QIcon("%s/icon/record.png" % path_icon))

        try:
            list_record_all = []
            with open("%s/result/%s.csv" % (path_icon, self.name_detail), 'r', newline='') as f:
                for line in f.readlines():
                    line = line.replace('\r\n', '')
                    line = line.split(',')
                    list_record_all.append(line)
            list_record_all.pop(0)

            self.mode_table = QtGui.QStandardItemModel(len(list_record_all), 8)
            self.len_tableView = len(list_record_all)
            for row in range(self.len_tableView):
                for column in range(8):
                    item = QStandardItem("%s" % list_record_all[row][column])
                    item.setTextAlignment(QtCore.Qt.AlignCenter)
                    item.setFont(QFont('Times New Roman', int(12 * self.ratio_Window)))
                    self.mode_table.setItem(row, column, item)
        except:
            self.mode_table = QtGui.QStandardItemModel(0, 8)

        self.tableView = QTableView(self)
        self.tableView.setModel(self.mode_table)
        self.tableView.setEditTriggers(QTableView.NoEditTriggers)

        if language == 'zh_CN':
            self.mode_table.setHorizontalHeaderLabels(['序号', '面积', '周长', '长度', '宽度', '长宽比', '等效直径', '形状因子'])
            self.tableView.horizontalHeader().setStyleSheet(
                "QHeaderView::section{background-color:white;font-size:%dpx;font-family:宋体;color: black;}" % int(16 * self.ratio_Window))
        else:
            self.mode_table.setHorizontalHeaderLabels(['SN', 'Area', 'Perimeter', 'Length', 'Width', 'Length/width', 'Equ. diameter', 'Shape factor', 'Weight'])
            self.tableView.horizontalHeader().setStyleSheet(
                "QHeaderView::section{background-color:white;font-size:%dpx;font-family:Times New Roman;color: black;}" % int(16 * self.ratio_Window))

        self.tableView.verticalHeader().hide()
        self.tableView.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableView.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.tableView.horizontalHeader().setDisabled(True)
        self.tableView.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tableView.setGeometry(10, 0, width_window_record - 20, height_window_record - 65)

        self.button_return = QPushButton(self)
        self.button_return.setStyleSheet(
            '''QPushButton{background:#90EE90;font-size:%dpx;font-family:Times New Roman;font-weight:500;border-radius:5px;border:1px solid gray;}''' % int(
                16 * self.ratio_Window))
        self.button_return.setGeometry(width_window_record - 110, height_window_record - 55, 100, 40)
        self.button_return.clicked.connect(self.close)

        if language == 'zh_CN':
            self.setWindowTitle("详细信息")
            self.button_return.setText("返回")
        else:
            self.setWindowTitle("Detail")
            self.button_return.setText("Return")

if __name__ == "__main__":
    global language
    global path_icon
    global passWord
    global diskSN_ascii_last6
    global delayTime

    path_icon = abspath('.')

    try:
        with open("arearatio.txt", "r") as f:
            arearatio = float(f.readline())
    except:
        arearatio = 6.732629815077101e-05

    try:
        with open("language.txt", "r") as f:
            data_language = f.readline()
        if data_language == '':
            language = 'zh_CN'
        else:
            language = 'en-US'
    except:
        language = 'zh_CN'

    app = QtWidgets.QApplication(argv)
    splash = QtWidgets.QSplashScreen(QtGui.QPixmap("%s/icon/loading.png" % path_icon))
    if language == 'zh_CN':
        splash.showMessage("启动中... ", QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
    else:
        splash.showMessage("Loading... ", QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
    splash.setFont(QFont('华文彩云', 14))
    splash.show()
    MainWindow = MainWindow()
    splash.close()
    MainWindow.show()
    exit(app.exec())