# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1370, 750)
        self.widget = QtWidgets.QWidget(Form)
        self.widget.setGeometry(QtCore.QRect(0, 0, 1370, 750))
        self.widget.setMaximumSize(QtCore.QSize(1370, 750))
        self.widget.setStyleSheet("background-color: #EBEBEB;")
        self.widget.setObjectName("widget")
        self.widget_8 = QtWidgets.QWidget(self.widget)
        self.widget_8.setGeometry(QtCore.QRect(610, 500, 71, 140))
        self.widget_8.setStyleSheet("background-color: rgb(255, 255, 255,);\n"
"border-radius:6px;")
        self.widget_8.setObjectName("widget_8")
        self.pushButton_delete = QtWidgets.QPushButton(self.widget_8)
        self.pushButton_delete.setGeometry(QtCore.QRect(10, 32, 50, 23))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_delete.setFont(font)
        self.pushButton_delete.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(0, 0, 0);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}")
        self.pushButton_delete.setObjectName("pushButton_delete")
        self.pushButton_add = QtWidgets.QPushButton(self.widget_8)
        self.pushButton_add.setGeometry(QtCore.QRect(10, 5, 50, 23))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_add.setFont(font)
        self.pushButton_add.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(0, 0, 0);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}")
        self.pushButton_add.setObjectName("pushButton_add")
        self.pushButton_open = QtWidgets.QPushButton(self.widget_8)
        self.pushButton_open.setGeometry(QtCore.QRect(10, 86, 50, 23))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_open.setFont(font)
        self.pushButton_open.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(0, 0, 0);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}")
        self.pushButton_open.setObjectName("pushButton_open")
        self.pushButton_save = QtWidgets.QPushButton(self.widget_8)
        self.pushButton_save.setGeometry(QtCore.QRect(10, 113, 50, 23))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_save.setFont(font)
        self.pushButton_save.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(0, 0, 0);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}")
        self.pushButton_save.setObjectName("pushButton_save")
        self.pushButton_update = QtWidgets.QPushButton(self.widget_8)
        self.pushButton_update.setGeometry(QtCore.QRect(10, 59, 50, 23))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_update.setFont(font)
        self.pushButton_update.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(0, 0, 0);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}")
        self.pushButton_update.setObjectName("pushButton_update")
        self.widget_12 = QtWidgets.QWidget(self.widget)
        self.widget_12.setGeometry(QtCore.QRect(10, 5, 671, 490))
        self.widget_12.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border-radius: 6px;")
        self.widget_12.setObjectName("widget_12")
        self.label_display = QtWidgets.QLabel(self.widget_12)
        self.label_display.setGeometry(QtCore.QRect(15, 5, 640, 480))
        self.label_display.setStyleSheet("background-color: rgb(80, 80, 80);")
        self.label_display.setText("")
        self.label_display.setAlignment(QtCore.Qt.AlignCenter)
        self.label_display.setObjectName("label_display")
        self.widget_2 = QtWidgets.QWidget(self.widget)
        self.widget_2.setEnabled(True)
        self.widget_2.setGeometry(QtCore.QRect(690, 5, 671, 421))
        self.widget_2.setStyleSheet("background-color: rgb(255, 255, 255,);\n"
"border-radius:6px;")
        self.widget_2.setObjectName("widget_2")
        self.label_servo1 = QtWidgets.QLabel(self.widget_2)
        self.label_servo1.setGeometry(QtCore.QRect(330, 10, 67, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_servo1.setFont(font)
        self.label_servo1.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_servo1.setAlignment(QtCore.Qt.AlignCenter)
        self.label_servo1.setObjectName("label_servo1")
        self.horizontalSlider_servo1 = QtWidgets.QSlider(self.widget_2)
        self.horizontalSlider_servo1.setGeometry(QtCore.QRect(70, 30, 581, 31))
        self.horizontalSlider_servo1.setStyleSheet("QSlider{\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::groove:horizontal {\n"
"background: transparent;\n"
"height:15px;\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 28px;\n"
"height:28px;\n"
"margin-top: -7px;\n"
"margin-bottom: -7px;\n"
"border-radius:14px;\n"
"background: rgba(240, 240, 240);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: #575757;\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF9600;\n"
"}")
        self.horizontalSlider_servo1.setMinimum(0)
        self.horizontalSlider_servo1.setMaximum(700)
        self.horizontalSlider_servo1.setPageStep(1)
        self.horizontalSlider_servo1.setProperty("value", 500)
        self.horizontalSlider_servo1.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_servo1.setObjectName("horizontalSlider_servo1")
        self.horizontalSlider_servo2 = QtWidgets.QSlider(self.widget_2)
        self.horizontalSlider_servo2.setGeometry(QtCore.QRect(70, 100, 581, 31))
        self.horizontalSlider_servo2.setStyleSheet("QSlider{\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::groove:horizontal {\n"
"background: transparent;\n"
"height:15px;\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 28px;\n"
"height:28px;\n"
"margin-top: -7px;\n"
"margin-bottom: -7px;\n"
"border-radius:14px;\n"
"background: rgba(240, 240, 240);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: #575757;\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF9600;\n"
"}")
        self.horizontalSlider_servo2.setMinimum(0)
        self.horizontalSlider_servo2.setMaximum(1000)
        self.horizontalSlider_servo2.setPageStep(1)
        self.horizontalSlider_servo2.setProperty("value", 500)
        self.horizontalSlider_servo2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_servo2.setObjectName("horizontalSlider_servo2")
        self.label_servo2 = QtWidgets.QLabel(self.widget_2)
        self.label_servo2.setGeometry(QtCore.QRect(330, 80, 67, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_servo2.setFont(font)
        self.label_servo2.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_servo2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_servo2.setObjectName("label_servo2")
        self.label_8 = QtWidgets.QLabel(self.widget_2)
        self.label_8.setGeometry(QtCore.QRect(20, 37, 30, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_8.setFont(font)
        self.label_8.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_8.setObjectName("label_8")
        self.label_9 = QtWidgets.QLabel(self.widget_2)
        self.label_9.setGeometry(QtCore.QRect(20, 107, 30, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_9.setFont(font)
        self.label_9.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_9.setObjectName("label_9")
        self.label_servo6 = QtWidgets.QLabel(self.widget_2)
        self.label_servo6.setGeometry(QtCore.QRect(330, 360, 67, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_servo6.setFont(font)
        self.label_servo6.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_servo6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_servo6.setObjectName("label_servo6")
        self.label_servo3 = QtWidgets.QLabel(self.widget_2)
        self.label_servo3.setGeometry(QtCore.QRect(330, 150, 67, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_servo3.setFont(font)
        self.label_servo3.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_servo3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_servo3.setObjectName("label_servo3")
        self.label_servo5 = QtWidgets.QLabel(self.widget_2)
        self.label_servo5.setGeometry(QtCore.QRect(330, 290, 67, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_servo5.setFont(font)
        self.label_servo5.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_servo5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_servo5.setObjectName("label_servo5")
        self.horizontalSlider_servo5 = QtWidgets.QSlider(self.widget_2)
        self.horizontalSlider_servo5.setGeometry(QtCore.QRect(70, 310, 581, 31))
        self.horizontalSlider_servo5.setStyleSheet("QSlider{\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::groove:horizontal {\n"
"background: transparent;\n"
"height:15px;\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 28px;\n"
"height:28px;\n"
"margin-top: -7px;\n"
"margin-bottom: -7px;\n"
"border-radius:14px;\n"
"background: rgba(240, 240, 240);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: #575757;\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF9600;\n"
"}")
        self.horizontalSlider_servo5.setMinimum(125)
        self.horizontalSlider_servo5.setMaximum(875)
        self.horizontalSlider_servo5.setPageStep(1)
        self.horizontalSlider_servo5.setProperty("value", 500)
        self.horizontalSlider_servo5.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_servo5.setObjectName("horizontalSlider_servo5")
        self.horizontalSlider_servo3 = QtWidgets.QSlider(self.widget_2)
        self.horizontalSlider_servo3.setGeometry(QtCore.QRect(70, 170, 581, 31))
        self.horizontalSlider_servo3.setStyleSheet("QSlider{\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::groove:horizontal {\n"
"background: transparent;\n"
"height:15px;\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 28px;\n"
"height:28px;\n"
"margin-top: -7px;\n"
"margin-bottom: -7px;\n"
"border-radius:14px;\n"
"background: rgba(240, 240, 240);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: #575757;\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF9600;\n"
"}")
        self.horizontalSlider_servo3.setMinimum(50)
        self.horizontalSlider_servo3.setMaximum(950)
        self.horizontalSlider_servo3.setPageStep(1)
        self.horizontalSlider_servo3.setProperty("value", 500)
        self.horizontalSlider_servo3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_servo3.setObjectName("horizontalSlider_servo3")
        self.horizontalSlider_servo6 = QtWidgets.QSlider(self.widget_2)
        self.horizontalSlider_servo6.setGeometry(QtCore.QRect(70, 380, 581, 31))
        self.horizontalSlider_servo6.setStyleSheet("QSlider{\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::groove:horizontal {\n"
"background: transparent;\n"
"height:15px;\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 28px;\n"
"height:28px;\n"
"margin-top: -7px;\n"
"margin-bottom: -7px;\n"
"border-radius:14px;\n"
"background: rgba(240, 240, 240);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: #575757;\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF9600;\n"
"}")
        self.horizontalSlider_servo6.setMinimum(0)
        self.horizontalSlider_servo6.setMaximum(1000)
        self.horizontalSlider_servo6.setPageStep(1)
        self.horizontalSlider_servo6.setProperty("value", 500)
        self.horizontalSlider_servo6.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_servo6.setObjectName("horizontalSlider_servo6")
        self.horizontalSlider_servo4 = QtWidgets.QSlider(self.widget_2)
        self.horizontalSlider_servo4.setGeometry(QtCore.QRect(70, 240, 581, 31))
        self.horizontalSlider_servo4.setStyleSheet("QSlider{\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::groove:horizontal {\n"
"background: transparent;\n"
"height:15px;\n"
"border-radius: 5px;\n"
"}\n"
"\n"
"QSlider::handle:horizontal{\n"
"width: 28px;\n"
"height:28px;\n"
"margin-top: -7px;\n"
"margin-bottom: -7px;\n"
"border-radius:14px;\n"
"background: rgba(240, 240, 240);\n"
"}\n"
"QSlider::add-page{/*还没有滑上去的地方*/\n"
"    background: #575757;\n"
"}\n"
"QSlider::sub-page{/*已经划过的从地方*/                            \n"
"    background: #FF9600;\n"
"}")
        self.horizontalSlider_servo4.setMinimum(0)
        self.horizontalSlider_servo4.setMaximum(1000)
        self.horizontalSlider_servo4.setPageStep(1)
        self.horizontalSlider_servo4.setProperty("value", 500)
        self.horizontalSlider_servo4.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_servo4.setObjectName("horizontalSlider_servo4")
        self.label_servo4 = QtWidgets.QLabel(self.widget_2)
        self.label_servo4.setGeometry(QtCore.QRect(330, 220, 67, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_servo4.setFont(font)
        self.label_servo4.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_servo4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_servo4.setObjectName("label_servo4")
        self.label_10 = QtWidgets.QLabel(self.widget_2)
        self.label_10.setGeometry(QtCore.QRect(20, 177, 30, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_10.setFont(font)
        self.label_10.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_10.setObjectName("label_10")
        self.label_17 = QtWidgets.QLabel(self.widget_2)
        self.label_17.setGeometry(QtCore.QRect(20, 247, 30, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_17.setFont(font)
        self.label_17.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_17.setObjectName("label_17")
        self.label_20 = QtWidgets.QLabel(self.widget_2)
        self.label_20.setGeometry(QtCore.QRect(20, 317, 30, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_20.setFont(font)
        self.label_20.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_20.setObjectName("label_20")
        self.label_23 = QtWidgets.QLabel(self.widget_2)
        self.label_23.setGeometry(QtCore.QRect(20, 387, 30, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_23.setFont(font)
        self.label_23.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_23.setObjectName("label_23")
        self.tableWidget = QtWidgets.QTableWidget(self.widget)
        self.tableWidget.setGeometry(QtCore.QRect(10, 500, 591, 140))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tableWidget.sizePolicy().hasHeightForWidth())
        self.tableWidget.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("黑体")
        font.setPointSize(11)
        self.tableWidget.setFont(font)
        self.tableWidget.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.tableWidget.setStyleSheet("border:1px solid #242424;\n"
"border-color: rgb(200, 200, 200);\n"
"")
        self.tableWidget.setSizeAdjustPolicy(QtWidgets.QAbstractScrollArea.AdjustIgnored)
        self.tableWidget.setAutoScrollMargin(16)
        self.tableWidget.setTextElideMode(QtCore.Qt.ElideMiddle)
        self.tableWidget.setRowCount(0)
        self.tableWidget.setColumnCount(8)
        self.tableWidget.setObjectName("tableWidget")
        item = QtWidgets.QTableWidgetItem()
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        item.setFont(font)
        self.tableWidget.setHorizontalHeaderItem(0, item)
        item = QtWidgets.QTableWidgetItem()
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        self.tableWidget.setHorizontalHeaderItem(1, item)
        item = QtWidgets.QTableWidgetItem()
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        item.setFont(font)
        self.tableWidget.setHorizontalHeaderItem(2, item)
        item = QtWidgets.QTableWidgetItem()
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        item.setFont(font)
        self.tableWidget.setHorizontalHeaderItem(3, item)
        item = QtWidgets.QTableWidgetItem()
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        item.setFont(font)
        self.tableWidget.setHorizontalHeaderItem(4, item)
        item = QtWidgets.QTableWidgetItem()
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        item.setFont(font)
        self.tableWidget.setHorizontalHeaderItem(5, item)
        item = QtWidgets.QTableWidgetItem()
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        item.setFont(font)
        self.tableWidget.setHorizontalHeaderItem(6, item)
        item = QtWidgets.QTableWidgetItem()
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(10)
        font.setBold(False)
        font.setWeight(50)
        item.setFont(font)
        self.tableWidget.setHorizontalHeaderItem(7, item)
        self.tableWidget.horizontalHeader().setDefaultSectionSize(71)
        self.tableWidget.horizontalHeader().setMinimumSectionSize(20)
        self.tableWidget.horizontalHeader().setSortIndicatorShown(True)
        self.tableWidget.verticalHeader().setVisible(False)
        self.tableWidget.verticalHeader().setDefaultSectionSize(22)
        self.tableWidget.verticalHeader().setMinimumSectionSize(22)
        self.widget_14 = QtWidgets.QWidget(self.widget)
        self.widget_14.setGeometry(QtCore.QRect(690, 555, 671, 85))
        self.widget_14.setStyleSheet("background-color: rgb(255, 255, 255,);\n"
"border-radius:6px;")
        self.widget_14.setObjectName("widget_14")
        self.pushButton_delete_2 = QtWidgets.QPushButton(self.widget_14)
        self.pushButton_delete_2.setGeometry(QtCore.QRect(20, 170, 90, 40))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_delete_2.setFont(font)
        self.pushButton_delete_2.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(255, 255, 255);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}")
        self.pushButton_delete_2.setObjectName("pushButton_delete_2")
        self.pushButton_roi = QtWidgets.QPushButton(self.widget_14)
        self.pushButton_roi.setGeometry(QtCore.QRect(485, 17, 80, 50))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_roi.setFont(font)
        self.pushButton_roi.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(0, 0, 0);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}\n"
"")
        self.pushButton_roi.setObjectName("pushButton_roi")
        self.pushButton_read_dev = QtWidgets.QPushButton(self.widget_14)
        self.pushButton_read_dev.setGeometry(QtCore.QRect(10, 17, 80, 50))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_read_dev.setFont(font)
        self.pushButton_read_dev.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(0, 0, 0);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}")
        self.pushButton_read_dev.setIconSize(QtCore.QSize(40, 40))
        self.pushButton_read_dev.setObjectName("pushButton_read_dev")
        self.pushButton_save_dev = QtWidgets.QPushButton(self.widget_14)
        self.pushButton_save_dev.setGeometry(QtCore.QRect(105, 17, 80, 50))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_save_dev.setFont(font)
        self.pushButton_save_dev.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(0, 0, 0);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}")
        self.pushButton_save_dev.setIconSize(QtCore.QSize(40, 40))
        self.pushButton_save_dev.setObjectName("pushButton_save_dev")
        self.pushButton_calibrate = QtWidgets.QPushButton(self.widget_14)
        self.pushButton_calibrate.setGeometry(QtCore.QRect(390, 17, 80, 50))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_calibrate.setFont(font)
        self.pushButton_calibrate.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(0, 0, 0);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}\n"
"")
        self.pushButton_calibrate.setIconSize(QtCore.QSize(30, 30))
        self.pushButton_calibrate.setObjectName("pushButton_calibrate")
        self.pushButton_reset = QtWidgets.QPushButton(self.widget_14)
        self.pushButton_reset.setGeometry(QtCore.QRect(200, 17, 80, 50))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_reset.setFont(font)
        self.pushButton_reset.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(0, 0, 0);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}")
        self.pushButton_reset.setIconSize(QtCore.QSize(40, 40))
        self.pushButton_reset.setObjectName("pushButton_reset")
        self.pushButton_init = QtWidgets.QPushButton(self.widget_14)
        self.pushButton_init.setGeometry(QtCore.QRect(295, 17, 80, 50))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_init.setFont(font)
        self.pushButton_init.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(0, 0, 0);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}")
        self.pushButton_init.setIconSize(QtCore.QSize(40, 40))
        self.pushButton_init.setObjectName("pushButton_init")
        self.pushButton_update_config = QtWidgets.QPushButton(self.widget_14)
        self.pushButton_update_config.setGeometry(QtCore.QRect(580, 15, 80, 50))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.pushButton_update_config.setFont(font)
        self.pushButton_update_config.setStyleSheet("QPushButton{\n"
"border:2px;\n"
"background-color: #FFA500;\n"
"color: rgb(0, 0, 0);\n"
"}\n"
"QPushButton:pressed{\n"
"border:2px solid rgb(255, 102, 0, 0);}\n"
"")
        self.pushButton_update_config.setObjectName("pushButton_update_config")
        self.widget_7 = QtWidgets.QWidget(self.widget)
        self.widget_7.setGeometry(QtCore.QRect(690, 430, 671, 121))
        self.widget_7.setStyleSheet("background-color: rgb(255, 255, 255,);\n"
"border-radius:6px;")
        self.widget_7.setObjectName("widget_7")
        self.widget_5 = QtWidgets.QWidget(self.widget_7)
        self.widget_5.setGeometry(QtCore.QRect(10, 10, 200, 41))
        self.widget_5.setStyleSheet("background-color:  #FFA500;\n"
"border-radius:6px;")
        self.widget_5.setObjectName("widget_5")
        self.label_5 = QtWidgets.QLabel(self.widget_5)
        self.label_5.setGeometry(QtCore.QRect(10, 10, 51, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_5.setFont(font)
        self.label_5.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.label_x = QtWidgets.QLabel(self.widget_5)
        self.label_x.setGeometry(QtCore.QRect(70, 10, 120, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_x.setFont(font)
        self.label_x.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_x.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_x.setObjectName("label_x")
        self.widget_6 = QtWidgets.QWidget(self.widget_7)
        self.widget_6.setGeometry(QtCore.QRect(235, 10, 200, 41))
        self.widget_6.setStyleSheet("background-color:  #FFA500;\n"
"border-radius:6px;")
        self.widget_6.setObjectName("widget_6")
        self.label_11 = QtWidgets.QLabel(self.widget_6)
        self.label_11.setGeometry(QtCore.QRect(10, 10, 51, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_11.setFont(font)
        self.label_11.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setObjectName("label_11")
        self.label_y = QtWidgets.QLabel(self.widget_6)
        self.label_y.setGeometry(QtCore.QRect(70, 10, 120, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_y.setFont(font)
        self.label_y.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_y.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_y.setObjectName("label_y")
        self.widget_9 = QtWidgets.QWidget(self.widget_7)
        self.widget_9.setGeometry(QtCore.QRect(460, 10, 200, 41))
        self.widget_9.setStyleSheet("background-color:  #FFA500;\n"
"border-radius:6px;")
        self.widget_9.setObjectName("widget_9")
        self.label_12 = QtWidgets.QLabel(self.widget_9)
        self.label_12.setGeometry(QtCore.QRect(10, 10, 51, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_12.setFont(font)
        self.label_12.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_12.setAlignment(QtCore.Qt.AlignCenter)
        self.label_12.setObjectName("label_12")
        self.label_z = QtWidgets.QLabel(self.widget_9)
        self.label_z.setGeometry(QtCore.QRect(70, 10, 120, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_z.setFont(font)
        self.label_z.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_z.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_z.setObjectName("label_z")
        self.widget_10 = QtWidgets.QWidget(self.widget_7)
        self.widget_10.setGeometry(QtCore.QRect(10, 70, 200, 41))
        self.widget_10.setStyleSheet("background-color:  #FFA500;\n"
"border-radius:6px;")
        self.widget_10.setObjectName("widget_10")
        self.label_13 = QtWidgets.QLabel(self.widget_10)
        self.label_13.setGeometry(QtCore.QRect(10, 10, 50, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_13.setFont(font)
        self.label_13.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.label_roll = QtWidgets.QLabel(self.widget_10)
        self.label_roll.setGeometry(QtCore.QRect(70, 10, 120, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_roll.setFont(font)
        self.label_roll.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_roll.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_roll.setObjectName("label_roll")
        self.widget_11 = QtWidgets.QWidget(self.widget_7)
        self.widget_11.setGeometry(QtCore.QRect(235, 70, 200, 41))
        self.widget_11.setStyleSheet("background-color:  #FFA500;\n"
"border-radius:6px;")
        self.widget_11.setObjectName("widget_11")
        self.label_14 = QtWidgets.QLabel(self.widget_11)
        self.label_14.setGeometry(QtCore.QRect(15, 10, 50, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_14.setFont(font)
        self.label_14.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_14.setAlignment(QtCore.Qt.AlignCenter)
        self.label_14.setObjectName("label_14")
        self.label_pitch = QtWidgets.QLabel(self.widget_11)
        self.label_pitch.setGeometry(QtCore.QRect(70, 10, 120, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_pitch.setFont(font)
        self.label_pitch.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_pitch.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_pitch.setObjectName("label_pitch")
        self.widget_13 = QtWidgets.QWidget(self.widget_7)
        self.widget_13.setGeometry(QtCore.QRect(460, 70, 200, 41))
        self.widget_13.setStyleSheet("background-color:  #FFA500;\n"
"border-radius:6px;")
        self.widget_13.setObjectName("widget_13")
        self.label_15 = QtWidgets.QLabel(self.widget_13)
        self.label_15.setGeometry(QtCore.QRect(10, 10, 50, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_15.setFont(font)
        self.label_15.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_15.setAlignment(QtCore.Qt.AlignCenter)
        self.label_15.setObjectName("label_15")
        self.label_yaw = QtWidgets.QLabel(self.widget_13)
        self.label_yaw.setGeometry(QtCore.QRect(70, 10, 120, 17))
        font = QtGui.QFont()
        font.setFamily("Source Han Sans CN")
        self.label_yaw.setFont(font)
        self.label_yaw.setStyleSheet("color: rgb(0, 0, 0);")
        self.label_yaw.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_yaw.setObjectName("label_yaw")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "calibration 1.0"))
        self.pushButton_delete.setText(_translate("Form", "删除"))
        self.pushButton_add.setText(_translate("Form", "增加"))
        self.pushButton_open.setText(_translate("Form", "打开"))
        self.pushButton_save.setText(_translate("Form", "保存"))
        self.pushButton_update.setText(_translate("Form", "更新"))
        self.label_servo1.setText(_translate("Form", "500"))
        self.label_servo2.setText(_translate("Form", "500"))
        self.label_8.setText(_translate("Form", "ID 1"))
        self.label_9.setText(_translate("Form", "ID 2"))
        self.label_servo6.setText(_translate("Form", "500"))
        self.label_servo3.setText(_translate("Form", "500"))
        self.label_servo5.setText(_translate("Form", "500"))
        self.label_servo4.setText(_translate("Form", "500"))
        self.label_10.setText(_translate("Form", "ID 3"))
        self.label_17.setText(_translate("Form", "ID 4"))
        self.label_20.setText(_translate("Form", "ID 5"))
        self.label_23.setText(_translate("Form", "ID 6"))
        item = self.tableWidget.horizontalHeaderItem(0)
        item.setText(_translate("Form", "编号"))
        item = self.tableWidget.horizontalHeaderItem(1)
        item.setText(_translate("Form", "类型"))
        item = self.tableWidget.horizontalHeaderItem(2)
        item.setText(_translate("Form", "X"))
        item = self.tableWidget.horizontalHeaderItem(3)
        item.setText(_translate("Form", "Y"))
        item = self.tableWidget.horizontalHeaderItem(4)
        item.setText(_translate("Form", "Z"))
        item = self.tableWidget.horizontalHeaderItem(5)
        item.setText(_translate("Form", "Roll"))
        item = self.tableWidget.horizontalHeaderItem(6)
        item.setText(_translate("Form", "Pitch"))
        item = self.tableWidget.horizontalHeaderItem(7)
        item.setText(_translate("Form", "Yaw"))
        self.pushButton_delete_2.setText(_translate("Form", "删除"))
        self.pushButton_roi.setText(_translate("Form", "ROI校准"))
        self.pushButton_read_dev.setText(_translate("Form", "读取偏差"))
        self.pushButton_save_dev.setText(_translate("Form", "保存偏差"))
        self.pushButton_calibrate.setText(_translate("Form", "手眼标定"))
        self.pushButton_reset.setText(_translate("Form", "回中"))
        self.pushButton_init.setText(_translate("Form", "复位"))
        self.pushButton_update_config.setText(_translate("Form", "更新参数"))
        self.label_5.setText(_translate("Form", "X(m)"))
        self.label_x.setText(_translate("Form", "0"))
        self.label_11.setText(_translate("Form", "Y(m)"))
        self.label_y.setText(_translate("Form", "0"))
        self.label_12.setText(_translate("Form", "Z(m)"))
        self.label_z.setText(_translate("Form", "0"))
        self.label_13.setText(_translate("Form", "Roll°"))
        self.label_roll.setText(_translate("Form", "0"))
        self.label_14.setText(_translate("Form", "Pitch°"))
        self.label_pitch.setText(_translate("Form", "0"))
        self.label_15.setText(_translate("Form", "Yaw°"))
        self.label_yaw.setText(_translate("Form", "0"))

import resource_rc
