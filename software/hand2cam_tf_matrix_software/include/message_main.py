import sys, os
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QDialog

pwd = os.getcwd()
file_path = os.path.dirname(__file__)

if pwd == file_path:
    import message
    import message_tips
else:
    from include import message
    from include import message_tips

class Message():
    def __init__(self, home_window):
        self.home_window = home_window
        self.delay = 1000
        self.message_info = None
        self.message_close = False
        self.message_timer = QTimer(timeout=self.message_create)

    def message_create(self):
        if self.message_close:
            self.message_close = False
            self.message_timer.stop()
            self.qdi.accept()
        else:
            self.qdi = QDialog()
            self.d = message_tips.Ui_Dialog()
            self.qdi.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)

            self.d.setupUi(self.qdi)
            self.d.label.setText(self.message_info)
            qr = self.home_window.frameGeometry()
            x = qr.x() + int(qr.width()/2)
            y = qr.y() + int(qr.height()/2)
            self.qdi.show()
            qr = self.qdi.frameGeometry()
            x_ = int(qr.width()/2)
            y_ = int(qr.height()/2)

            self.qdi.move(x - x_, y - y_)
            self.message_close = True
            self.message_timer.stop()
            self.message_timer.start(self.delay)
    
    def tips(self, msg, delay=1000):
        self.delay = delay
        self.message_info = msg
        self.message_timer.start(50)

    def info(self, msg):
        self.qdi = QDialog()
        self.d = message.Ui_Dialog()
        self.qdi.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)

        self.d.setupUi(self.qdi)
        self.qdi.show()
        self.d.label.setText(msg)
        self.d.pushButton_ok.pressed.connect(self.closeqdialog)

    def closeqdialog(self):
        self.qdi.accept()
