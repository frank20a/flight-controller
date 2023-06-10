# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'controller.ui'
##
## Created by: Qt User Interface Compiler version 6.5.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QGridLayout, QHBoxLayout, QMainWindow,
    QMenuBar, QPushButton, QSizePolicy, QSlider,
    QSpacerItem, QStatusBar, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayoutWidget = QWidget(self.centralwidget)
        self.gridLayoutWidget.setObjectName(u"gridLayoutWidget")
        self.gridLayoutWidget.setGeometry(QRect(0, 0, 801, 551))
        self.gridLayout = QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.yaw = QSlider(self.gridLayoutWidget)
        self.yaw.setObjectName(u"yaw")
        self.yaw.setMaximum(1023)
        self.yaw.setValue(512)
        self.yaw.setOrientation(Qt.Horizontal)

        self.gridLayout.addWidget(self.yaw, 1, 0, 1, 1)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalSpacer_3 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_2.addItem(self.horizontalSpacer_3)

        self.pitch = QSlider(self.gridLayoutWidget)
        self.pitch.setObjectName(u"pitch")
        self.pitch.setMaximum(1023)
        self.pitch.setValue(512)
        self.pitch.setOrientation(Qt.Vertical)

        self.horizontalLayout_2.addWidget(self.pitch)

        self.horizontalSpacer_4 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_2.addItem(self.horizontalSpacer_4)


        self.gridLayout.addLayout(self.horizontalLayout_2, 0, 1, 1, 1)

        self.roll = QSlider(self.gridLayoutWidget)
        self.roll.setObjectName(u"roll")
        self.roll.setMaximum(1023)
        self.roll.setValue(512)
        self.roll.setOrientation(Qt.Horizontal)

        self.gridLayout.addWidget(self.roll, 1, 1, 1, 1)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)

        self.thrust = QSlider(self.gridLayoutWidget)
        self.thrust.setObjectName(u"thrust")
        self.thrust.setMaximum(1023)
        self.thrust.setOrientation(Qt.Vertical)

        self.horizontalLayout.addWidget(self.thrust)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer_2)


        self.gridLayout.addLayout(self.horizontalLayout, 0, 0, 1, 1)

        self.btn1 = QPushButton(self.gridLayoutWidget)
        self.btn1.setObjectName(u"btn1")

        self.gridLayout.addWidget(self.btn1, 2, 0, 1, 1)

        self.btn2 = QPushButton(self.gridLayoutWidget)
        self.btn2.setObjectName(u"btn2")

        self.gridLayout.addWidget(self.btn2, 2, 1, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 800, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.btn1.setText(QCoreApplication.translate("MainWindow", u"Left Stick Press", None))
        self.btn2.setText(QCoreApplication.translate("MainWindow", u"Right Stick Press", None))
    # retranslateUi

