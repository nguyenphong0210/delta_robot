# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_main_window.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1059, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setIconSize(QtCore.QSize(60, 60))
        self.tabWidget.setObjectName("tabWidget")
        self.tab_1 = QtWidgets.QWidget()
        self.tab_1.setObjectName("tab_1")
        self.tab1_labelVideo = QtWidgets.QLabel(self.tab_1)
        self.tab1_labelVideo.setGeometry(QtCore.QRect(10, 10, 671, 501))
        self.tab1_labelVideo.setText("")
        self.tab1_labelVideo.setPixmap(QtGui.QPixmap("images/image.jpg"))
        self.tab1_labelVideo.setScaledContents(True)
        self.tab1_labelVideo.setObjectName("tab1_labelVideo")
        self.tab1_Start = QtWidgets.QPushButton(self.tab_1)
        self.tab1_Start.setGeometry(QtCore.QRect(750, 10, 93, 28))
        self.tab1_Start.setObjectName("tab1_Start")
        self.tab1_Stop = QtWidgets.QPushButton(self.tab_1)
        self.tab1_Stop.setGeometry(QtCore.QRect(860, 10, 93, 28))
        self.tab1_Stop.setObjectName("tab1_Stop")
        self.tabWidget.addTab(self.tab_1, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.layoutWidget = QtWidgets.QWidget(self.tab_2)
        self.layoutWidget.setGeometry(QtCore.QRect(120, 17, 295, 30))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.tab2_Start = QtWidgets.QPushButton(self.layoutWidget)
        self.tab2_Start.setObjectName("tab2_Start")
        self.horizontalLayout.addWidget(self.tab2_Start)
        self.tab2_Stop = QtWidgets.QPushButton(self.layoutWidget)
        self.tab2_Stop.setObjectName("tab2_Stop")
        self.horizontalLayout.addWidget(self.tab2_Stop)
        self.tab2_Reset = QtWidgets.QPushButton(self.layoutWidget)
        self.tab2_Reset.setObjectName("tab2_Reset")
        self.horizontalLayout.addWidget(self.tab2_Reset)
        self.tab2_forward_button = QtWidgets.QPushButton(self.tab_2)
        self.tab2_forward_button.setGeometry(QtCore.QRect(250, 90, 51, 81))
        self.tab2_forward_button.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("icons/up-icon.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.tab2_forward_button.setIcon(icon)
        self.tab2_forward_button.setIconSize(QtCore.QSize(60, 60))
        self.tab2_forward_button.setAutoRepeat(True)
        self.tab2_forward_button.setAutoDefault(False)
        self.tab2_forward_button.setDefault(False)
        self.tab2_forward_button.setObjectName("tab2_forward_button")
        self.tab2_backward_button = QtWidgets.QPushButton(self.tab_2)
        self.tab2_backward_button.setGeometry(QtCore.QRect(250, 320, 51, 81))
        self.tab2_backward_button.setText("")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("icons/down-icon.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.tab2_backward_button.setIcon(icon1)
        self.tab2_backward_button.setIconSize(QtCore.QSize(60, 60))
        self.tab2_backward_button.setAutoRepeat(True)
        self.tab2_backward_button.setObjectName("tab2_backward_button")
        self.tab2_left_button = QtWidgets.QPushButton(self.tab_2)
        self.tab2_left_button.setGeometry(QtCore.QRect(360, 220, 81, 51))
        self.tab2_left_button.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("icons/right-icon.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.tab2_left_button.setIcon(icon2)
        self.tab2_left_button.setIconSize(QtCore.QSize(60, 60))
        self.tab2_left_button.setAutoRepeat(True)
        self.tab2_left_button.setDefault(False)
        self.tab2_left_button.setObjectName("tab2_left_button")
        self.tab2_right_button = QtWidgets.QPushButton(self.tab_2)
        self.tab2_right_button.setGeometry(QtCore.QRect(110, 220, 81, 51))
        self.tab2_right_button.setText("")
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("icons/left-icon.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.tab2_right_button.setIcon(icon3)
        self.tab2_right_button.setIconSize(QtCore.QSize(60, 60))
        self.tab2_right_button.setAutoRepeat(True)
        self.tab2_right_button.setObjectName("tab2_right_button")
        self.tab2_up_button = QtWidgets.QPushButton(self.tab_2)
        self.tab2_up_button.setGeometry(QtCore.QRect(240, 200, 71, 41))
        self.tab2_up_button.setText("")
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap("icons/up_icon.jpg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.tab2_up_button.setIcon(icon4)
        self.tab2_up_button.setIconSize(QtCore.QSize(60, 60))
        self.tab2_up_button.setAutoRepeat(True)
        self.tab2_up_button.setObjectName("tab2_up_button")
        self.tab2_down_button = QtWidgets.QPushButton(self.tab_2)
        self.tab2_down_button.setGeometry(QtCore.QRect(240, 250, 71, 41))
        self.tab2_down_button.setText("")
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap("icons/down_icon.jpg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.tab2_down_button.setIcon(icon5)
        self.tab2_down_button.setIconSize(QtCore.QSize(60, 60))
        self.tab2_down_button.setAutoRepeat(True)
        self.tab2_down_button.setObjectName("tab2_down_button")
        self.tab2_Gripper_button = QtWidgets.QPushButton(self.tab_2)
        self.tab2_Gripper_button.setGeometry(QtCore.QRect(240, 420, 81, 51))
        self.tab2_Gripper_button.setObjectName("tab2_Gripper_button")
        self.tab2_labelX1 = QtWidgets.QLabel(self.tab_2)
        self.tab2_labelX1.setGeometry(QtCore.QRect(580, 140, 55, 41))
        self.tab2_labelX1.setMinimumSize(QtCore.QSize(10, 10))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.tab2_labelX1.setFont(font)
        self.tab2_labelX1.setObjectName("tab2_labelX1")
        self.tab2_labelX2 = QtWidgets.QLabel(self.tab_2)
        self.tab2_labelX2.setGeometry(QtCore.QRect(580, 210, 55, 41))
        self.tab2_labelX2.setMinimumSize(QtCore.QSize(10, 10))
        self.tab2_labelX2.setBaseSize(QtCore.QSize(10, 10))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.tab2_labelX2.setFont(font)
        self.tab2_labelX2.setObjectName("tab2_labelX2")
        self.tab2_labelX3 = QtWidgets.QLabel(self.tab_2)
        self.tab2_labelX3.setGeometry(QtCore.QRect(580, 290, 55, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.tab2_labelX3.setFont(font)
        self.tab2_labelX3.setObjectName("tab2_labelX3")
        self.tab2_textBrowserX1 = QtWidgets.QTextBrowser(self.tab_2)
        self.tab2_textBrowserX1.setGeometry(QtCore.QRect(650, 130, 151, 51))
        self.tab2_textBrowserX1.setObjectName("tab2_textBrowserX1")
        self.tab2_textBrowserX2 = QtWidgets.QTextBrowser(self.tab_2)
        self.tab2_textBrowserX2.setGeometry(QtCore.QRect(650, 210, 151, 51))
        self.tab2_textBrowserX2.setObjectName("tab2_textBrowserX2")
        self.tab2_textBrowserX3 = QtWidgets.QTextBrowser(self.tab_2)
        self.tab2_textBrowserX3.setGeometry(QtCore.QRect(650, 290, 151, 51))
        self.tab2_textBrowserX3.setObjectName("tab2_textBrowserX3")
        self.tabWidget.addTab(self.tab_2, "")
        self.gridLayout.addWidget(self.tabWidget, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.tab1_Start.setText(_translate("MainWindow", "Start"))
        self.tab1_Stop.setText(_translate("MainWindow", "Stop"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_1), _translate("MainWindow", "Auto"))
        self.tab2_Start.setText(_translate("MainWindow", "Start"))
        self.tab2_Stop.setText(_translate("MainWindow", "Stop"))
        self.tab2_Reset.setText(_translate("MainWindow", "Reset"))
        self.tab2_Gripper_button.setText(_translate("MainWindow", "Gripper"))
        self.tab2_labelX1.setText(_translate("MainWindow", "X1"))
        self.tab2_labelX2.setText(_translate("MainWindow", "X2"))
        self.tab2_labelX3.setText(_translate("MainWindow", "X3"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Manual"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
