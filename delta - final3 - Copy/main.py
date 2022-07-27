import serial
import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer
from ui_main_window import *
ser = serial.Serial('COM4', 57600, timeout=1)
classes = []
with open('classes.txt', 'r') as f:
    classes = f.read().splitlines()
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent=parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.net = cv2.dnn.readNetFromDarknet("yolov3.cfg",r"yolov3.weights")
        self.colors = np.random.uniform(0, 255, size=(100, 3))


        self.x_Axis = 0
        self.y_Axis = 0
        self.z_Axis = 150    
        self.gripper = 0    

        self.timer = QTimer()
        self.timer.timeout.connect(self.viewCam)

        self.ui.tab2_forward_button.pressed.connect(self.x_Axis_forward)
        self.ui.tab2_backward_button.pressed.connect(self.x_Axis_backward)
        self.ui.tab2_right_button.pressed.connect(self.y_Axis_Right)
        self.ui.tab2_left_button.pressed.connect(self.y_Axis_Left)
        self.ui.tab2_up_button.pressed.connect(self.z_Axis_Up)
        self.ui.tab2_down_button.pressed.connect(self.z_Axis_Down)
        self.ui.tab2_Gripper_button.pressed.connect(self.gripper_On)
        self.ui.tab2_Gripper_button.released.connect(self.gripper_Off)
        self.ui.tab2_Reset.pressed.connect(self.reset_Theta)
        # self.ui.tab1_Start.clicked.connect(self.controlTimer)
        self.ui.tab1_Start.clicked.connect(self.Start)
        self.ui.tab1_Stop.clicked.connect(self.Stop)

    def update_label(self, x_Axis, y_Axis, z_Axis):
        self.ui.tab2_textBrowserX1.setText(str(self.x_Axis))
        self.ui.tab2_textBrowserX2.setText(str(self.y_Axis))
        self.ui.tab2_textBrowserX3.setText(str(self.z_Axis))

    def reset_Theta(self):
        self.x_Axis = 0
        self.y_Axis = 0
        self.z_Axis = 150     
        self.sent_to_adruino(self.gripper, self.x_Axis, self.y_Axis, self.z_Axis)
        self.update_label(self.x_Axis, self.y_Axis, self.z_Axis)  

    def sent_to_adruino(self, gripper, x_Axis, y_Axis, z_Axis):
        rot = (str(gripper) + "," + str(x_Axis) +","+ str(y_Axis) + "," + str(z_Axis) + "\n")
        ser.write(rot.encode("utf-8"))

    def gripper_On(self):
        self.gripper = 1
        self.sent_to_adruino(self.gripper, self.x_Axis, self.y_Axis, self.z_Axis)
    def gripper_Off(self):
        self.gripper = 0
        self.sent_to_adruino(self.gripper, self.x_Axis, self.y_Axis, self.z_Axis)

    def x_Axis_forward(self):
        self.x_Axis += 1
        self.sent_to_adruino(self.gripper, self.x_Axis, self.y_Axis, self.z_Axis)
        self.update_label(self.x_Axis, self.y_Axis, self.z_Axis) 
    def x_Axis_backward(self):
        self.x_Axis -= 1
        self.sent_to_adruino(self.gripper, self.x_Axis, self.y_Axis, self.z_Axis)
        self.update_label(self.x_Axis, self.y_Axis, self.z_Axis) 
    def y_Axis_Right(self):
        self.y_Axis += 1
        self.sent_to_adruino(self.gripper, self.x_Axis, self.y_Axis, self.z_Axis)
        self.update_label(self.x_Axis, self.y_Axis, self.z_Axis) 
    def y_Axis_Left(self):
        self.y_Axis -= 1
        self.sent_to_adruino(self.gripper, self.x_Axis, self.y_Axis, self.z_Axis)
        self.update_label(self.x_Axis, self.y_Axis, self.z_Axis) 
    def z_Axis_Up(self):
        self.z_Axis += 1
        self.sent_to_adruino(self.gripper, self.x_Axis, self.y_Axis, self.z_Axis)
        self.update_label(self.x_Axis, self.y_Axis, self.z_Axis) 
    def z_Axis_Down(self):
        self.z_Axis -= 1
        self.sent_to_adruino(self.gripper, self.x_Axis, self.y_Axis, self.z_Axis)
        self.update_label(self.x_Axis, self.y_Axis, self.z_Axis) 

    def viewCam(self):
        ret, image = self.cap.read()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width, channel = image.shape
        blob = cv2.dnn.blobFromImage(image, 1/255,(416,416),(0,0,0),swapRB = True,crop= False)
        self.net.setInput(blob)
        output_layers_name = self.net.getUnconnectedOutLayersNames()
        layerOutputs = self.net.forward(output_layers_name)
        boxes =[]
        confidences = []
        class_ids = []

        for output in layerOutputs:
            for detection in output:
                score = detection[5:]
                class_id = np.argmax(score)
                confidence = score[class_id]
                if confidence > 0.6:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.2, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        if len(indexes)> 0:
            for i in indexes.flatten():
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                confidence = str(round(confidences[i],2))
                color = self.colors[i]
                cv2.rectangle(image, (x,y), (x+w, y+h), color, 2)
                print(x)
                cv2.putText(image, label + " " + confidence, (x, y+20), font, 2, (255,255,255), 2)
        step = channel*width
        qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)
        self.ui.tab1_labelVideo.setPixmap(QPixmap.fromImage(qImg))

    def Start(self):
        self.cap = cv2.VideoCapture(0)
        self.timer.start(20)
    def Stop(self):
        self.timer.stop()
        self.cap.release()
        self.ui.tab1_labelVideo.setPixmap(QtGui.QPixmap("images/image.jpg"))
        self.ui.tab1_labelVideo.setScaledContents(True)     

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())