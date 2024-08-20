import sys
import serial
from PyQt6.QtWidgets import QMainWindow, QApplication, QInputDialog
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import pyqtSlot, QTimer
from Car_frontend import Ui_MainWindow
from Camera_Feed import WebCam


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.ui = self.setupUi(self)
        self.arduino_port = serial.Serial(baudrate= 9600)
        self.attempsToOpenPort = 0
        self.webcam = WebCam()
        self.mode = 0
        self.webcam.change_pixmap_signal.connect(self.update_image)
        self.Screenshot.clicked.connect(self.take_screenshot)

        self.start_camera_feed()
        self.dataRead = QTimer()
        self.openPort()
        self.dataRead.timeout.connect(slot= self.readData)
        self.dataRead.start(100)
        self.forward.pressed.connect(self.Forward)
        self.backward.pressed.connect(self.Backward)
        self.left.pressed.connect(self.Left)
        self.right.pressed.connect(self.Right)
        self.forward.released.connect(self.Stop)
        self.left.released.connect(self.Stop)
        self.right.released.connect(self.Stop)
        self.backward.released.connect(self.Stop)
        self.setSpeed.clicked.connect(self.findSelectedSpeed)
        self.setMode.clicked.connect(self.findSelectedMode)
        self.setDistancePB.clicked.connect(self.getDistanceTotheWall)



    def openPort(self):
        try:
            self.arduino_port.port = "COM5"
            self.arduino_port.open()
        except serial.SerialException as communication:
            if self.attempsToOpenPort < 10:
                self.attempsToOpenPort += 1
                print(f"Their is error in opening the serial port attempt: {self.attempsToOpenPort}")
                self.openPort()
            else:
                print(f"Connection failed: {communication}")
                sys.exit()
        self.connectivity.setText("Connectivity\n Connected")


    def start_camera_feed(self):
        self.webcam.start()

    # def stop_camera_feed(self):
    # self.webcam.stop()

    def take_screenshot(self):
        self.webcam.screenshot()

    @pyqtSlot(QImage)
    def update_image(self, image: QImage):
        self.camera_feed.setPixmap(QPixmap.fromImage(image))

    # def closeEvent(self, event):
    # self.stop_camera_feed()
    # event.accept()

    def Forward(self):
        self.motion.setText("Motion:\nCar moves forward")
        self.send_direction(b'F')

    def Backward(self):
        self.motion.setText("Motion:\nCar moves backward")
        self.send_direction(b'B')

    def Left(self):
        self.motion.setText("Motion:\nCar moves left")
        self.send_direction(b'L')

    def Right(self):
        self.motion.setText("Motion:\nCar moves right")
        self.send_direction(b'R')

    def Stop(self):
        self.motion.setText("Motion:\nCar stops")
        self.send_direction(b'S')

    def send_direction(self, direction):
        if self.mode:
            return
        print(f"Car movement: {direction}") # this line test the code prints in terminal (can be removed)
        try:
            # Send the direction to the Arduino
            self.arduino_port.write(direction)
            print(f"Car movement: {direction}")

        except serial.SerialException as communication:
            print(f"Connection lost: {communication}")
            self.connectivity.setText("Connectivity\nNot connected")
            self.openPort()
    def setCarSpeed(self, speed):
        print(f"Car speed: {speed}")  # this line test the code prints in terminal (can be removed)
        try:
            # Send the direction to the Arduino
            self.arduino_port.write(speed)
            print(f"Car speed: {speed}")

        except serial.SerialException as communication:
            print(f"Connection lost: {communication}")
            self.connectivity.setText("Connectivity\nNot connected")
            self.openPort()
    def findSelectedSpeed(self):

        content = self.speed.currentText()
        if content == "Low":
            self.setCarSpeed(b'l')
        elif content == "Medium":
            self.setCarSpeed(b'm')
        elif content == "High":
            self.setCarSpeed(b'h')

    def findSelectedMode(self):
        content = self.modes.currentText()
        if content == "Autonomous":
            self.mode = 1
            self.sendSelectedMode(b'A')
        else:
            self.mode = 0
            self.sendSelectedMode(b'M')
    def getDistanceTotheWall(self):
        if self.mode == 0:
            return
        text, ok = QInputDialog.getText(self, "Set point", "Enter the distance to the wall.")
        if ok and text:
            self.distanceSet.setText(f"{text}Cm")
            self.arduino_port.write(b's')
            self.arduino_port.write(text.encode(encoding="utf-8"))
            self.arduino_port.write(b' ')



    def sendSelectedMode(self, mode):
        print(f"Car mode: {mode}")  # this line test the code prints in terminal (can be removed)
        try:
            # Send the direction to the Arduino
            self.arduino_port.write(mode)
            print(f"Car mode: {mode}")

        except serial.SerialException as communication:
            print(f"Connection lost: {communication}")
            self.connectivity.setText("Connectivity\nNot connected")
            self.openPort()
    def readData(self):
        try:
            if self.arduino_port.in_waiting:
                data = self.arduino_port.read(1)
                data = data.decode("utf-8")
                if data == 'c':
                    self.updateCurrent()
                elif data == 'd':
                    self.updateDistance()
                else:
                    self.updateVolt()
        except serial.SerialException as communication:
            print(f"Connection lost: {communication}")
            self.connectivity.setText("Connectivity\nNot connected")
            self.openPort()


    def updateCurrent(self):
        try:
            data = self.arduino_port.read_until(b' ')
            data = data.decode("utf-8")
            self.current.setText(f"Current: {data}A")
        except serial.SerialException as communication:
            print(f"Connection lost: {communication}")
            self.connectivity.setText("Connectivity\nNot connected")
            self.openPort()

    def updateVolt(self):
        try:
            data = self.arduino_port.read_until(b' ')
            data = data.decode("utf-8")
            self.voltage.setText(f"volt: {data}V")
        except serial.SerialException as communication:
            print(f"Connection lost: {communication}")
            self.connectivity.setText("Connectivity\nNot connected")
            self.openPort()

    def updateDistance(self):
        try:
            data = self.arduino_port.read_until(b' ')
            data = data.decode("utf-8")
            if data[0] == '0':
                return
            self.ultrasonic.setText(f"Distance to the wall:\n {data}Cm")
        except serial.SerialException as communication:
            print(f"Connection lost: {communication}")
            self.connectivity.setText("Connectivity\nNot connected")
            self.openPort()


    def closeEvent(self):
        # Close the serial connection when the GUI is closed
        if self.arduino_port:
            self.arduino_port.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()
