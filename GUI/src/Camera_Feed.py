import sys
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtGui import QPixmap, QImage
import cv2
from datetime import datetime


class WebCam(QThread):
    change_pixmap_signal = pyqtSignal(QImage)

    def __init__(self):
        super().__init__()
        self.width = 640
        self.height = 480
        self.active = False
        self.capture = None

    def run(self):
        self.active = True
        self.capture = cv2.VideoCapture(0)
        while self.active:
            ret, frame = self.capture.read()
            if ret:
                rgb_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                height, width, channels = rgb_img.shape
                bytes_per_line = channels * width
                convertToQtFormat = QImage(
                    rgb_img.data,
                    width,
                    height,
                    bytes_per_line,
                    QImage.Format.Format_RGB888,
                )
                pic = convertToQtFormat.scaled(
                    width, height, Qt.AspectRatioMode.KeepAspectRatio
                )
                self.change_pixmap_signal.emit(pic)
        self.capture.release()
        # cv2.destroyAllWindows()

    # def stop(self):
    # self.active = False
    # self.wait()

    def screenshot(self):
        if not self.active:
            print("Webcam is not streaming.")
            return

        if self.capture:
            ret, frame = self.capture.read()
            if ret:
                filename = f"screenshot_{datetime.now().strftime('%d%m%Y_%H%M')}.png"
                cv2.imwrite(filename, frame)
                print(f"Screenshot saved as {filename}")
            else:
                print("Failed to take screenshot.")
