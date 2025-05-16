import sys
import math
from PyQt5 import QtWidgets, QtCore, QtGui


class AnimatedSpotlight(QtWidgets.QWidget):
    def __init__(self, x, y, target_radius, decay, duration):
        super().__init__(
            flags=(
                QtCore.Qt.FramelessWindowHint
                | QtCore.Qt.WindowStaysOnTopHint
                | QtCore.Qt.Tool
            )
        )
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.setAttribute(QtCore.Qt.WA_ShowWithoutActivating)
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)

        screen_geom = QtWidgets.QApplication.primaryScreen().geometry()
        self.setGeometry(screen_geom)

        self.cx = x
        self.cy = y
        self.decay = decay
        self._target_radius = target_radius

        w, h = screen_geom.width(), screen_geom.height()
        initial_rad = math.hypot(w, h)
        self._radius = initial_rad

        self.anim = QtCore.QPropertyAnimation(self, b"radius")
        self.anim.setDuration(int(duration * 1000))
        self.anim.setStartValue(initial_rad)
        self.anim.setEndValue(self._target_radius)
        self.anim.setEasingCurve(QtCore.QEasingCurve.InOutQuad)
        self.anim.finished.connect(self.on_radius_finished)
        self.anim.start()

    @QtCore.pyqtProperty(float)
    def radius(self):
        return self._radius

    @radius.setter
    def radius(self, r):
        self._radius = r
        self.update()

    def on_radius_finished(self):
        QtCore.QTimer.singleShot(3000, self.start_fade_out)

    def start_fade_out(self):
        self.fade_anim = QtCore.QPropertyAnimation(self, b"windowOpacity")
        self.fade_anim.setDuration(1000)
        self.fade_anim.setStartValue(1.0)
        self.fade_anim.setEndValue(0.0)
        self.fade_anim.finished.connect(self.close)
        self.fade_anim.start()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        outer = self._radius + self.decay
        grad = QtGui.QRadialGradient(QtCore.QPointF(self.cx, self.cy), outer)
        grad.setColorAt(0.0, QtGui.QColor(0, 0, 0, 0))
        grad.setColorAt(
            self._radius / outer if outer > 0 else 0.0, QtGui.QColor(0, 0, 0, 0)
        )
        grad.setColorAt(1.0, QtGui.QColor(0, 0, 0, 170))
        painter.setBrush(QtGui.QBrush(grad))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawRect(self.rect())

    def closeEvent(self, event):
        QtWidgets.QApplication.instance().quit()
        super().closeEvent(event)


def spotlight(x, y, target_radius, decay, duration):
    app = QtWidgets.QApplication(sys.argv)
    w = AnimatedSpotlight(
        x=x,
        y=y,
        target_radius=target_radius,
        decay=decay,
        duration=duration,
    )
    w.show()
    return app.exec_()
