from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtCore import (
    Qt,
    QTimer,
    pyqtProperty,
    QPropertyAnimation,
    QEasingCurve,
    QObject,
)
from PyQt5.QtGui import QColor, QPainter


class OpacityWrapper(QObject):
    def __init__(self, index, parent):
        super().__init__(parent)
        self.index = index
        self.parent = parent
        self._opacity = 0.0

    def getOpacity(self):
        return self._opacity

    def setOpacity(self, val):
        self._opacity = val
        # actualiza la opacidad en el overlay y fuerza repaint
        self.parent.opacities[self.index] = val
        self.parent.update()

    opacity = pyqtProperty(float, getOpacity, setOpacity)


class BoxOverlay(QWidget):
    def __init__(self, bboxes, duration_total=5000, min_duration_per_box=500):
        super().__init__(
            flags=Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint | Qt.Tool
        )
        boxes = [
            (*[int(v) for v in bbox.top_left], *[int(v) for v in bbox.bottom_right])
            for bbox in bboxes
        ]
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setAttribute(Qt.WA_ShowWithoutActivating)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)

        # ventana full-screen transparente
        self.setGeometry(QApplication.primaryScreen().geometry())

        self.boxes = boxes
        n = len(boxes)
        self.opacities = [0.0] * n

        # colores HSL espaciados
        self.colors = []
        for i in range(n):
            c = QColor.fromHsv(int(i * 360 / n), 255, 200)
            self.colors.append(c)

        # wrappers que expondrán la propiedad 'opacity'
        self.wrappers = [OpacityWrapper(i, self) for i in range(n)]

        total_ms = duration_total
        min_seg = min_duration_per_box

        if n > 1 and total_ms / n < min_seg:
            seg_ms = min_seg
            dt_ms = int((total_ms - seg_ms) / (n - 1))
        else:
            seg_ms = int(total_ms / n)
            dt_ms = seg_ms

        # crea y programa cada animación con retraso
        for i, wrapper in enumerate(self.wrappers):
            # fade-in
            fade_in = QPropertyAnimation(wrapper, b"opacity", self)
            fade_in.setDuration(seg_ms // 2)
            fade_in.setStartValue(0.0)
            fade_in.setEndValue(0.5)
            fade_in.setEasingCurve(QEasingCurve.InOutQuad)

            # fade-out
            fade_out = QPropertyAnimation(wrapper, b"opacity", self)
            fade_out.setDuration(seg_ms // 2)
            fade_out.setStartValue(0.5)
            fade_out.setEndValue(0.0)
            fade_out.setEasingCurve(QEasingCurve.InOutQuad)

            # en vez de QSequentialAnimationGroup, arrancamos cada anim por callback
            def start_fade_out(f=fade_out):
                f.start()

            fade_in.finished.connect(start_fade_out)

            # arranca fade-in tras i*dt_ms
            QTimer.singleShot(i * dt_ms, fade_in.start)

        # cierra la ventana al acabar total_ms
        QTimer.singleShot(total_ms, self.close)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        for i, (x1, y1, x2, y2) in enumerate(self.boxes):
            op = self.opacities[i]
            if op <= 0:
                continue
            c = QColor(self.colors[i])
            c.setAlphaF(op)
            painter.setBrush(c)
            painter.setPen(Qt.NoPen)
            painter.drawRect(x1, y1, x2 - x1, y2 - y1)

    def closeEvent(self, event):
        # cierra la app completa
        QApplication.instance().quit()
        super().closeEvent(event)
