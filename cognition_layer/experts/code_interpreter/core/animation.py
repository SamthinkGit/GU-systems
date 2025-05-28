import time
from typing import Literal

import markdown
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import QEventLoop
from PyQt5.QtCore import QThread
from PyQt5.QtWebEngineWidgets import QWebEngineView

from action_space.tools.pyqt_utils import get_app
from ecm.shared import get_root_path

# Highlight.js assets for code syntax highlighting (GitHub Dark Dimmed)
HIGHLIGHT_JS_CSS = "https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.7.0/styles/github-dark-dimmed.min.css"
HIGHLIGHT_JS_SCRIPT = (
    "https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.7.0/highlight.min.js"
)


class WorkerThread(QThread):
    data_ready = pyqtSignal(object)
    finished = pyqtSignal()
    ask_confirmation = pyqtSignal()
    confirmation_response = pyqtSignal(str)

    def __init__(self, stream_generator, delay=0, parent=None):
        super().__init__(parent)
        self._stream = stream_generator
        self.delay = delay
        self._confirmation = None
        self.confirmation_response.connect(self._on_confirmation)

    def run(self):
        for message in self._stream:
            if message.get("type") == "confirmation":

                self.ask_confirmation.emit()
                loop = QEventLoop()
                self.confirmation_response.connect(loop.quit)
                loop.exec_()

                self.confirmation_response.disconnect(loop.quit)

                print(f"RECEIVED CONFIRMATION={self._confirmation}")
                if self._confirmation.lower() == "n":
                    self.data_ready.emit(
                        {"role": "system", "content": "Task Aborted by the user"}
                    )
                    break

            # emito cualquier otro mensaje o confirmación aceptada
            self.data_ready.emit(message)
            time.sleep(self.delay)

        self.finished.emit()

    @pyqtSlot(str)
    def _on_confirmation(self, resp: str):
        """Guarda el valor, el quit lo hace el loop conectado."""
        self._confirmation = resp


class GradientFrame(QtWidgets.QFrame):
    def __init__(self, w, h, radius=15, parent=None):
        super().__init__(parent)
        self.w = w
        self.h = h
        self.radius = radius
        self.setGeometry(0, 0, w, h)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        grad = QtGui.QLinearGradient(0, 0, self.w, 0)
        grad.setColorAt(0.0, QtGui.QColor(0, 0, 0, 0))
        grad.setColorAt(0.1, QtGui.QColor(0, 0, 0, 180))
        grad.setColorAt(1.0, QtGui.QColor(0, 0, 0, 180))
        painter.fillRect(0, 0, self.w, self.h, QtGui.QBrush(grad))
        painter.end()


class OverlayMarkdownViewer(QtWidgets.QWidget):

    def __init__(
        self, width_fraction=1 / 3, gradient_radius=15, on_pause=None, on_continue=None
    ):
        super().__init__()
        self.on_pause = on_pause
        self.on_continue = on_continue

        # Frameless, always on top
        self.setWindowFlags(
            QtCore.Qt.FramelessWindowHint
            | QtCore.Qt.WindowStaysOnTopHint
            | QtCore.Qt.Tool
        )
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)

        screen = QtWidgets.QApplication.primaryScreen().geometry()
        self.w = int(screen.width() * width_fraction)
        self.h = screen.height()
        x_target = screen.width() - self.w
        y = 0
        self.target_rect = QtCore.QRect(x_target, y, self.w, self.h)
        self.setGeometry(self.target_rect)

        # Gradient background
        self.background = GradientFrame(self.w, self.h, gradient_radius, self)

        btn_size = 24
        margin_btn = 10

        # Pause button
        self.pause_btn = QtWidgets.QPushButton(self)
        icon_path = (
            get_root_path()
            / "cognition_layer"
            / "experts"
            / "code_interpreter"
            / "media"
        )
        icon = QtGui.QIcon(str(icon_path / "pause.png"))
        self.pause_btn.setIcon(icon)
        self.pause_btn.setIconSize(QtCore.QSize(btn_size * 2, btn_size * 2))
        self.pause_btn.setFixedSize(btn_size, btn_size)
        self.pause_btn.move(self.w - 2 * btn_size - 2 * margin_btn, margin_btn)
        self.pause_btn.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.pause_btn.clicked.connect(self._handle_pause)

        # Continue
        self.continue_ = QtWidgets.QPushButton(self)
        icon = QtGui.QIcon(str(icon_path / "continue.png"))
        self.continue_.setIcon(icon)
        self.continue_.setIconSize(QtCore.QSize(btn_size * 2, btn_size * 2))
        self.continue_.setFixedSize(btn_size, btn_size)
        self.continue_.move(self.w - 3 * btn_size - 3 * margin_btn, margin_btn)
        self.continue_.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.continue_.clicked.connect(self._handle_continue)

        # Close button
        self.close_btn = QtWidgets.QPushButton("✕", self)
        self.close_btn.setFixedSize(btn_size, btn_size)
        self.close_btn.move(self.w - btn_size - margin_btn, margin_btn)
        self.close_btn.setStyleSheet(
            "QPushButton { background: rgba(0,0,0,0); color: white; border: none; font-size: 16pt; }"
            "QPushButton:hover { color: #ff5555; }"
        )
        self.close_btn.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.close_btn.clicked.connect(self.finish)

        # Markdown buffer and load flag
        self._md_buffer = []
        self._loaded = False

        # Setup transparent Web view
        self.browser = QWebEngineView(self)
        side_margin = 15
        top_margin = btn_size + margin_btn + 5  # move text down below button
        indent = int(self.w * 0.1)
        self.browser.setGeometry(
            side_margin + indent,
            top_margin,
            self.w - 2 * side_margin - indent,
            self.h - top_margin - side_margin,
        )
        self.browser.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.browser.setStyleSheet("background: transparent; border: none;")
        self.browser.page().setBackgroundColor(QtGui.QColor(0, 0, 0, 0))
        self.browser.loadFinished.connect(self._on_load)
        self._load_initial()

    def _handle_pause(self):
        if callable(self.on_pause):
            self.on_pause()

    def _handle_continue(self):
        if callable(self.on_continue):
            self.on_continue()

    def _on_load(self, ok):
        if ok:
            self._loaded = True

    def _load_initial(self):
        ignore = "{ignoreUnescapedHTML: true}"
        html = f"""
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'>
  <link rel='stylesheet' href='{HIGHLIGHT_JS_CSS}'>
  <style>
    body {{ margin:0; padding:0; background:transparent; color:#c9d1d9; font-family:'Segoe UI'; }}
    pre {{ overflow-x:auto; padding:8px; border-radius:4px; background:#161b22; }}
    code {{ font-family:Consolas, monospace; background:#161b22; padding:2px 4px; border-radius:3px; }}
  </style>
  <script src='{HIGHLIGHT_JS_SCRIPT}'></script>
  <script>hljs.configure({ignore});</script>
</head>
<body id='content'></body>
</html>
"""
        self.browser.setHtml(html)

    def ask_yes_no(self) -> Literal["yes", "no", "yes-all"]:
        no_btn = QtWidgets.QPushButton("No", self)
        yes_btn = QtWidgets.QPushButton("Si", self)
        yes_to_all_btn = QtWidgets.QPushButton("Si a todo", self)

        common_style = """
            QPushButton {
                background: rgba(0,0,0,180);
                color: white;
                border-radius: 5px;
                font-size: 12pt;
            }
            QPushButton:hover {
                background: rgba(255,0,0,180);
            }
        """
        for btn in (yes_btn, no_btn, yes_to_all_btn):
            btn.setFixedSize(90, 30)
            btn.setFlat(True)
            btn.setStyleSheet(common_style)
            btn.show()

        spacing = 10
        total_w = (
            yes_btn.width() + no_btn.width() + yes_to_all_btn.width() + spacing * 2
        )
        x0 = (self.w - total_w) // 2
        y0 = 10
        yes_btn.move(x0, y0)
        no_btn.move(x0 + yes_btn.width() + spacing, y0)
        yes_to_all_btn.move(x0 + yes_btn.width() + no_btn.width() + spacing * 2, y0)

        loop = QtCore.QEventLoop()
        result = {"value": None}

        yes_btn.clicked.connect(
            lambda: (result.__setitem__("value", "yes"), loop.quit())
        )
        no_btn.clicked.connect(lambda: (result.__setitem__("value", "no"), loop.quit()))
        yes_to_all_btn.clicked.connect(
            lambda: (result.__setitem__("value", "yes-all"), loop.quit())
        )

        original_styles = {
            yes_btn: common_style,
            no_btn: common_style,
            yes_to_all_btn: common_style,
        }
        blink_style = """
            QPushButton {
                background: rgba(150,0,0, 100);
                color: white;
                border-radius: 5px;
                font-size: 12pt;
            }
        """

        def restore_styles():
            for btn, style in original_styles.items():
                btn.setStyleSheet(style)

        def blink():
            for btn in original_styles:
                time.sleep(0.3)
                btn.setStyleSheet(blink_style)
            QtCore.QTimer.singleShot(200, restore_styles)

        timer = QtCore.QTimer(self)
        timer.setInterval(3000)  # cada 2 segundos
        timer.timeout.connect(blink)
        timer.start()

        loop.exec_()

        timer.stop()
        for btn in (yes_btn, no_btn, yes_to_all_btn):
            btn.deleteLater()
        return result["value"]

    def write_stream(self, msg: dict):
        if "content" in msg:
            msg["content"] = str(msg["content"])
        if not self._loaded:
            return
        t = msg.get("type")
        if t == "code":
            if msg.get("start"):
                self._md_buffer.append(f"\n```{msg.get('format','')}\n")
            elif msg.get("end"):
                self._md_buffer.append("\n```\n")
            else:
                self._md_buffer.append(msg.get("content", ""))
        elif t == "console":
            if msg.get("start"):
                self._md_buffer.append("\n```console\n")
            elif msg.get("end"):
                self._md_buffer.append("\n```\n")
            else:
                self._md_buffer.append(msg.get("content", ""))
        else:
            self._md_buffer.append(msg.get("content", ""))

        full_md = "".join(self._md_buffer)
        if full_md.count("\n```\n") % 2 == 1:
            full_md += "\n```"
        body_html = markdown.markdown(
            full_md, extensions=["extra", "codehilite"], output_format="html5"
        )
        safe_html = body_html.replace("`", "\\`")
        js = f"(function(){{document.getElementById('content').innerHTML=`{safe_html}`;hljs.highlightAll();}})()"
        self.browser.page().runJavaScript(js)

    def slide_in(self, duration=1500):
        """
        Anima la aparición de la ventana desde fuera de pantalla (derecha) hasta su target_rect.
        """
        # Rectángulo inicial (justo fuera de la pantalla, a la derecha)
        start_rect = QtCore.QRect(
            self.target_rect.right(), self.target_rect.y(), self.w, self.h
        )
        anim = QtCore.QPropertyAnimation(self, b"geometry")
        anim.setDuration(duration)
        anim.setStartValue(start_rect)
        anim.setEndValue(self.target_rect)
        anim.setEasingCurve(QtCore.QEasingCurve.OutCubic)
        anim.start()
        # Para que no se recoja el GC
        self._slide_anim = anim

    def slide_out(self, duration=1000):
        """
        Anima la desaparición de la ventana desplazándola hacia la derecha,
        y al acabar cierra el widget.
        """
        # Rectángulo final (fuera de la pantalla, a la derecha)
        end_rect = QtCore.QRect(
            self.target_rect.right(), self.target_rect.y(), self.w, self.h
        )
        anim = QtCore.QPropertyAnimation(self, b"geometry")
        anim.setDuration(duration)
        anim.setStartValue(self.geometry())
        anim.setEndValue(end_rect)
        anim.setEasingCurve(QtCore.QEasingCurve.InCubic)
        # Al terminar, cerrar la ventana
        anim.finished.connect(self.close)
        anim.start()
        self._slide_anim = anim

    def finish(self):
        self.close()


# Public API
_app = None
_window = None


def start(on_pause=lambda: None, on_continue=lambda: None):
    global _app, _window
    if not QtWidgets.QApplication.instance():
        _app = get_app()
    else:
        _app = QtWidgets.QApplication.instance()

    _window = OverlayMarkdownViewer(on_pause=on_pause, on_continue=on_continue)
    _window.show()
    _window.slide_in()
    return _app, _window


def finish():
    if _window:
        _window.slide_out(duration=500)
        _window._slide_anim.finished.connect(lambda: _app.quit())
    else:
        if _app:
            _app.quit()
