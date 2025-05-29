import sys

from PyQt5.QtCore import Qt
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QCheckBox
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget


class NovaUI(QWidget):
    def __init__(
        self, on_sync=None, on_start=None, on_toggle_localhost=None, on_exit=None
    ):
        super().__init__()
        self.setWindowTitle("Nova")
        self.setStyleSheet(self._style())
        self.setMinimumSize(500, 300)

        self.on_sync_callback = on_sync
        self.on_start_callback = on_start
        self.on_toggle_callback = on_toggle_localhost
        self.on_exit_callback = on_exit

        self._init_ui()

    def _init_ui(self):
        self.layout = QVBoxLayout()
        self.layout.setAlignment(Qt.AlignTop)
        self.layout.setSpacing(20)

        self.title = QLabel("Nova")
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setStyleSheet("font-size: 32px; font-weight: bold; margin: 20px;")
        self.layout.addWidget(self.title)

        self.localhost_checkbox = QCheckBox("Modo localhost")
        self.localhost_checkbox.setStyleSheet("font-size: 18px;")
        self.localhost_checkbox.stateChanged.connect(self._on_toggle_localhost)
        self.layout.addWidget(self.localhost_checkbox)

        self.sync_button = QPushButton("Sincronizar con Nova")
        self.sync_button.clicked.connect(self._on_sync_clicked)
        self.layout.addWidget(self.sync_button)

        self.start_button = QPushButton("Iniciar")
        self.start_button.clicked.connect(self._on_start_clicked)
        self.layout.addWidget(self.start_button)

        self.setLayout(self.layout)

    def _on_toggle_localhost(self, state):
        if self.on_toggle_callback:
            is_localhost = state == Qt.Checked
            self.on_toggle_callback(is_localhost)

    def _on_sync_clicked(self):
        self.sync_button.setEnabled(False)
        self.sync_button.setText("Sincronizando...")

        def finish_sync():
            message = "Sincronización completada con éxito"
            if self.on_sync_callback:
                message = self.on_sync_callback() or message
            QMessageBox.information(self, "Sincronización", message)
            self.sync_button.setEnabled(True)
            self.sync_button.setText("Sincronizar con Nova")

        QTimer.singleShot(100, finish_sync)

    def _on_start_clicked(self):
        # Cambiar estilo del botón y desactivar
        self.start_button.setText("Cargando...")
        self.start_button.setStyleSheet(
            "background-color: #00bcd4; color: black; font-size: 18px; padding: 14px; border: none; border-radius: 10px;"  # noqa
        )
        self.start_button.setEnabled(False)

        def finish_start():
            # Ocultar todo
            self.localhost_checkbox.hide()
            self.sync_button.hide()
            self.start_button.hide()

            # Callback
            if self.on_start_callback:
                self.on_start_callback()

            # Mostrar mensaje de espera
            self.status_label = QLabel("Esperando instrucciones de Nova...")
            self.status_label.setAlignment(Qt.AlignCenter)
            self.status_label.setStyleSheet("font-size: 20px; margin-top: 40px;")
            self.layout.addWidget(self.status_label)

        QTimer.singleShot(1000, finish_start)  # Simula carga

    def closeEvent(self, event):
        if self.on_exit_callback:
            self.on_exit_callback()
        event.accept()

    def _style(self):
        return """
            QWidget {
                background-color: #1e1e1e;
                color: #ffffff;
                font-family: Segoe UI;
            }
            QPushButton {
                background-color: #3a3a3a;
                color: #ffffff;
                font-size: 18px;
                padding: 14px;
                border: none;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #505050;
            }
            QCheckBox {
                font-size: 18px;
                padding: 10px;
            }
        """


# Ejemplo de uso
def sync_callback():
    print("[Callback] Sincronización ejecutada")
    return "¡Sincronización exitosa con Nova!"


def start_callback():
    print("[Callback] Proceso Nova iniciado")


def toggle_callback(state):
    print(f"[Callback] Modo localhost: {'Activado' if state else 'Desactivado'}")


def exit_callback():
    print("[Callback] Saliendo de Nova")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = NovaUI(
        on_sync=sync_callback,
        on_start=start_callback,
        on_toggle_localhost=toggle_callback,
        on_exit=exit_callback,
    )
    window.show()
    sys.exit(app.exec_())
