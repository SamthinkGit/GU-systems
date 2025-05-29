import sys

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QCheckBox
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget


class NovaUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Nova")
        self.setStyleSheet(self._style())
        self.setMinimumSize(300, 200)
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignTop)

        # Título
        title = QLabel("Nova")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 24px; font-weight: bold;")
        layout.addWidget(title)

        # Checkbox modo localhost
        self.localhost_checkbox = QCheckBox("Modo localhost")
        self.localhost_checkbox.stateChanged.connect(self.on_localhost_toggle)
        layout.addWidget(self.localhost_checkbox)

        # Botón de sincronización
        self.sync_button = QPushButton("Sincronizar con Nova")
        self.sync_button.clicked.connect(self.on_sync_clicked)
        layout.addWidget(self.sync_button)

        # Botón de iniciar
        start_button = QPushButton("Iniciar")
        start_button.clicked.connect(self.on_start_clicked)
        layout.addWidget(start_button)

        self.setLayout(layout)

    def on_localhost_toggle(self, state):
        is_localhost = state == Qt.Checked
        print(f"[Callback] Modo localhost: {'Activado' if is_localhost else 'Desactivado'}")

    def on_sync_clicked(self):
        # Aquí iría tu lógica real de sincronización
        success = True  # Simulación
        if success:
            QMessageBox.information(self, "Sincronización", "¡Sincronizado con éxito!")
            print("[Callback] Sincronización completada")
        else:
            QMessageBox.critical(self, "Error", "No se pudo sincronizar")
            print("[Callback] Error en la sincronización")

    def on_start_clicked(self):
        print("[Callback] Iniciando proceso Nova...")

    def _style(self):
        return """
            QWidget {
                background-color: #1e1e1e;
                color: #ffffff;
                font-family: Arial;
            }
            QPushButton {
                background-color: #2c2c2c;
                border: 1px solid #555;
                padding: 10px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #444;
            }
            QCheckBox {
                font-size: 16px;
                margin: 10px 0;
            }
        """


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = NovaUI()
    window.show()
    sys.exit(app.exec_())
