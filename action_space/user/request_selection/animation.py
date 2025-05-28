from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog
from PyQt5.QtWidgets import QDialogButtonBox
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QVBoxLayout

from action_space.tools.pyqt_utils import get_app


class OptionSelector(QDialog):
    def __init__(self, message, options):
        super().__init__()
        self.setWindowTitle("Selecciona una opción")
        self.setStyleSheet(dark_style())
        self.setMinimumWidth(400)

        self.selected_option = None
        layout = QVBoxLayout()

        label = QLabel(message)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size: 18px; font-weight: bold; margin-bottom: 12px;")
        layout.addWidget(label)

        for option in options:
            button = QPushButton(option)
            button.setProperty("class", "optionButton")
            button.clicked.connect(lambda _, opt=option: self.choose_option(opt))
            layout.addWidget(button)

        # Botón para "Otros"
        other_button = QPushButton("Otros...")
        other_button.setProperty("class", "otherButton")
        other_button.clicked.connect(self.choose_other)
        layout.addWidget(other_button)

        self.setLayout(layout)

    def choose_option(self, option):
        self.selected_option = option
        self.accept()

    def choose_other(self):
        text, ok = TextInputDialog.get_text("Introduce tu opción")
        if ok and text:
            self.selected_option = text
            self.accept()

    @staticmethod
    def get_selection(message, options):
        dialog = OptionSelector(message, options)
        result = dialog.exec_()
        return dialog.selected_option if result == QDialog.Accepted else None


class TextInputDialog(QDialog):
    def __init__(self, title):
        super().__init__()
        self.setWindowTitle(title)
        self.setStyleSheet(dark_style())

        layout = QVBoxLayout()
        self.input = QLineEdit()
        self.input.setStyleSheet("font-size: 16px; padding: 8px;")
        layout.addWidget(self.input)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        layout.addWidget(buttons)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        self.setLayout(layout)

    @staticmethod
    def get_text(title):
        dialog = TextInputDialog(title)
        result = dialog.exec_()
        return dialog.input.text(), result == QDialog.Accepted


def dark_style():
    return """
        QWidget {
            background-color: #1e1e1e;
            color: #f0f0f0;
            font-size: 16px;
        }

        QPushButton[class="optionButton"], QPushButton[class="otherButton"] {
            background-color: #2c2c2c;
            border: 1px solid #444;
            border-radius: 10px;
            padding: 10px;
            margin: 6px 0;
        }

        QPushButton[class="optionButton"]:hover, QPushButton[class="otherButton"]:hover {
            background-color: #3a3a3a;
        }

        QLineEdit {
            background-color: #2a2a2a;
            border: 1px solid #555;
            border-radius: 6px;
            color: #fff;
        }

        QDialogButtonBox QPushButton {
            background-color: #444;
            color: white;
            border: 1px solid #666;
            border-radius: 6px;
            padding: 6px 12px;
        }

        QDialogButtonBox QPushButton:hover {
            background-color: #555;
        }
    """


def request_user_input(message: str, options: list[str]) -> str:
    app = get_app()  # noqa
    selection = OptionSelector.get_selection(message, options)
    return selection
