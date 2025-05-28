import sys
from functools import cache

from PyQt5.QtWidgets import QApplication


@cache
def get_app():
    """
    Returns a singleton instance of QApplication.
    If no instance exists, it creates a new one using sys.argv.
    """
    return QApplication.instance() or QApplication(sys.argv)
