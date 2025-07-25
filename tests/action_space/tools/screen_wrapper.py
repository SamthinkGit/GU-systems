"""
Screen Wrapper
==============================
this module provides a full-screen window application that tracks mouse
clicks and keyboard events. it utilizes the tkinter library for the gui
and multiprocessing to handle events in a separate process. the events
are recorded using dataclasses to maintain structured data for each
event type.
"""
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from ecm.shared import get_logger
from multiprocessing import Manager
from multiprocessing import Process
from typing import List
from typing import Literal


# ======================= CLASSES ============================
@dataclass
class ClickEvent:
    """
    ClickEvent class
    -----------------
    Represents a mouse click event with associated details such as
    coordinates, button pressed, side of the screen, and timestamps
    for press and release actions.
    """

    x: int
    y: int
    button: Literal["Right", "Middle", "Left"]
    side: Literal["red", "blue"]
    press_time: datetime
    release_time: datetime


@dataclass
class KeyEvent:
    """
    KeyEvent class
    ---------------
    Represents a keyboard event with associated details such as the key
    pressed and timestamps for press and release actions.
    """

    key: str
    press_time: datetime
    release_time: datetime


class Tracker:
    """
    Tracker class
    -------------
    Manages the storage of click and key events using multiprocessing
    manager lists.
    """

    def __init__(self, manager):
        self.clicks: List[ClickEvent] = manager.list()
        self.keys: List[KeyEvent] = manager.list()


class FullScreenWindow:
    """
    FullScreenWindow class
    -----------------------
    Creates a full-screen tkinter window and handles the tracking of
    mouse clicks and keyboard events. It displays messages on the
    window and manages event bindings for user interactions.
    """

    _logger = get_logger("Window")

    def __init__(self, tracker: Tracker):
        self.tracker = tracker
        self.root = tk.Tk()
        self.root.attributes("-topmost", True)
        self.root.update()
        self.root.focus_force()

        self.root.attributes("-fullscreen", True)

        self.canvas = tk.Canvas(self.root)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.width = self.root.winfo_screenwidth()
        self.height = self.root.winfo_screenheight()

        self.canvas.create_rectangle(
            0, 0, self.width // 2, self.height, fill="blue", outline="blue"
        )
        self.canvas.create_rectangle(
            self.width // 2, 0, self.width, self.height, fill="red", outline="red"
        )

        self.current_click_time = {}
        self.current_key_time = {}
        self.current_text_id = None

        self.root.bind("<ButtonPress>", self.on_click_press)
        self.root.bind("<ButtonRelease>", self.on_click_release)
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)

        self.root.bind("<Escape>", lambda e: self.close())

    def show_message(self, text):
        if self.current_text_id is not None:
            self.canvas.delete(self.current_text_id)

        self.current_text_id = self.canvas.create_text(
            self.width // 2,
            self.height // 2,
            text=text,
            fill="white",
            font=("Arial", 50),
        )

        self.root.after(500, lambda: self.canvas.delete(self.current_text_id))

    def on_click_press(self, event):
        self.current_click_time[event.num] = datetime.now()

    def on_click_release(self, event):
        press_time = self.current_click_time.pop(event.num, datetime.now())
        side = "blue" if event.x < self.width // 2 else "red"
        click_event = ClickEvent(
            x=event.x,
            y=event.y,
            button=(
                "Left" if event.num == 1 else "Right" if event.num == 3 else "Middle"
            ),
            side=side,
            press_time=press_time,
            release_time=datetime.now(),
        )
        self.tracker.clicks.append(click_event)
        self.show_message("Click")

    def on_key_press(self, event):
        if event.keysym not in self.current_key_time:
            self.current_key_time[event.keysym] = datetime.now()

    def on_key_release(self, event):
        press_time = self.current_key_time.pop(event.keysym, datetime.now())
        key_event = KeyEvent(
            key=event.keysym, press_time=press_time, release_time=datetime.now()
        )
        self.tracker.keys.append(key_event)
        self._logger.debug(f"Pressed {key_event}")
        self.show_message(event.keysym)

    def run(self):
        self.root.mainloop()

    def close(self):
        self.root.destroy()


# ======================= UTILITIES ============================
def sync_window_with_tracker(tracker: Tracker):
    """Synchronizes the full-screen window with the event tracker."""
    app = FullScreenWindow(tracker)
    app.run()


# ======================= Example Usage ============================
if __name__ == "__main__":
    manager = Manager()
    tracker = Tracker(manager)  # Will be updated with the keys/clicks pressed

    p = Process(target=sync_window_with_tracker, args=(tracker,))
    p.start()

    print("Hello")
    time.sleep(10)

    p.kill()
