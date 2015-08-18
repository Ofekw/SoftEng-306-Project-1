from collections import deque
import threading


def init():
    global picker_queue
    picker_queue = deque([])

    global targeted_pickers
    targeted_pickers = []

    global globals_lock
    globals_lock = threading.RLock()