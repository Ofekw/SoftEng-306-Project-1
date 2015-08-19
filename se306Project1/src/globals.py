from collections import deque

def init():
    global picker_queue
    picker_queue = deque([])

    global targeted_pickers
    targeted_pickers = []