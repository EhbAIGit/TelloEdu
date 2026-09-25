from pynput.keyboard import Key, Listener
import threading

keys_pressed = set()

def on_press(key):
    try:
        if hasattr(key, 'char') and key.char:
            keys_pressed.add(key.char.lower())
        else:
            keys_pressed.add(str(key))
    except AttributeError:
        keys_pressed.add(str(key))

def on_release(key):
    try:
        if hasattr(key, 'char') and key.char:
            keys_pressed.discard(key.char.lower())
        else:
            keys_pressed.discard(str(key))
    except AttributeError:
        keys_pressed.discard(str(key))

def init():
    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()
    # Keep the listener running in a separate thread
    threading.Thread(target=listener.join, daemon=True).start()

def getKey(key):
    key_lower = key.lower()
    if key_lower in keys_pressed:
        return True
    # Map special keys
    key_mappings = {
        'left': '<Key.left>',
        'right': '<Key.right>',
        'up': '<Key.up>',
        'down': '<Key.down>',
        'space': '<Key.space>',
        'enter': '<Key.enter>',
        'esc': '<Key.esc>',
        'q': 'q',
        'w': 'w',
        'a': 'a',
        's': 's',
        'd': 'd'
    }
    mapped = key_mappings.get(key_lower, key)
    return mapped in keys_pressed