from pynput.keyboard import Key, Listener
import threading

# 用于存储当前按下的按键
current_keys = set()

def on_press(key):
    try:
        current_keys.add(key.char)  # 将按下的字符添加到集合中
    except AttributeError:
        current_keys.add(key)  # 对于特殊按键（如空格、方向键等）

    if 'w' in current_keys and len(current_keys) == 1:
        print("Both 'W' ")
    if 'a' in current_keys and len(current_keys) == 1:
        print("Both 'A' ")
    # 检查是否同时按下了 'w' 和 'a'
    if 'w' in current_keys and 'a' in current_keys:
        print("Both 'w' and 'a' are pressed simultaneously.")

def on_release(key):
    try:
        current_keys.remove(key.char)  # 从集合中移除释放的按键字符
    except AttributeError:
        current_keys.remove(key)  # 对于特殊按键的处理

    if key == Key.esc:
        return False  # 结束监听

# 启动监听
with Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
