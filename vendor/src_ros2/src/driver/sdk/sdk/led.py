#!/usr/bin/python3
# coding=utf8
import Jetson.GPIO as GPIO

LED_PIN = 24  # Pin number corresponding to LED (LED对应引脚号)

mode = GPIO.getmode()
if mode == 1 or mode is None:  # Whether the pin code is set(是否已经设置引脚编码)
    GPIO.setmode(GPIO.BCM)  # Set as BCM code(设为BCM编码)

GPIO.setwarnings(False)  # Close alarm print(关闭警告打印)

GPIO.setup(LED_PIN, GPIO.OUT)  # Set pin as output mode(设置引脚为输出模式)

def on():
    GPIO.output(LED_PIN, 0)

def off():
    GPIO.output(LED_PIN, 1)

def set(new_state):
    GPIO.output(LED_PIN, new_state)

if __name__ == "__main__":
    import time
    while True:
        on()
        time.sleep(1)
        off()
        time.sleep(1)
