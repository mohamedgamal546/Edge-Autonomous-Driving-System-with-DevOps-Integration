"""
Pytest configuration — mocks hardware dependencies so tests
run on any machine without Raspberry Pi GPIO or a physical camera.
"""
import sys
from unittest.mock import MagicMock

# Mock RPi.GPIO so tests pass on non-Pi hardware
gpio_mock = MagicMock()
gpio_mock.BCM = 11
gpio_mock.OUT = 0
gpio_mock.IN  = 1
gpio_mock.HIGH = 1
gpio_mock.LOW  = 0
sys.modules["RPi"]       = MagicMock()
sys.modules["RPi.GPIO"]  = gpio_mock

# Mock paho.mqtt so tests don't need a broker
sys.modules["paho"]                    = MagicMock()
sys.modules["paho.mqtt"]               = MagicMock()
sys.modules["paho.mqtt.client"]        = MagicMock()
