# -- coding: utf-8 --
# board_config.py — конфигурация GPIO для Raspberry Pi и LoRa

import RPi.GPIO as GPIO
import spidev
import time

class BOARD:
    """ Board initialisation/teardown and pin configuration is kept here. """

    # Пины подключения LoRa — используем BCM нумерацию
    DIO0 = 4      # GPIO25
    DIO1 = 17     # GPIO24
    DIO2 = 18     # GPIO24
    RESET = 22    # GPIO22
    NSS = 8       # GPIO8 (CE0)
    
    MISO = 9
    MOSI = 10
    SCk = 11

    # Настройки диапазона
    low_band = True

    @staticmethod
    def SpiDev():
        """Возвращает SPI объект"""
        return spidev.SpiDev()

    @staticmethod
    def setup():
        """ Настройка GPIO """
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(BOARD.DIO0, GPIO.IN)
        GPIO.setup(BOARD.DIO1, GPIO.IN)
        GPIO.setup(BOARD.DIO2, GPIO.IN)
        GPIO.setup(BOARD.RESET, GPIO.OUT)
        GPIO.setup(BOARD.NSS, GPIO.OUT)

    @staticmethod
    def teardown():
        """ Очистка GPIO """
        GPIO.cleanup()
    @staticmethod
    def add_events(dio0=None, dio1=None, dio2=None):
        for pin in (dio0, dio1, dio2):
            if isinstance(pin, int):
                try:
                    GPIO.add_event_detect(pin, GPIO.RISING)
                except RuntimeError:
                    pass
