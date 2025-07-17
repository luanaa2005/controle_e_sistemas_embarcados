import RPi.GPIO as GPIO
import time

rele_pin = 18
botao_pin = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(rele_pin, GPIO.OUT)
GPIO.setup(botao_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Sistema começa ligado
GPIO.output(rele_pin, GPIO.HIGH)

print("Sistema ligado. Pressione o botão para desligar.")

try:
    while True:
        if GPIO.input(botao_pin) == GPIO.LOW:
            GPIO.output(rele_pin, GPIO.LOW)  # corta energia
            print("Energia cortada!")
            break
        time.sleep(0.1)

finally:
    GPIO.cleanup()
