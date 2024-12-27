import socket
import tkinter as tk
from threading import Thread

# Pygame Setup
from rpi_hardware_pwm import HardwarePWM
import pygame
import RPi.GPIO as GPIO
import time

# Initialisierung des Servo-Pins und des PWM-Signals

# servoPin2 = 13
frontlight = 17
backlight = 27
blinkLeft = 26
blinkRight = 16
steering_angle = 0
motorControlPin = 22
blinker_active_left = False
blinker_active_right = False

GPIO.setmode(GPIO.BCM)
# GPIO.setup(servoPIN, GPIO.OUT)
GPIO.setup(frontlight, GPIO.OUT)
GPIO.setup(backlight, GPIO.OUT)
GPIO.setup(blinkLeft, GPIO.OUT)
GPIO.setup(blinkRight, GPIO.OUT)
GPIO.output(frontlight, 0)
GPIO.output(blinkRight, 0)
GPIO.output(blinkLeft, 0)

GPIO.setup(motorControlPin, GPIO.OUT)
pwmMotorControl = GPIO.PWM(motorControlPin, 50)  # 333 Hz
pwmMotorControl.start(0)

pwm = HardwarePWM(pwm_channel=0, hz=60, chip=0)
pwm.start(50)  # full duty cycle
pwm1 = HardwarePWM(pwm_channel=1, hz=60, chip=0)
pwm1.start(50)  # full duty cycle

pygame.init()

# Funktion zur Ermittlung der IP-Adresse
def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('172.20.10.6', 1))  # Beliebige IP-Adresse im Netzwerk
        IP = s.getsockname()[0]
    except Exception:
        IP = '172.20.10.2'
    finally:
        s.close()
    return IP

HOST = get_ip_address()
BROCKER = '172.20.10.6'
PORT = 65432

# Globale Variable zum Speichern der Nachricht
received_message = None
steering_mode = "Vorderradlenkung"

def handle_client_connection(client_socket):
    global received_message
    try:
        while True:
            message = client_socket.recv(1024).decode('utf-8')
            if message:
                print(f"Nachricht empfangen: {message}")
                received_message = message
            else:
                break
    except Exception as e:
        print(f"Fehler beim Empfangen der Nachricht: {e}")
    finally:
        client_socket.close()

def accept_connections(server_socket):
    while True:
        client_socket, addr = server_socket.accept()
        print(f"Verbindung von {addr}")
        client_thread = Thread(target=handle_client_connection, args=[client_socket])
        client_thread.daemon = True
        client_thread.start()

def set_servo_angle(steering_angle, gegensaetzlich, vorderrad, hinterrad, gewichtung):
    global limitAngle
    gewichtung = 450 / 90
    limitAngle = 9
    onTimeDiff = 0.8  # millisekunden
    onTime = 3  # millisekunden

    # Lenkradwinkel invertieren
    steering_angle = -steering_angle

    # Berechne den Servo-Winkel aus dem invertierten Lenkradwinkel
    servo_angle = (steering_angle / 450 * gewichtung)  # Umrechnung von -450° bis 450° auf -90° bis 90°
    servo_angle = max(min(servo_angle, 1), -1)
    frequency_PWM = 1 / (onTime + (servo_angle) * onTimeDiff) * 1e3
    frequency_PWM = max(min(frequency_PWM, 555), 238)

    if gegensaetzlich:
        frequency_PWM_inverse = 1 / (onTime + (-servo_angle) * onTimeDiff) * 1e3
        frequency_PWM_inverse = max(min(frequency_PWM_inverse, 555), 238)
    else:
        frequency_PWM_inverse = frequency_PWM

    if vorderrad:
        pwm1.change_frequency(frequency_PWM)
        pwm.change_frequency(333)
    elif hinterrad:
        pwm.change_frequency(frequency_PWM)
        pwm1.change_frequency(333)
    else:
        pwm.change_frequency(frequency_PWM)
        pwm1.change_frequency(frequency_PWM_inverse)

    return servo_angle * 60 / 1.2 * onTimeDiff, frequency_PWM

def blinkControlLeft():
    global steering_angle, blinker_active_left, blinker_active_right
    while True:
        if blinker_active_left:
            GPIO.output(blinkLeft, 1)
            time.sleep(0.5)
            GPIO.output(blinkLeft, 0)
            time.sleep(0.5)
        if blinker_active_right:
            GPIO.output(blinkRight, 1)
            time.sleep(0.5)
            GPIO.output(blinkRight, 0)
            time.sleep(0.5)
        time.sleep(0.1)

def pygameLoop():
    global steering_angle, blinker_active_left, blinker_active_right, steering_mode
    screen = pygame.display.set_mode((500, 700))
    pygame.display.set_caption("Joystick example")
    clock = pygame.time.Clock()

    font = pygame.font.Font(None, 36)

    done = False
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        screen.fill((255, 255, 255))

        # Anzeigen des Lenkmodus
        mode_text = font.render(f"Lenkmodus: {steering_mode}", True, (0, 0, 0))
        screen.blit(mode_text, (10, 10))

        pygame.display.flip()
        clock.tick(20)

    pygame.quit()
    GPIO.cleanup()

def main():
    global steering_mode

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()
    print(f"Warte auf Verbindungen auf {BROCKER}:{PORT}...")

    accept_thread = Thread(target=accept_connections, args=[s])
    accept_thread.daemon = True
    accept_thread.start()

    blinkThread = Thread(target=blinkControlLeft)
    blinkThread.start()

    pygameThread = Thread(target=pygameLoop)
    pygameThread.start()

if __name__ == "__main__":
    main()
