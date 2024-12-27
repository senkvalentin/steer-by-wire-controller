import socket
import tkinter as tk
from threading import Thread

#Pygame Setup
from rpi_hardware_pwm import HardwarePWM
import pygame
import RPi.GPIO as GPIO
import time

# Initialisierung des Servo-Pins und des PWM-Signals

#servoPin2 = 13
frontlight = 17
backlight = 27
blinkLeft = 26
blinkRight = 16
steering_angle = 0
motorControlPin = 22
blinker_active_left = False
blinker_active_right = False

GPIO.setmode(GPIO.BCM)
#GPIO.setup(servoPIN, GPIO.OUT)
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
pwm.start(50) # full duty cycle
pwm1 = HardwarePWM(pwm_channel=1, hz=60, chip=0)
pwm1.start(50) # full duty cycle


pygame.init()
# Taster entprellen



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


def handle_client_connection(client_socket):
    global received_message
    try:
        while True:
            message = client_socket.recv(1024).decode('utf-8')
            if message:
                print(f"Nachricht empfangen: {message}")
                
                received_message = message  # Nachricht speichern
                #print(received_message)
                
            else:
                break
    except Exception as e:
        print(f"Fehler beim Empfangen der Nachricht: {e}")
    finally:
        client_socket.close()

# Funktion zum Akzeptieren neuer Verbindungen

def accept_connections(server_socket):
    while True:
        client_socket, addr = server_socket.accept()
        print(f"Verbindung von {addr}")
        client_thread = Thread(target=handle_client_connection, args=[client_socket])
        client_thread.daemon = True
        client_thread.start()
        

# Funktion zum Setzen des Servo-Winkels und Anzeigen des Duty Cycle
def set_servo_angle(steering_angle, gegensaetzlich, vorderrad, hinterrad, gewichtung):
    global limitAngle
    gewichtung = 450/90
    limitAngle = 9
    onTimeDiff = 0.8 #millisekunden
    onTime = 3 #millisekunden
    # Lenkradwinkelbereich: -450° bis 450°
    # Servo-Winkelbereich: -90° bis 90°
    # Servo-PWM Frequenz 238 - 555 Hz = +-222Hz
    # Servo-PWM DutyCycle = 50%

    # Berechne den Servo-Winkel aus dem Lenkradwinkel
    servo_angle = (steering_angle / 450 * gewichtung) # Umrechnung von -450° bis 450° auf -90° bis 90°
    # Begrenzung des Winkels auf -90° bis 90°
    servo_angle = max(min(servo_angle,1), -1)
    # Berechnung des Duty Cycles basierend auf der Gewichtung
    frequency_PWM = 1/(onTime + (servo_angle) * onTimeDiff) * 1e3 # Calculation Steering Angle to PWM Frequency
    frequency_PWM = max(min(frequency_PWM, 555), 238)  # Frequenz limit from 238-555Hz
    if gegensaetzlich:
        frequency_PWM_inverse = 1/(onTime + (-servo_angle) * onTimeDiff) * 1e3
        frequency_PWM_inverse = max(min(frequency_PWM_inverse, 555), 238)  # Invertiert den Duty Cycle für den zweiten Servo; 7 ist der Mittelpunkt vom Servo/Ausgangsstellung
    else:
        frequency_PWM_inverse = frequency_PWM       
        
    if vorderrad:
        #GPIO.output(servoPIN, True)
        pwm1.change_frequency(frequency_PWM)
        pwm.change_frequency(333)  # Zweiten Servo deaktivieren
    elif hinterrad:
        #GPIO.output(servoPin2, True)
        pwm.change_frequency(frequency_PWM)
        pwm1.change_frequency(333)  # Ersten Servo deaktivieren
    else:
        #GPIO.output(servoPIN, True)       
        #GPIO.output(servoPin2, True)
        pwm.change_frequency(frequency_PWM)
        pwm1.change_frequency(frequency_PWM_inverse)
   
    return servo_angle*60/1.2*onTimeDiff, frequency_PWM


def blinkControlLeft():
    global steering_angle, blinker_active_left, blinker_active_right
    counterBlink = 1
    while True:
        if blinker_active_left:
            start_steering_angle = steering_angle
            
            if start_steering_angle >= 0:
                
                while steering_angle <= start_steering_angle:
                    if steering_angle <= 0:
                        start_steering_angle = 0
                    GPIO.output(blinkLeft, 1)
                    time.sleep(0.5)
                    GPIO.output(blinkLeft, 0)
                    time.sleep(0.5)
                    
            GPIO.output(blinkLeft, 0)  
            blinker_active_left = False
            
            if start_steering_angle < 0:
                while steering_angle <= start_steering_angle:  
                    GPIO.output(blinkLeft, 1)
                    time.sleep(0.5)
                    GPIO.output(blinkLeft, 0)
                    time.sleep(0.5)

            GPIO.output(blinkLeft, 0)  
            blinker_active_left = False
        if blinker_active_right:
            start_steering_angle = steering_angle
            
            if start_steering_angle <= 0:
               
                while steering_angle >= start_steering_angle:
                    if steering_angle >= 0:
                        start_steering_angle = 0
                    GPIO.output(blinkRight, 1)
                    time.sleep(0.5)
                    GPIO.output(blinkRight, 0)
                    time.sleep(0.5)
            GPIO.output(blinkRight, 0)
            blinker_active_right = False

            if start_steering_angle > 0:
                while steering_angle >= start_steering_angle:
                    GPIO.output(blinkRight, 1)
                    time.sleep(0.5)
                    GPIO.output(blinkRight, 0)
                    time.sleep(0.5)
            GPIO.output(blinkRight, 0)
            blinker_active_right = False
        time.sleep(0.1)  


def pygameLoop():
    global steering_angle, blinker_active_left, blinker_active_right
    global gewichtung  # Verwende die globale Variable für Gewichtung
    global counter
    counter = 0
    
    #Taster entprellen
    state = 0
    ped = 0
    ned = 0
    sw = 0
# Setze die Breite und Höhe des Bildschirms (Breite, Höhe) und benenne das Fenster
    screen = pygame.display.set_mode((500, 700))
    pygame.display.set_caption("Joystick example")
# Verwendet, um zu verwalten, wie schnell der Bildschirm aktualisiert wird
    clock = pygame.time.Clock()
   # Diese Klasse hilft beim Ausdrucken auf den Bildschirm
    class TextPrint:
        def __init__(self):
            self.reset()
            self.font = pygame.font.Font(None, 25)
        def tprint(self, screen, text):
            text_bitmap = self.font.render(text, True, (0, 0, 0))
            screen.blit(text_bitmap, (self.x, self.y))
            self.y += self.line_height
        def reset(self):
            self.x = 10
            self.y = 10
            self.line_height = 15
        def indent(self):
            self.x += 10
        def unindent(self):
            self.x -= 10
   # Bereit zum Drucken
    text_print = TextPrint()
   # Dieses Dict kann so belassen werden, da pygame bei Programmstart ein pygame.JOYDEVICEADDED-Event für jedes angeschlossene Joystick generiert
    joysticks = {}
    done = False
# Initialisierung der Lenkungsart
    gegensaetzlich = False
    vorderrad = False
    hinterrad = False
    gewichtung = 450 / 450
    limitAngle = 9
    dutyAlt = 7
    steering_angle_alt = 0
    servo_angle = 0
    last_button_time = 0  
    

    

    while not done:
    # Schritt zur Ereignisverarbeitung
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True  # Flag, dass wir fertig sind, damit wir diese Schleife verlassen
           # Hotplugging behandeln
            if event.type == pygame.JOYDEVICEADDED:
                joy = pygame.joystick.Joystick(event.device_index)
                joysticks[joy.get_instance_id()] = joy
                print(f"Joystick {joy.get_instance_id()} connected")
            if event.type == pygame.JOYDEVICEREMOVED:
                del joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} disconnected")
       # Zeichenschritt
        screen.fill((255, 255, 255))
        text_print.reset()
       # Anzahl der Joysticks abrufen.
        joystick_count = pygame.joystick.get_count()
       #text_print.tprint(screen, f"Number of joysticks: {joystick_count}")
        text_print.indent()
       # Für jeden Joystick:
        
        for joystick in joysticks.values():
            jid = joystick.get_instance_id()
           #text_print.tprint(screen, f"Joystick {jid}")
            text_print.indent()
           # Achsen laufen normalerweise paarweise, hoch/runter für eine, und links/rechts für die andere. Trigger zählen als Achsen.
            axes = joystick.get_numaxes()
            text_print.tprint(screen, f"Number of axes: {axes}")
            text_print.indent()
            for i in range(axes):
                axis = joystick.get_axis(i)
                
                
                if i == 0:  # Annahme, dass die Lenkachse die erste Achse ist
                     
                    steering_angle = axis * 450  # Lenkachse auf -450 bis 450 Grad skalieren
                    #if abs(steering_angle - steering_angle_alt) > 3:
                    servo_angle, duty = set_servo_angle(steering_angle, gegensaetzlich, vorderrad, hinterrad, gewichtung)
                    #steering_angle_alt = steering_angle
                        #print("drinnen")
                    #else:
                        #print("lalala")
                        
                    text_print.tprint(screen, f"Steering wheel angle: {steering_angle:>6.2f}°")
                 
                    text_print.tprint(screen, f"Servo angle:                {max(min(servo_angle * gewichtung, 90), -90):>6.2f}°")
                    text_print.tprint(screen, f"PWM Duty Cycle:          {duty:>5.2f}%")
                
                if i == 2:   # Annahme, dass die dritte Achse das Gaspedal ist
                    if axisBreak < 90 and (((1 - axis) * 50) / 100 * 20) - (axisBreak / 100 * 20) >= 1:
                        throttle_value = (1 - axis) * 50  # Skalieren von -1 bis 1 auf 0 bis 100, umgekehrt
                    
                        pwmMotorControl.ChangeDutyCycle((throttle_value / 100 * 20) - (axisBreak / 100 * 20))
                        axisThrottle = joystick.get_axis(2)
                    
                        text_print.tprint(screen, f"Gas pedal value:       {throttle_value:>6.0f}%")
                    
                    else:
                        pwmMotorControl.ChangeDutyCycle(0)
                  
                elif i == 1:  # Annahme, dass die zweite Achse das Bremspedal ist
                    brake_value = (1 - axis) * 50  # Skalieren von -1 bis 1 auf 0 bis 100, umgekehrt
                    text_print.tprint(screen, f"Brake pedal value:   {brake_value:>6.0f}%")
                    if brake_value > 10:
                        GPIO.output(backlight, 1)
                    else:
                        GPIO.output(backlight, 0)
                    
                    axisBreak = (1 -  joystick.get_axis(1)) *50
                elif i == 3:  # Annahme, dass die vierte Achse das Kupplungspedal ist
                    clutch_value = (1 - axis) * 50  # Skalieren von -1 bis 1 auf 0 bis 100, umgekehrt
                    text_print.tprint(screen, f"Clutch pedal value:   {clutch_value:>6.0f}%")
                  
            text_print.unindent()
            buttons = joystick.get_numbuttons()
            text_print.tprint(screen, f"Number of buttons: {buttons}")
            text_print.indent()

            
            if joystick.get_button(2) == 1:
                        
                hinterrad = False
                vorderrad = False
                gegensaetzlich = False
                gewichtung = 450 / 90
            elif joystick.get_button(3) == 1:
                gegensaetzlich = True
                vorderrad = False
                hinterrad = False
                gewichtung = 450 / 180
                print("Offroad-Mode aktiviert")
            elif joystick.get_button(4) == 1:
                gegensaetzlich = False
                vorderrad = True
                hinterrad = False
                gewichtung = 450/ 450
                print("Comfort-Mode aktiviert")
            elif joystick.get_button(5) == 1:
                #vorderrad = True
                #hinterrad = False
                print("Benutzerdefiniert")
            
            for i in range(buttons):
                button = joystick.get_button(i)
                text_print.tprint(screen, f"Button {i:>2} value: {button}")
               # Überprüfen, ob die Tasten 6, 7, 8 oder 9 gedrückt sind und die Lenkungsart entsprechend einstellen
                
                

                sw = joystick.get_button(7)
                swBlinkLeft = joystick.get_button(0)
            
               
                debounce_time = 0.2


                current_time = time.time()

                '''
                if received_message == 'Sport-Mode':
                        
                    hinterrad = False
                    vorderrad = False
                    gegensaetzlich = False
                    gewichtung = 450 / 90
                elif received_message == 'Offroad-Mode':
                    gegensaetzlich = True
                    vorderrad = False
                    hinterrad = False
                    gewichtung = 450 / 180
                    print("Offroad-Mode aktiviert")
                elif received_message == 'Comfort-Mode':
                    gegensaetzlich = False
                    vorderrad = True
                    hinterrad = False
                    gewichtung = 450/ 450
                    print("Comfort-Mode aktiviert")
                elif received_message == 'Benutzerdefiniert':
                    #vorderrad = True
                    #hinterrad = False
                    print("Benutzerdefiniert")
                '''
                if button:
                    if sw and (current_time - last_button_time > debounce_time):
   
                        last_button_time = current_time
                        
                        ped = 1
                        ned = 0
                        state = 1
                    else:
                        if sw == 0:
                            ped = 0
                            ned = 1
                            state = 0
                    
                    if ped:
                        ped = 0
                        
                        counter = counter + 1
                        
                        time.sleep(0.02)
                        
                        
                    if ned:
                        ned = 0
                        
                        time.sleep(0.02)
                        
                    if counter % 2 != 0:
                        GPIO.output(frontlight, 1)
                    else:
                        GPIO.output(frontlight, 0)

                    if joystick.get_button(0) and not blinker_active_left and not blinker_active_right:
                        blinker_active_left = True
                    elif joystick.get_button(1) and not blinker_active_left and not blinker_active_right:
                        blinker_active_right = True 
                    
            text_print.unindent()
            text_print.unindent()
            hats = joystick.get_numhats()
            text_print.tprint(screen, f"Number of hats: {hats}")
            
            text_print.indent()
           # Hutposition. Alles oder nichts für die Richtung, kein Float wie get_axis(). Position ist ein Tupel aus int-Werten (x, y).
            for i in range(hats):
                hat = joystick.get_hat(i)
                text_print.tprint(screen, f"Hat {i} value: {hat}")
            text_print.tprint(screen, f"")
            
            text_print.tprint(screen, f"")


            text_print.tprint(screen, f"Modus: {received_message}")
            text_print.unindent()
            text_print.unindent()
       # Aktualisieren Sie den Bildschirm mit dem, was wir gezeichnet haben.
        pygame.display.flip()

            

       # Begrenzen Sie die Bildwiederholrate auf 20 Frames pro Sekunde
        clock.tick(20)
   # Beenden Sie das Programm

    pygame.quit()
    GPIO.cleanup()

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Option zum Wiederverwenden des Ports setzen
    s.bind((HOST, PORT))
    s.listen()
    print(f"Warte auf Verbindungen auf {BROCKER}:{PORT}...")

    accept_thread = Thread(target=accept_connections, args= [s])
    accept_thread.daemon = True
    accept_thread.start()
    #accept_thread.join() #Hauptthread wartet, bis andere threads abgeschlossen wurden
   
    blinkThread = Thread(target=blinkControlLeft)
    blinkThread.start()

    pygameThread = Thread(target=pygameLoop)
    pygameThread.start()
    

if __name__ == "__main__":
   main()
   