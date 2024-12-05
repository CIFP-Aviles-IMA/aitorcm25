#Aqui poneis el Docdtring que querais
"""
Código en: https://www.printables.com/model/818975-compact-robot-arm-arduino-3d-printed/files.

Descripción del Script: Este script se encarga del control de un brazo robótico utilizando servomotores y un controlador PWM PCA9685 conectado a una placa Jetson. El brazo cuenta con varios servos para mover las articulaciones (hombro, codo, muñeca, base y garra) y sensores de posición (potenciómetros) que permiten ajustar su movimiento en tiempo real. Asimismo, dispone de un botón para abrir y cerrar la garra.

Requisitos:

Jetson.GPIO: Para manejar los pines GPIO de la placa Jetson.
adafruit_pca9685: Para la comunicación con el controlador PWM PCA9685.
adafruit_servokit: Para la gestión de los servomotores utilizando la librería de Adafruit.
time: Para incluir retrasos y configurar el sistema.

Funcionamiento:

Configuración de servos: Se inicializa el controlador PWM PCA9685, el cual se encarga de generar las señales PWM necesarias para mover los servos a las posiciones deseadas.
Lectura de potenciómetros: Se leen los valores de los potenciómetros conectados a los pines GPIO y se mapean estos valores a un rango adecuado para modificar los ángulos de los servos.
Control del servo: El valor obtenido de cada potenciómetro se convierte en un ancho de pulso PWM que regula el movimiento de los servos.
Control de la garra: Un botón conectado al pin GPIO 15 se utiliza para controlar la garra del brazo. Esta se cierra cuando el botón no está presionado y se abre cuando se presiona.

Funciones Principales:

moveMotor(controlIn, motorOut): Esta función lee el valor de un potenciómetro conectado a un pin GPIO y ajusta la posición del servo correspondiente según la lectura del potenciómetro.
Bucle principal: El script ejecuta un bucle continuo que ajusta las posiciones de los servos con base en los valores de los potenciómetros y gestiona el estado de la garra según el botón.

Parámetros de Configuración:

MIN_PULSE_WIDTH: Ancho de pulso mínimo (650 microsegundos).
MAX_PULSE_WIDTH: Ancho de pulso máximo (2350 microsegundos).
FREQUENCY: Frecuencia de la señal PWM (50 Hz).
"""
#import wire 
#import Adafruit_PWMServodriver
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_pca9685
import time
i2c = busio.I2C(broad.SCL, board.SDA)
from adafruit_servokit import Servokit

#Declaro variables globales
MIN_PULSE_WIDTH=    650
MAX_PULSE_WIDTH=    2350
FREQUENCY      =    50

#Instancio el Driver del controlador de servos
pwm = adafruit_pca9685.PCA9685("i2C")
kit = Servokit(channels=16)

#Configuro el SetUP
time.sleep(5)                       #<--- So I have time to get controller to starting position 
"pca.frequency" = FREQUENCY
GPIO.setmode(GPIO.BOARD)
hand = "adafruit_motot".servo.Servo(1)  #cualquiera de los dos
wrist = "adafruit_motor".servo.Servo(1)
elbow = "adafruit_motor".servo.Servo(2)
shoulder = "adafruit_motor".servo.Servo(3) 
base = "adafruit_motor".servo.Servo(4)
potWrist = "adafruit_motor".servo.Servo(5)
potElbow = "adafruit_motor".servo.Servo(6)
potShoulder = "adafruit_motor".servo.Servo(7)
potBase = "adafruit_motor".servo.Servo(8)

pwm.begin()
pwm.setPWMFrenq(FREQUENCY)
pwm.setPWM(32, 0,90)                 #Set  Gripper to 90 degrees (Close Gripper ) x en Jetson x=32
GPIO.setup(13, GPIO.IN)           # Channel tiene que ser un pin valido para jetson

def moveMotor(controlIn, motorOut ):

    pulse_wide, pulse_widht, potVal = -7

    """
    A continuación vamos a describir las funciones que estan relacionadas con la función de MoveMotor(controlIN, motorOUT):

    Args:
    ControlIN (int): El  valor del potenciometro se lee en el pin GPIO correspondiente,el cual esta determinado por el pin seleccionado.
    MotorOUT (int): El pin de salida del motor se utiliza para enviar la señal PWM al motor que se va a controlar

    Returns:
    Su función principal es devolver el robot a la posición que indique el valor del potenciometro """

 #   portVal= analogRead(controlIn);
    PORTvAL= GPIO.input(controlIn)
    pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096); 

    pwm = GPIO.PWM(motorOut, 0, pulse_widht)

while (True): 
    moveMotor(potWrist, wrist)
    moveMotor(potElbow, elbow)
    moveMotor("portShoulder", shoulder)
    moveMotor(portBase, base)
    pushButton = GPIO.input(13)
    if("pushBotton" == GPIO.LOW):

     pwm.setPWM(hand, 0, 10)
    print("Grab")
else:   

        pwm.setPWM(hand)
        print("Release")
    
GPIO.cleanup()
