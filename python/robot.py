# -*- coding: utf-8 -*-
#MAGTRONICA

"""----------------IMPORTACIÓN DE TODAS LAS LIBRERÍAS NECESARIAS----------------------"""
#Importamos la libreria del Webiopi, será la encargada de controlar los GPIO
#Además de ser el servidor
import webiopi
#Importamos la librería de tiempo
import time
#Importamos los GPIO como xGPIO, con el fin de controlarlos externos al webiopi
import RPi.GPIO as GPIO
#Importamos la libreria que permite activar nuevos procesos
import subprocess
#Importamos la librería que nos permite manejar varios hilos
import threading
#Importamos la libreria del sistema
import sys
#Añadimos la ruta del proyecto donde están los scripts que importaremos
sys.path.append('/home/pi/mercury/libreria')
#Importamos la librería de PWM para la shield del servo
from Adafruit_PWM_Servo_Driver import PWM

"""----------------------------CONTROL DE LOS PINES GPIO------------------------------"""
#Colocamos los pines GPIO en modo BCM
GPIO.setmode(GPIO.BCM)


"""------------LOS PINES PARA EL CONTROL DE LOS MOTORES TODOS SON GPIO BCM------------"""
#Definimos el pin 22 GPIO BCM donde se encontrara el motors in1
motors_in1 = 23
#Colocamos el pin del motors in1 como salida
GPIO.setup(motors_in1, GPIO.OUT)
#Definimos el pin 22 GPIO BCM donde se encontrara el motors in1
motors_in2 = 24
#Colocamos el pin del motors in1 como salida
GPIO.setup(motors_in2, GPIO.OUT)
#Definimos el pin 4 GPIO BCM donde se encontrara el motord in1
motord_in1 = 27
#Colocamos el pin del motord in1 como salida
GPIO.setup(motord_in1, GPIO.OUT)
#Definimos el pin 17 GPIO BCM donde se encontrara el motord in2
motord_in2 = 18
#Colocamos el pin del motord in2 como salida
GPIO.setup(motord_in2, GPIO.OUT)
#Definimos el pin 18 GPIO BCM donde se encontrara el motori in1
motori_in1 = 15
#Colocamos el pin del motori in1 como salida
GPIO.setup(motori_in1, GPIO.OUT)
#Definimos el pin 27 GPIO BCM donde se encontrara el motori in2
motori_in2 = 4
#Colocamos el pin del motori in2 como salida
GPIO.setup(motori_in2, GPIO.OUT)



"""-------------------------------SWICHEO DE MOTORES------------------------------
----"""


def subirp():
	"""El robot se moverá hacia atras, en reversa"""
	GPIO.output(motors_in1, 1)
	GPIO.output(motors_in2, 0)
	spwm_motor.ChangeDutyCycle(100)
	
	
def parars():
	"""El robot se moverá hacia atras, en reversa"""
	GPIO.output(motors_in1, 0)
	GPIO.output(motors_in2, 0)

	
def bajarp():
	"""El robot se moverá hacia adelante"""
	GPIO.output(motors_in1, 0)
	GPIO.output(motors_in2, 1)
	spwm_motor.ChangeDutyCycle(100)
	

def reversa():
	"""El robot se moverá hacia atras, en reversa"""
	GPIO.output(motord_in1, 1)
	GPIO.output(motord_in2, 0)
	GPIO.output(motori_in1, 0)
	GPIO.output(motori_in2, 1)


def adelante():
	"""El robot se moverá hacia adelante"""
	GPIO.output(motord_in1, 0)
	GPIO.output(motord_in2, 1)
	GPIO.output(motori_in1, 1)
	GPIO.output(motori_in2, 0)


def stop():
	"""El robot se detiene"""
	GPIO.output(motord_in1, 0)
	GPIO.output(motord_in2, 0)
	GPIO.output(motori_in1, 0)
	GPIO.output(motori_in2, 0)


def derecha():
	"""El robot se moverá hacia la derecha"""
	GPIO.output(motord_in1, 0)
	GPIO.output(motord_in2, 1)
	GPIO.output(motori_in1, 0)
	GPIO.output(motori_in2, 1)


def izquierda():
	"""El robot se moverá hacia la izquierda"""
	GPIO.output(motord_in1, 1)
	GPIO.output(motord_in2, 0)
	GPIO.output(motori_in1, 1)
	GPIO.output(motori_in2, 0)


"""--------------------------ACELERACIÓN DE LOS MOTORES-------------------------------"""

smotor_pwm = 22
GPIO.setup(smotor_pwm, GPIO.OUT)
#Ponemos el pin 36 como salida
spwm_motor = GPIO.PWM(smotor_pwm, 5000)
#Ponemos el pin 36 en modo PWM y enviamos 50 pulsos por segundo
spwm_motor.start(50)


motor_pwm = 17
GPIO.setup(motor_pwm, GPIO.OUT)
#Ponemos el pin 36 como salida
pwm_motor = GPIO.PWM(motor_pwm, 1000)
#Ponemos el pin 36 en modo PWM y enviamos 50 pulsos por segundo
pwm_motor.start(50)

xmotor_pwm = 14
GPIO.setup(xmotor_pwm, GPIO.OUT)
#Ponemos el pin 36 como salida
xpwm_motor = GPIO.PWM(xmotor_pwm, 1000)
#Ponemos el pin 36 en modo PWM y enviamos 50 pulsos por segundo
xpwm_motor.start(50)


def acelerar(valora):
	"""Controla la velocidad de los motores"""
	#Toma el valor recibido del deslizador en la página respecto a la aceleración
	#Convierte ese valor de String a entero
	tx = int(valora)
	pwm_motor.ChangeDutyCycle(tx)
	xpwm_motor.ChangeDutyCycle(tx)

	
def acelerars(velevar):
	"""Controla la velocidad de los motores"""
	#Toma el valor recibido del deslizador en la página respecto a la aceleración
	#Convierte ese valor de String a entero
	vs = int(velevar)
	spwm_motor.ChangeDutyCycle(vs)


"""--------------------------CONTROL DE LOS SERVOMOTORES------------------------------"""
#Se inicia la fución pwm en la ruta 0 de I2C que sería 0x40
pwm = PWM(0x40)
#Se establece la frecuencia a 50 Hz
freq = 50
#Debido a tener una frecuencia de 50Hz, el periodo es de 20ms
#periodo = 0.020
#Colocamos los pulsos del pwm al valor de la frecuencia
pwm.setPWMFreq(freq)


"""-------------CANVAS JOYSTICK----------------"""

def canvass(x,y):
	print("AQUIII")
	x=float(x)
	xx=int(x)
	y=float(y)
	yy=int(-y)
	if (yy==0 and x==0):
		stop()
	if (yy>0 and xx>=-50 and xx<50):
		adelante()
		pwm_motor.ChangeDutyCycle(yy)
		xpwm_motor.ChangeDutyCycle(yy)
		
	if (yy<0 and xx>=-50 and xx<50):
		reversa()
		pwm_motor.ChangeDutyCycle(-yy)
		xpwm_motor.ChangeDutyCycle(-yy)
		
	if (yy>0 and xx>50):
		derecha()
		pwm_motor.ChangeDutyCycle(yy)
		xpwm_motor.ChangeDutyCycle(yy)
	
	if (yy>0 and xx<=-50):
		izquierda()
		pwm_motor.ChangeDutyCycle(yy)
		xpwm_motor.ChangeDutyCycle(yy)

		


"""--------------------------CONTROL PINZA------------------------------"""

def convtick(angulo, mini, maxi):
	"""Funcion tick"""
	periodo = 0.020
	print (periodo)
	t_tick = periodo / 4096 * 1000000
	print (t_tick)
	tick_min = mini / t_tick
	print (tick_min)
	tick_max = maxi / t_tick
	print (tick_max)
	m = (tick_max - tick_min) / 180
	print (m)
	tick = m * angulo + tick_min
	print (tick)
	return (tick)


def pinza(angulo):
	"""Controla el primer servomotor, este tiene un rango entre HS425BB 553-2520μsec"""
	#Toma el valor recibido del deslizador en la página respecto al servo 1
	#Convierte ese valor de String a entero
	angulo = int(angulo)
	u_min = 553
	u_max = 2520
	tick = convtick(angulo, u_min, u_max)
	#Para comodidad, en la página el deslizador va de 13 a 50, por eso se multiplica * 10
	print ("El valor del servo es: " + str(int(tick)))
	#Colocamos el servo del canal 0, iniciando con alto hasta el valor del tick
	#La función hace lo siguiente |¨¨¨¨¨¨¨|______|¨¨¨¨¨¨¨|______
	#Esta función tiene como parámetro el canal, valor de on y off
	#Es decir, se enciende en 0 y se apaga en el tiempo del tick
	pwm.setPWM(0, 0, int(tick))


"""---------------------SWICHEO DE LUZ LED PARA CRUZAR EL TÚNEL-----------------------"""
#Definimos el pin 24 donde se encontrara el led que servirá de linterna
led = 11
#Colocamos el pin del led como salida
GPIO.setup(led, GPIO.OUT)
#Variable que controla el estado de la tira led de la linterna
estadoluz = "apagado"

def linterna():
	"""Controla la tira led que funcionará de linterna en el túnel"""
	global estadoluz
	#Si el estado de la luz es apagado
	if(estadoluz == "apagado"):
		#Enciende la tira led
		GPIO.output(led, 1)
		#Coloca el estado de la luz en encendido
		estadoluz = "encendido"
	#Si el estado de la luz es encendido
	elif(estadoluz == "encendido"):
		#Apaga la tira led
		GPIO.output(led, 0)
		#Coloca el estado de la luz en apagado
		estadoluz = "apagado"

"""-------------------------VERIFICAR CONEXIÓN A INTERNET-----------------------------"""


def internet():
	"""Realiza un ping a google para saber si hay conexión a internet
	Además de controlar la variable tira_led dependiendo de la conexión a internet
	Y si no hay internet ejecuta la función stop"""
	global tira_led
	global infinito
	#Mientras que la variable infinito sea True
	while infinito:
		#Hace un ping a google
		w = subprocess.Popen(["ping", "-c 1", "www.google.com"], stdout=subprocess.PIPE)
		w.wait()
		#Si da error el ping
		if w.poll():
			#Coloca la variable tira_led en False
			tira_led = False
			#Para los motores
			stop()
			print ("No hay internet")
			#Llama a la función linterna
			linterna()
			time.sleep(1)
		#Si el ping es correcto
		else:
			print ("Si hay internet")
			#Pone la tira_led en True
			tira_led = True
			time.sleep(1)


"""-------------TIRA LED PARA DECORACIÓN Y SEÑAL DE PÉRDIDA DE INTERNET---------------"""
#LEDS
green = 10
red = 25
blue = 9
GPIO.setup(red, GPIO.OUT) 
GPIO.setup(green, GPIO.OUT)
GPIO.setup(blue, GPIO.OUT)

#PWM LEDS
Freq = 100

RED = GPIO.PWM(red, Freq)  
GREEN = GPIO.PWM(green, Freq)
BLUE = GPIO.PWM(blue, Freq)
tira_led = True


def whitru():
	"""Ejecuta el ciclo infinito para la visualización de la tira led"""
	global tira_led
	#Recorre cada posición del vector colores
	
	
	RED.start(1)
	GREEN.start(100)
	BLUE.start(100)

	for x in range(1,101):
				if (tira_led is False):
					break
				GREEN.ChangeDutyCycle(101-x)
				time.sleep(0.05) 

	for x in range(1,101):
				if (tira_led is False):
					break
				RED.ChangeDutyCycle(x)
				time.sleep(0.015)

	for x in range(1,101):
				if (tira_led is False):
					break
				GREEN.ChangeDutyCycle(x)
				BLUE.ChangeDutyCycle(101-x)
				time.sleep(0.015)

	for x in range(1,101):
				if (tira_led is False):
					break
				RED.ChangeDutyCycle(101-x)
				time.sleep(0.015)


def whifalse():
	"""Función que muestra la tira led en rojo y parpadea cuando no hay internet"""
	#Enciende el rojo y apaga los otros colores
	GREEN.ChangeDutyCycle(0)
	RED.ChangeDutyCycle(100)
	BLUE.ChangeDutyCycle(0)
	time.sleep(1)
	#Apaga el rojo
	RED.ChangeDutyCycle(0)
	time.sleep(1)


def tira():
	"""Control total de la tira led"""
	#Mientras que la variable tira_led sea True
	global tira_led
	global infinito
	#Mientras que la variable infinito sea True
	while infinito:
		if (tira_led is True):
			whitru()
		if (tira_led is False):
			whifalse()


"""----------------------CONTROL DE LOS HILOS EN EJECUCIÓN----------------------------"""
global infinito
infinito = True
#Inicia el hilo que comprueba si hay internet
hilo_internet = threading.Thread(target=internet)
#Inicia el hilo que hace funcionar la tira led para estética y comprobar el internet
hilo_tira = threading.Thread(target=tira)
#Inicia el hilo de internet
hilo_internet.start()
#Inicia el hilo de la tira led
hilo_tira.start()



"""-----------------CONTROL DE VARIABLES PROVENIENTES DEL SERVIDOR--------------------"""

@webiopi.macro
def Canvas(x,y):
	"""Función que recibe los datos del deslizador de la página web de la aceleración
	y ejecuta la función de acelrar"""
	print("*********CANVAS******")
	canvass(x,y)

@webiopi.macro
def Pinza(angulo):
	print("si lleguo")
	pinza(angulo)

	
@webiopi.macro
def Aceleracion(valora):
	print("acelerando andoooooooooooo")
	"""Función que recibe los datos del deslizador de la página web de la aceleración
	y ejecuta la función de acelrar"""
	acelerar(valora)

@webiopi.macro
def Aceleras(velevar):
	acelerars(velevar)

@webiopi.macro
def pararpeq():
	parars()

@webiopi.macro
def BotonAdelante():
	"""Función que recibe los datos del botón de adelante de la página web
	y ejecuta la función de adelante"""
	print("*********ADELANTE******")
	adelante()

@webiopi.macro
def subirpeq():
	"""Función que recibe los datos del botón de adelante de la página web
	y ejecuta la función de adelante"""
	print("*********SUBIR PPPPP******")
	subirp()

@webiopi.macro
def bajarpeq():
	"""Función que recibe los datos del botón de adelante de la página web
	y ejecuta la función de adelante"""
	print("*********BAJAAAR PPPPP******")
	bajarp()


@webiopi.macro
def BotonReversa():
	"""Función que recibe los datos del botón de atrás de la página web
	y ejecuta la función de reversa"""
	reversa()


@webiopi.macro
def BotonDerecha():
	"""Función que recibe los datos del botón de derecha de la página web
	y ejecuta la función de derecha"""
	derecha()


@webiopi.macro
def BotonIzquierda():
	"""Función que recibe los datos del botón de izquierda de la página web
	y ejecuta la función de izquierda"""
	izquierda()

@webiopi.macro
def BotonStop():
	"""Función que recibe los datos del botón de parar de la página web
	y ejecuta la función stop"""
	stop()

@webiopi.macro
def luz():
	"""Función que recibe los datos del botón de luz de la página web
	y ejecuta la función linterna"""
	linterna()

"""---------------------------DESTRUYE LAS VARIABLES USADAS---------------------------"""


def destroy():
	"""Destruye todas las variables"""
	global infinito
	GPIO.cleanup()
	infinito = False
