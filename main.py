#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from time import sleep
import math

# Variáveis do robô
diametro_roda = 5.6

def cm_to_degree(cm):
  return (cm * 360) / (diametro_roda * math.pi) 

def media_wheels(motor_a, motor_b):
  return (motor_a.angle() + motor_b.angle()) / 2

# Inicializando variáveis do EV3, os motores, valor de sensor, etc...
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
gyro = GyroSensor(Port.S3)

# Resetando valores
left_motor.reset_angle(0)
right_motor.reset_angle(0)
gyro.reset_angle(0)

# Variáveis utilizadas no processo
porcentagem = 60
distancia = cm_to_degree(100)
target = 45
kp, ki, kd = 1, 0.01, 2
integral, derivate, error, last_error = 0, 0, 0, 0

while distancia > media_wheels(left_motor, right_motor):
  error = target - gyro.angle()
  integral += error
  derivate = error - last_error

  correcao = kp * (error + ki * integral + kd * derivate)

  if (correcao >= 0):
    left_motor.dc(porcentagem + correcao)
    right_motor.dc(porcentagem)
  else:
    right_motor.dc(porcentagem + correcao * -1)
    left_motor.dc(porcentagem)

left_motor.hold()
right_motor.hold()