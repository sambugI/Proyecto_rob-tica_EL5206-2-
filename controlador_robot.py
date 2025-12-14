#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Controlador:
    def __init__(self):
        rospy.init_node('controlador_velocidad', anonymous=True)

        # Variables recibidas
        self.objeto_detectado = None
        self.distancia = None
        self.error_x = None

        # Suscriptores
        self.distancia_objeto = rospy.Subscriber("/distancia_objeto", String, self.callback)
        self.cmd_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

        self.previous_error_twist = 0
        self.previous_error_linear = 0
        self.integral_linear = 0
        self.integral_twist = 0
        self.factor1 = 0.0008
        self.factor2 = 0.005
        self.distancia_objetivo = 60
        self.last_time = rospy.Time.now()
        rospy.loginfo("Iniciando envío de velocidades del controlador.")

        # Loop de control cada 0.1 segundos
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def callback(self, msg):
        res = msg.data.split(',')
        for item in res:
            print(item)
        self.objeto_detectado = res[0]
        if self.objeto_detectado == "Mochila":
            self.distancia_objetivo = 100
        else:
            self.distancia_objetivo = 60
            
        nueva_distancia = float(res[1])

        if self.distancia is None:
            # Primera lectura: no hay valor previo, solo asigna
            self.distancia = nueva_distancia
        else:
            alpha = 0.7   # Coeficiente de suavizado
            self.distancia = alpha * nueva_distancia + (1 - alpha) * self.distancia
        self.error_x = float(res[2])

    def pid_controller(self, setpoint, pv, kp, ki, kd, previous_error, integral, dt):
        error = setpoint - pv
        integral += error * dt
        integral = max(min(integral, 100), -100)
        derivative = (error - previous_error) / dt
        control = kp * error + ki * integral + kd * derivative
        return control, error, integral

    def control_loop(self, event):
        """Controlador que publica Twist"""
        # Calcular dt correctamente
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt <= 0:
            return
        if self.error_x is None or self.distancia is None:
            return  # No hay datos aún

        twist = Twist()

        if abs(self.error_x) >= 10:
            control, error, self.integral_twist = self.pid_controller(0, self.error_x, self.factor1, 0.00001, 0.0005,self.previous_error_twist,self.integral_twist, dt)
            self.previous_error_twist = error
            #if self.previous_error_twist >= 30:
            #    self.previous_error_twist =  30
            velocidad_twist = control
            rospy.loginfo("Girando a velocidad {}. Falta distancia: {}".format(
                velocidad_twist, self.error_x
            ))
             # Cuando está corrigiendo el ángulo, NO debe avanzar
            twist.linear.x = 0
        else:
            velocidad_twist = 0.0
            rospy.loginfo("Se llegó al objetivo con distancia: {}".format(self.error_x))
        if abs (self.distancia - self.distancia_objetivo) >= 5:
            control, error, self .integral_linear = self.pid_controller (self.distancia, self.distancia_objetivo, self.factor2/2.5, 0.00001, 0.0001, self.previous_error_linear, self.integral_linear, dt)
            self.previous_error_linear = error
            #if self.previous_error_linear >= 10:
            #    self.previous_error_linear = 10
            velocidad_lineal = control
        else:
            velocidad_lineal = 0
        twist.linear.x = velocidad_lineal 
        rospy.loginfo("Moviendo lineal: vel={} distancia={}".format(
            velocidad_lineal, self.distancia
           ))
        twist.angular.z = velocidad_twist
        self.cmd_pub.publish(twist)


if __name__ == '__main__':
    try:
        Controlador()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
