#!/usr/bin/env python3
"""
Teleop Node for TMR 2026
Control vehicle with PS4 DualShock controller
Switch between AUTONOMOUS and TELEOP modes

PS4 Mapping:
  Axes:
    0: Left stick horizontal  (steering)
    1: Left stick vertical
    3: Right stick horizontal
    4: Right stick vertical
    5: R2 trigger (accelerate forward)  -1=released, 1=fully pressed
    6: L2 trigger (reverse/brake)       -1=released, 1=fully pressed
  Buttons:
    0: Cross    (X)  → Toggle mode
    1: Circle   (O)
    2: Triangle (△)  → Emergency Stop
    3: Square   (□)
    4: L1
    5: R1
    8: Share
    9: Options
"""

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, String, Bool

class TeleopController:
    def __init__(self):
        rospy.init_node('teleop_controller', anonymous=False)

        # ── Mode state ────────────────────────────────────────────────────
        self.autonomous_mode = True
        self.last_cross_state = 0      # para detectar flanco de subida

        # ── Control limits ────────────────────────────────────────────────
        self.max_speed  = 435   # máximo valor de velocidad (igual que on_lane)
        self.max_angle  = 45    # ±45° desde centro (90°)
        self.deadzone   = 0.1   # zona muerta del stick

        # ── PS4 axis / button indices (verificado con rostopic echo /joy) ──
        self.AXIS_STEER   = 0   # left stick horizontal → dirección
                                #   derecha = -1.0, izquierda = +1.0
        self.AXIS_R2      = 5   # R2 → acelerar adelante
                                #   suelto = 0.0, presionado = -1.0
        self.AXIS_L2      = 2   # L2 → reversa
                                #   suelto = 0.0, presionado = -1.0
        self.BTN_CROSS    = 0   # ✕  → cambiar modo
        self.BTN_TRIANGLE = 2   # △  → paro de emergencia

        # ── Publishers ────────────────────────────────────────────────────
        self.speed_pub     = rospy.Publisher('/AutoModelMini/manual_control/speed',
                                             Int16,  queue_size=1)
        self.steering_pub  = rospy.Publisher('/AutoModelMini/manual_control/steering',
                                             Int16,  queue_size=1)
        self.mode_pub      = rospy.Publisher('/vehicle_mode',   String, queue_size=1)
        self.emergency_pub = rospy.Publisher('/emergency_stop', Bool,   queue_size=1)

        # ── Subscriber ────────────────────────────────────────────────────
        rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1)

        self.publish_mode()

        rospy.loginfo("Teleop Controller initialized")
        rospy.loginfo("Controls (PS4):")
        rospy.loginfo("  R2              : Acelerar (adelante)")
        rospy.loginfo("  L2              : Reversa / frenar")
        rospy.loginfo("  Stick izq. horiz: Direccion izq/der")
        rospy.loginfo("  Cruz (X)        : Cambiar modo Autonomo/Teleop")
        rospy.loginfo("  Triangulo       : Paro de emergencia")
        rospy.loginfo(f"Modo actual: {'AUTONOMO' if self.autonomous_mode else 'TELEOP'}")

    # ── Callback principal ────────────────────────────────────────────────
    def joy_callback(self, joy_msg):

        # ── Cambio de modo — Cruz (flanco de subida) ──────────────────────
        cross_now = joy_msg.buttons[self.BTN_CROSS]
        if cross_now == 1 and self.last_cross_state == 0:
            self.toggle_mode()
        self.last_cross_state = cross_now

        # ── Paro de emergencia — Triángulo ────────────────────────────────
        if joy_msg.buttons[self.BTN_TRIANGLE] == 1:
            self.emergency_stop()
            return

        # ── Comandos solo en modo TELEOP ──────────────────────────────────
        if not self.autonomous_mode:

            # ── Dirección — stick izquierdo horizontal ────────────────────
            # derecha = -1.0, izquierda = +1.0 → invertir para que
            # derecha = ángulo > 90°, izquierda = ángulo < 90°
            steer_raw = joy_msg.axes[self.AXIS_STEER]
            if abs(steer_raw) < self.deadzone:
                steer_raw = 0.0
            angle = int(90 - steer_raw * self.max_angle)  # signo invertido
            angle = max(45, min(135, angle))

            # ── Velocidad — R2 adelante, L2 reversa ──────────────────────
            # Gatillos PS4: suelto=0.0, presionado=-1.0 → invertir y normalizar
            r2_raw = joy_msg.axes[self.AXIS_R2]  # 0 a -1
            l2_raw = joy_msg.axes[self.AXIS_L2]  # 0 a -1
            r2 = -r2_raw   # 0=suelto, 1=fondo
            l2 = -l2_raw   # 0=suelto, 1=fondo

            # Sistema: negativo=adelante, positivo=reversa
            if r2 > 0.05:
                speed = int(-r2 * self.max_speed)   # adelante
            elif l2 > 0.05:
                speed = int(l2 * self.max_speed)    # reversa
            else:
                speed = 0                            # parado

            self.publish_speed(speed)
            self.publish_steering(angle)

            if speed != 0 or angle != 90:
                rospy.logdebug(f"TELEOP: Speed={speed}, Angle={angle}")

    # ── Helpers ───────────────────────────────────────────────────────────
    def toggle_mode(self):
        self.autonomous_mode = not self.autonomous_mode
        self.publish_mode()
        if self.autonomous_mode:
            rospy.loginfo("Switched to AUTONOMOUS mode")
            # Detener el carro al volver a autónomo
            self.publish_speed(0)
            self.publish_steering(90)
        else:
            rospy.logwarn("Switched to TELEOP mode - Control manual activo")

    def emergency_stop(self):
        rospy.logwarn("EMERGENCY STOP!")
        self.publish_speed(0)
        self.publish_steering(90)
        msg = Bool(); msg.data = True
        self.emergency_pub.publish(msg)
        rospy.Timer(rospy.Duration(1.0), self.release_emergency, oneshot=True)

    def release_emergency(self, event):
        msg = Bool(); msg.data = False
        self.emergency_pub.publish(msg)
        rospy.loginfo("Emergency stop released")

    def publish_speed(self, speed):
        msg = Int16(); msg.data = speed
        self.speed_pub.publish(msg)

    def publish_steering(self, angle):
        msg = Int16(); msg.data = angle
        self.steering_pub.publish(msg)

    def publish_mode(self):
        msg = String()
        msg.data = "AUTONOMOUS" if self.autonomous_mode else "TELEOP"
        self.mode_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.publish_mode()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = TeleopController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
