#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
from Ax12 import Ax12
import time
import RPi.GPIO as GPIO
from gpiozero import DigitalInputDevice

class AX12Controller(Node):
    def __init__(self):
        super().__init__('ax12_controller')
        self.publisher_ = self.create_publisher(Float32, 'direction_position', 10)
        self.speed_publisher = self.create_publisher(Float32, 'motor_speed', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        Ax12.DEVICENAME = '/dev/ttyUSB0'
        Ax12.BAUDRATE = 1_000_000
        Ax12.connect()
        self.motor = Ax12(1)
        self.motor.set_moving_speed(300)

        # Motor GPIO Pins
        self.M1_En = 21
        self.M1_In1 = 20
        self.M1_In2 = 16

        # Encoder GPIO Pins
        self.encoder_a_pin = 17
        self.encoder_b_pin = 27

        # Encoder setup
        self.encoder_a = DigitalInputDevice(self.encoder_a_pin, pull_up=True)
        self.encoder_b = DigitalInputDevice(self.encoder_b_pin, pull_up=True)
        
        self.count = 0
        self.last_time = time.time()
        self.pulses_per_revolution = 4741  # Replace with your encoder's pulses per revolution
        
        self.encoder_a.when_activated = self.encoder_pulse

        # Setup GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.M1_En, GPIO.OUT)
        GPIO.setup(self.M1_In1, GPIO.OUT)
        GPIO.setup(self.M1_In2, GPIO.OUT)

        # Initialize PWM
        self.M1_Vitesse = GPIO.PWM(self.M1_En, 100)
        self.M1_Vitesse.start(0)  # Start with 0 speed

        # Initialize the last_msg attribute
        self.last_msg = time.time()

        # Timers
        self.timer_direction = self.create_timer(0.1, self.publish_position)  # Publish every 100ms
        self.timer_speed = self.create_timer(0.5, self.publish_speed)  # Publish every 500ms
        self.timer_AR = self.create_timer(0.1, self.AR)

        self.MIN_PWM_START = 30  # Minimum starting speed
        self.MIN_PWM_RUN = 10  # Minimum running speed

    def encoder_pulse(self):
        self.count += 1

    def AR(self):
        if (time.time() - self.last_msg > 0.5):
            msg_AR = Twist()
            msg_AR.linear.x = 0.0
            msg_AR.angular.z = 0.0
            self.cmd_vel_callback(msg_AR)
            print("ARRET D'URGENCE")

    def publish_position(self):
        position = self.motor.get_present_position()
        position = -(position-512)*0.01
        msg = Float32()
        msg.data = position
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing motor position: %d' % position)

    def publish_speed(self):
        current_time = time.time()
        elapsed_time = current_time - self.last_time
        counts_per_second = self.count / elapsed_time
        rotations_per_minute = (counts_per_second / self.pulses_per_revolution) * 60
        
        msg = Float32()
        msg.data = float(rotations_per_minute)
        self.speed_publisher.publish(msg)
        #self.get_logger().info('Publishing motor speed: %d' % rotations_per_minute)

        # Reset for the next measurement
        self.count = 0
        self.last_time = current_time

    def cmd_vel_callback(self, msg):
        self.last_msg = time.time()
        self.motor.set_goal_position(int((-msg.angular.z + 3 + 2.2 - 0.05) * 100))
        
        linear_x = msg.linear.x
        if abs(linear_x) == 1.0:
            speed = 100
        else:
            speed = abs(linear_x) * 100  # Scale linear speed to PWM duty cycle (0-100)
        if speed < self.MIN_PWM_START:
            speed = self.MIN_PWM_START

        if linear_x > 0:
            self.sens1(speed)
        elif linear_x < 0:
            self.sens2(speed)
        else:
            self.arret()

    def sens1(self, speed):
        GPIO.output(self.M1_In1, GPIO.HIGH)
        GPIO.output(self.M1_In2, GPIO.LOW)
        self.M1_Vitesse.ChangeDutyCycle(speed)
        
    def sens2(self, speed):
        GPIO.output(self.M1_In1, GPIO.LOW)
        GPIO.output(self.M1_In2, GPIO.HIGH)
        self.M1_Vitesse.ChangeDutyCycle(speed)
        
    def arret(self):
        GPIO.output(self.M1_In1, GPIO.LOW)
        GPIO.output(self.M1_In2, GPIO.LOW)
        self.M1_Vitesse.ChangeDutyCycle(0)
        self.get_logger().info('Moteur arret.')

    def arretComplet(self):
        GPIO.output(self.M1_In1, GPIO.LOW)
        GPIO.output(self.M1_In2, GPIO.LOW)
        self.M1_Vitesse.ChangeDutyCycle(0)
        self.get_logger().info('Moteur arrete.')

    def destroy_node(self):
        self.encoder_a.close()
        self.encoder_b.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    ax12_controller = AX12Controller()
    try:
        rclpy.spin(ax12_controller)
    except KeyboardInterrupt:
        pass
    finally:
        ax12_controller.arretComplet()
        ax12_controller.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
