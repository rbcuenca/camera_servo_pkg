# Este script é uma versão para ROS2 dos scripts originais
# em ROS1 feitos pelo Arnaldo Júnior e Lícia Sales.
# Feito por: Rogério Cuenca.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from rclpy.qos import ReliabilityPolicy, QoSProfile
import RPi.GPIO as GPIO
import time

class CameraServo(Node):

    def __init__(self):
        super().__init__('camera_servo')

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.cleanup()         

        pinMotorAForwards = 19

        # Seta a frequencia
        Frequency = 50
        # Seta o DutyCycle
        self.DutyCycle = 8
        self.DutyCycle2 = 5
        # Para o servo motor
        self.Stop = 0
        

        # Seta o pino do servo motor como saida
        GPIO.setup(pinMotorAForwards, GPIO.OUT)

        # Ajusta a frequencia do PWM
        self.pwmMotor = GPIO.PWM(pinMotorAForwards, Frequency)

        # Inicializa o PWM (servo parado)
        self.pwmMotor.start(self.Stop)
        
        self.subscriber = self.create_subscription(
            Float32,
            '/servo_camera/position',
            self.command_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read LaserScan data and some sensor data.
        self.subscriber = self.create_subscription(
            String,
            '/servo_camera/command',
            self.command_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read LaserScan data and some sensor data.

    # Funcao stop servo motor
    def StopMotor(self):
        self.pwmMotor.ChangeDutyCycle(self.Stop)

    # Funcao sobe camera
    def SobeCamera(self):
        self.pwmMotor.ChangeDutyCycle(self.DutyCycle)

    # Funcao desce camera
    def DesceCamera(self):
        self.pwmMotor.ChangeDutyCycle(self.DutyCycle2)

    # Funcao posicao
    def PosCamera(self, command):
        self.pwmMotor.ChangeDutyCycle(command)



    def command_callback(self, commandMessage):
        # print the log info in the terminal
        self.get_logger().info(str(commandMessage.data))
        command = commandMessage.data
        if command == 'sobe':
            self.get_logger().info('sobe Camera')
            self.SobeCamera()
            time.sleep(0.5)
            self.StopMotor()
            time.sleep(0.5)

        elif command == 'desce':
            self.get_logger().info('Desce Camera')
            self.DesceCamera()
            time.sleep(0.5)
            self.StopMotor()
            time.sleep(0.5)

        elif command == 'stop':
            self.get_logger().info('Parando')
            self.StopMotor()
        else:
            self.get_logger().info('Posicao', command)
            self.PosCamera(command)
            time.sleep(0.5)
            self.StopMotor()
            time.sleep(0.5)



def main(args=None):
    rclpy.init(args=args)
    camera_servo = CameraServo()
    rclpy.spin(camera_servo)
    camera_servo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()