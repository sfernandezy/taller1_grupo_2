#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading

class TurtleBotTeleop(Node):
    def __init__(self, linear_vel, angular_vel):
        super().__init__('turtle_bot_teleop')
        
        # Velocidades configuradas por el usuario
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel
        
        # Publisher para enviar velocidades al robot
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'turtlebot_cmdVel', 
            10
        )
        
        # Mensaje Twist para controlar el robot
        self.twist_msg = Twist()
        
        self.get_logger().info('Nodo turtle_bot_teleop iniciado')
        self.get_logger().info(f'Velocidad lineal: {self.linear_vel} m/s')
        self.get_logger().info(f'Velocidad angular: {self.angular_vel} rad/s')
        self.get_logger().info('Controles:')
        self.get_logger().info('  W - Adelante')
        self.get_logger().info('  S - Atrás')
        self.get_logger().info('  A - Girar izquierda')
        self.get_logger().info('  D - Girar derecha')
        self.get_logger().info('  Q - Salir')
        
        # Thread para leer teclas
        self.running = True
        self.key_thread = threading.Thread(target=self.read_keys)
        self.key_thread.daemon = True
        self.key_thread.start()
        
    def read_keys(self):
        """Lee las teclas presionadas del usuario"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setraw(fd)
            while self.running and rclpy.ok():
                key = sys.stdin.read(1).lower()
                self.process_key(key)
                
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    def process_key(self, key):
        """Procesa la tecla presionada y publica velocidades"""
        # Reiniciar velocidades
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0
        
        if key == 'w':
            # Adelante
            self.twist_msg.linear.x = self.linear_vel
            self.get_logger().info('Moviendo adelante')
            
        elif key == 's':
            # Atrás
            self.twist_msg.linear.x = -self.linear_vel
            self.get_logger().info('Moviendo atrás')
            
        elif key == 'a':
            # Girar izquierda
            self.twist_msg.angular.z = self.angular_vel
            self.get_logger().info('Girando izquierda')
            
        elif key == 'd':
            # Girar derecha
            self.twist_msg.angular.z = -self.angular_vel
            self.get_logger().info('Girando derecha')
            
        elif key == 'q':
            # Salir
            self.get_logger().info('Saliendo...')
            self.running = False
            rclpy.shutdown()
            return
        
        # Publicar velocidades
        self.cmd_vel_pub.publish(self.twist_msg)
    
    def stop(self):
        """Detiene el robot"""
        self.running = False
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)

def main(args=None):
    # Solicitar velocidades al usuario
    try:
        print("\n=== Configuración de TurtleBot Teleop ===")
        linear_vel = float(input("Ingrese velocidad lineal (m/s, máx 0.7): "))
        angular_vel = float(input("Ingrese velocidad angular (rad/s, máx 3.14): "))
        
        # Validar límites
        linear_vel = max(0.0, min(linear_vel, 0.7))
        angular_vel = max(0.0, min(angular_vel, 3.14))
        
    except ValueError:
        print("Error: Ingrese valores numéricos válidos")
        return
    
    rclpy.init(args=args)
    teleop_node = TurtleBotTeleop(linear_vel, angular_vel)
    
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.stop()
        teleop_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
