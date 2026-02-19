#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from example_interfaces.srv import SetBool
import tkinter as tk
from tkinter import filedialog, messagebox
import json
import threading
import time

class PlayTrajectoryService(Node):
    """Servicio personalizado para recibir el nombre del archivo"""
    pass

class TurtleBotPlayer(Node):
    def __init__(self, root):
        super().__init__('turtle_bot_player')
        
        self.root = root
        self.root.title("TurtleBot Player - Reproducción de Trayectorias")
        self.root.geometry("600x400")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Variables
        self.is_playing = False
        self.trajectory_data = None
        self.current_file = None
        
        # Publisher para velocidades
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'turtlebot_cmdVel',
            10
        )
        
        # Timer para reproducción
        self.play_timer = None
        self.play_index = 0
        
        # Configurar GUI
        self.setup_gui()
        
        self.get_logger().info('Nodo turtle_bot_player iniciado')
        
    def setup_gui(self):
        """Configura la interfaz gráfica"""
        main_frame = tk.Frame(self.root, padx=20, pady=20)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Título
        title_label = tk.Label(
            main_frame,
            text="Reproductor de Trayectorias",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=(0, 20))
        
        # Frame de información
        info_frame = tk.LabelFrame(main_frame, text="Archivo Actual", padx=10, pady=10)
        info_frame.pack(fill=tk.X, pady=10)
        
        self.file_label = tk.Label(
            info_frame,
            text="Ningún archivo cargado",
            font=("Arial", 10),
            fg="gray"
        )
        self.file_label.pack()
        
        self.info_label = tk.Label(
            info_frame,
            text="",
            font=("Arial", 9)
        )
        self.info_label.pack()
        
        # Botón para cargar archivo
        load_button = tk.Button(
            main_frame,
            text="📁 Cargar Archivo de Trayectoria",
            command=self.load_trajectory,
            font=("Arial", 12),
            bg="#2196F3",
            fg="white",
            padx=20,
            pady=10
        )
        load_button.pack(pady=10)
        
        # Botón para reproducir
        self.play_button = tk.Button(
            main_frame,
            text="▶️ Reproducir Trayectoria",
            command=self.play_trajectory,
            font=("Arial", 12),
            bg="#4CAF50",
            fg="white",
            padx=20,
            pady=10,
            state=tk.DISABLED
        )
        self.play_button.pack(pady=10)
        
        # Botón para detener
        self.stop_button = tk.Button(
            main_frame,
            text="⏹️ Detener Reproducción",
            command=self.stop_playback,
            font=("Arial", 12),
            bg="#f44336",
            fg="white",
            padx=20,
            pady=10,
            state=tk.DISABLED
        )
        self.stop_button.pack(pady=10)
        
        # Label de estado
        self.status_label = tk.Label(
            main_frame,
            text="Esperando archivo...",
            font=("Arial", 10),
            fg="blue"
        )
        self.status_label.pack(pady=20)
        
        # Barra de progreso
        self.progress_label = tk.Label(
            main_frame,
            text="",
            font=("Arial", 9)
        )
        self.progress_label.pack()
        
    def load_trajectory(self):
        """Carga un archivo de trayectoria"""
        filename = filedialog.askopenfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("JSON files", "*.json"), ("All files", "*.*")],
            title="Cargar Trayectoria"
        )
        
        if not filename:
            return
            
        try:
            with open(filename, 'r') as f:
                self.trajectory_data = json.load(f)
            
            self.current_file = filename
            
            # Actualizar interfaz
            file_name = filename.split('/')[-1]
            self.file_label.config(text=f"📄 {file_name}", fg="black")
            
            total_commands = self.trajectory_data['metadata']['total_commands']
            duration = self.trajectory_data['metadata']['duration']
            
            self.info_label.config(
                text=f"Comandos: {total_commands} | Duración: {duration:.2f}s"
            )
            
            self.play_button.config(state=tk.NORMAL)
            self.status_label.config(text="Archivo cargado. Listo para reproducir.", fg="green")
            
            self.get_logger().info(f'Trayectoria cargada: {filename}')
            self.get_logger().info(f'Total comandos: {total_commands}')
            
        except Exception as e:
            self.get_logger().error(f'Error al cargar archivo: {str(e)}')
            messagebox.showerror("Error", f"No se pudo cargar el archivo:\n{str(e)}")
    
    def play_trajectory(self):
        """Reproduce la trayectoria cargada"""
        if not self.trajectory_data:
            messagebox.showwarning("Advertencia", "No hay trayectoria cargada")
            return
            
        self.is_playing = True
        self.play_index = 0
        
        self.play_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.status_label.config(text="▶️ Reproduciendo...", fg="red")
        
        self.get_logger().info('Iniciando reproducción de trayectoria')
        
        # Iniciar reproducción en thread separado
        play_thread = threading.Thread(target=self.playback_loop, daemon=True)
        play_thread.start()
    
    def playback_loop(self):
        """Loop de reproducción de la trayectoria"""
        trajectory = self.trajectory_data['trajectory']
        
        prev_timestamp = 0.0
        
        for i, cmd in enumerate(trajectory):
            if not self.is_playing:
                break
                
            # Calcular tiempo de espera
            current_timestamp = cmd['timestamp']
            wait_time = current_timestamp - prev_timestamp
            prev_timestamp = current_timestamp
            
            if wait_time > 0:
                time.sleep(wait_time)
            
            # Publicar velocidad
            twist_msg = Twist()
            twist_msg.linear.x = cmd['linear_x']
            twist_msg.linear.y = cmd['linear_y']
            twist_msg.linear.z = cmd['linear_z']
            twist_msg.angular.x = cmd['angular_x']
            twist_msg.angular.y = cmd['angular_y']
            twist_msg.angular.z = cmd['angular_z']
            
            self.cmd_vel_pub.publish(twist_msg)
            
            # Actualizar progreso
            progress = (i + 1) / len(trajectory) * 100
            self.progress_label.config(
                text=f"Progreso: {i+1}/{len(trajectory)} ({progress:.1f}%)"
            )
            
        # Detener robot al finalizar
        self.stop_robot()
        
        if self.is_playing:  # Si terminó normalmente
            self.status_label.config(text="✅ Reproducción completada", fg="green")
            self.get_logger().info('Reproducción completada')
        
        self.is_playing = False
        self.play_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
    
    def stop_playback(self):
        """Detiene la reproducción"""
        self.is_playing = False
        self.stop_robot()
        
        self.status_label.config(text="⏹️ Reproducción detenida", fg="orange")
        self.play_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        
        self.get_logger().info('Reproducción detenida por el usuario')
    
    def stop_robot(self):
        """Detiene el robot enviando velocidades cero"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
    
    def on_closing(self):
        """Maneja el cierre de la ventana"""
        if self.is_playing:
            self.stop_playback()
        
        self.root.quit()
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    
    # Crear ventana
    root = tk.Tk()
    
    # Crear nodo
    player_node = TurtleBotPlayer(root)
    
    # Thread para ROS
    def spin_node():
        rclpy.spin(player_node)
    
    ros_thread = threading.Thread(target=spin_node, daemon=True)
    ros_thread.start()
    
    # Ejecutar GUI
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        player_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
    
