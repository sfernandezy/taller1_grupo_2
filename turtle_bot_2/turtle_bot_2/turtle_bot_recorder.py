#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import filedialog, messagebox
import json
import threading
from datetime import datetime

class TurtleBotRecorder(Node):
    def __init__(self, root):
        super().__init__('turtle_bot_recorder')
        
        self.root = root
        self.root.title("TurtleBot Recorder - Grabación de Trayectorias")
        self.root.geometry("500x300")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Variables de grabación
        self.is_recording = False
        self.recorded_velocities = []
        self.start_time = None
        
        # Suscriptor a velocidades
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'turtlebot_cmdVel',
            self.cmd_vel_callback,
            10
        )
        
        # Configurar GUI
        self.setup_gui()
        
        self.get_logger().info('Nodo turtle_bot_recorder iniciado')
        
    def setup_gui(self):
        """Configura la interfaz gráfica"""
        # Frame principal
        main_frame = tk.Frame(self.root, padx=20, pady=20)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Título
        title_label = tk.Label(
            main_frame,
            text="Grabación de Trayectorias",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=(0, 20))
        
        # Pregunta inicial
        question_label = tk.Label(
            main_frame,
            text="¿Desea guardar el recorrido del robot?",
            font=("Arial", 12)
        )
        question_label.pack(pady=10)
        
        # Frame para botones
        button_frame = tk.Frame(main_frame)
        button_frame.pack(pady=20)
        
        # Botón SÍ
        yes_button = tk.Button(
            button_frame,
            text="SÍ, Grabar",
            command=self.start_recording,
            font=("Arial", 12),
            bg="#4CAF50",
            fg="white",
            padx=30,
            pady=10,
            width=15
        )
        yes_button.pack(side=tk.LEFT, padx=10)
        
        # Botón NO
        no_button = tk.Button(
            button_frame,
            text="NO",
            command=self.skip_recording,
            font=("Arial", 12),
            bg="#f44336",
            fg="white",
            padx=30,
            pady=10,
            width=15
        )
        no_button.pack(side=tk.LEFT, padx=10)
        
        # Label de estado
        self.status_label = tk.Label(
            main_frame,
            text="Esperando decisión...",
            font=("Arial", 10),
            fg="blue"
        )
        self.status_label.pack(pady=20)
        
        # Botón para detener y guardar
        self.stop_button = tk.Button(
            main_frame,
            text="Detener y Guardar Grabación",
            command=self.stop_and_save_recording,
            font=("Arial", 12),
            bg="#FF9800",
            fg="white",
            padx=20,
            pady=10,
            state=tk.DISABLED
        )
        self.stop_button.pack(pady=10)
        
    def start_recording(self):
        """Inicia la grabación"""
        self.is_recording = True
        self.recorded_velocities = []
        self.start_time = self.get_clock().now()
        
        self.status_label.config(
            text="🔴 GRABANDO... Controle el robot con teleop",
            fg="red"
        )
        self.stop_button.config(state=tk.NORMAL)
        
        self.get_logger().info('Grabación iniciada')
        
    def skip_recording(self):
        """Omite la grabación"""
        self.is_recording = False
        self.status_label.config(
            text="Grabación omitida. Puede cerrar esta ventana.",
            fg="gray"
        )
        self.get_logger().info('Grabación omitida por el usuario')
        
    def cmd_vel_callback(self, msg):
        """Callback para grabar velocidades"""
        if not self.is_recording:
            return
            
        # Calcular tiempo transcurrido
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        # Grabar velocidad con timestamp
        velocity_data = {
            'timestamp': elapsed,
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'linear_z': msg.linear.z,
            'angular_x': msg.angular.x,
            'angular_y': msg.angular.y,
            'angular_z': msg.angular.z
        }
        
        self.recorded_velocities.append(velocity_data)
        
        # Actualizar contador
        self.status_label.config(
            text=f"🔴 GRABANDO... {len(self.recorded_velocities)} comandos grabados"
        )
        
    def stop_and_save_recording(self):
        """Detiene la grabación y solicita guardar"""
        if not self.is_recording:
            return
            
        self.is_recording = False
        
        if len(self.recorded_velocities) == 0:
            messagebox.showwarning("Advertencia", "No hay datos para guardar")
            return
        
        # Solicitar nombre de archivo
        filename = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("JSON files", "*.json"), ("All files", "*.*")],
            title="Guardar Recorrido",
            initialfile=f"trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        )
        
        if filename:
            try:
                # Guardar en formato JSON para mejor legibilidad
                data_to_save = {
                    'metadata': {
                        'total_commands': len(self.recorded_velocities),
                        'duration': self.recorded_velocities[-1]['timestamp'] if self.recorded_velocities else 0,
                        'timestamp': datetime.now().isoformat()
                    },
                    'trajectory': self.recorded_velocities
                }
                
                with open(filename, 'w') as f:
                    json.dump(data_to_save, f, indent=2)
                
                self.get_logger().info(f'Recorrido guardado en: {filename}')
                self.status_label.config(
                    text=f"✅ Grabación guardada: {len(self.recorded_velocities)} comandos",
                    fg="green"
                )
                messagebox.showinfo("Éxito", f"Recorrido guardado en:\n{filename}")
                
                self.stop_button.config(state=tk.DISABLED)
                
            except Exception as e:
                self.get_logger().error(f'Error al guardar: {str(e)}')
                messagebox.showerror("Error", f"No se pudo guardar:\n{str(e)}")
    
    def on_closing(self):
        """Maneja el cierre de la ventana"""
        if self.is_recording:
            response = messagebox.askyesno(
                "Grabación en curso",
                "Hay una grabación en curso. ¿Desea guardarla antes de salir?"
            )
            if response:
                self.stop_and_save_recording()
        
        self.root.quit()
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    
    # Crear ventana
    root = tk.Tk()
    
    # Crear nodo
    recorder_node = TurtleBotRecorder(root)
    
    # Thread para ROS
    def spin_node():
        rclpy.spin(recorder_node)
    
    ros_thread = threading.Thread(target=spin_node, daemon=True)
    ros_thread.start()
    
    # Ejecutar GUI
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        recorder_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
