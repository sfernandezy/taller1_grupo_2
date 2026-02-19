#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import filedialog, messagebox
import threading
import numpy as np

class TurtleBotInterface(Node):
    def __init__(self, root):
        super().__init__('turtle_bot_interface')
        
        self.root = root
        self.root.title("TurtleBot Interface - Visualización en Tiempo Real")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Datos de posición
        self.x_data = []
        self.y_data = []
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Suscriptor a la posición del robot
        self.position_sub = self.create_subscription(
            Twist,
            'turtlebot_position',  # Nota: el tópico tiene un typo en la guía
            self.position_callback,
            10
        )
        
        # Configurar interfaz gráfica
        self.setup_gui()
        
        self.get_logger().info('Nodo turtle_bot_interface iniciado')
        self.get_logger().info('Suscrito a: turtlebot_position')
        
    def setup_gui(self):
        """Configura la interfaz gráfica"""
        # Frame principal
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Frame para controles
        control_frame = tk.Frame(main_frame)
        control_frame.pack(side=tk.TOP, fill=tk.X, pady=(0, 10))
        
        # Label de información
        info_label = tk.Label(
            control_frame, 
            text="Trayectoria del Robot en Tiempo Real",
            font=("Arial", 14, "bold")
        )
        info_label.pack(pady=5)
        
        # Frame para posición actual
        pos_frame = tk.Frame(control_frame)
        pos_frame.pack(pady=5)
        
        self.pos_label = tk.Label(
            pos_frame,
            text="Posición: X=0.00 m, Y=0.00 m",
            font=("Arial", 10)
        )
        self.pos_label.pack()
        
        # Botón para guardar imagen
        save_button = tk.Button(
            control_frame,
            text="Guardar Gráfica",
            command=self.save_plot,
            font=("Arial", 12),
            bg="#4CAF50",
            fg="white",
            padx=20,
            pady=5
        )
        save_button.pack(pady=5)
        
        # Crear figura de matplotlib
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlabel('X (metros)', fontsize=12)
        self.ax.set_ylabel('Y (metros)', fontsize=12)
        self.ax.set_title('Trayectoria del TurtleBot2', fontsize=14, fontweight='bold')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Línea de trayectoria
        self.line, = self.ax.plot([], [], 'b-', linewidth=2, label='Trayectoria')
        self.point, = self.ax.plot([], [], 'ro', markersize=10, label='Posición actual')
        self.ax.legend()
        
        # Configurar límites iniciales
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        
        # Incrustar figura en tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=main_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def position_callback(self, msg):
        """Callback para recibir la posición del robot"""
        # La posición viene en msg.linear (x, y, z)
        self.current_x = msg.linear.x
        self.current_y = msg.linear.y
        
        # Agregar a las listas
        self.x_data.append(self.current_x)
        self.y_data.append(self.current_y)
        
        # Actualizar gráfica
        self.update_plot()
        
    def update_plot(self):
        """Actualiza la gráfica con los nuevos datos"""
        if len(self.x_data) == 0:
            return
            
        # Actualizar datos de la línea
        self.line.set_data(self.x_data, self.y_data)
        self.point.set_data([self.current_x], [self.current_y])
        
        # Actualizar label de posición
        self.pos_label.config(
            text=f"Posición: X={self.current_x:.2f} m, Y={self.current_y:.2f} m"
        )
        
        # Ajustar límites si es necesario
        if len(self.x_data) > 0:
            margin = 0.5
            x_min, x_max = min(self.x_data) - margin, max(self.x_data) + margin
            y_min, y_max = min(self.y_data) - margin, max(self.y_data) + margin
            
            self.ax.set_xlim(x_min, x_max)
            self.ax.set_ylim(y_min, y_max)
        
        # Redibujar
        self.canvas.draw()
        
    def save_plot(self):
        """Guarda la gráfica en un archivo"""
        if len(self.x_data) == 0:
            messagebox.showwarning("Advertencia", "No hay datos para guardar")
            return
            
        # Solicitar nombre y ubicación
        filename = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[
                ("PNG files", "*.png"),
                ("PDF files", "*.pdf"),
                ("All files", "*.*")
            ],
            title="Guardar Gráfica"
        )
        
        if filename:
            try:
                self.fig.savefig(filename, dpi=300, bbox_inches='tight')
                self.get_logger().info(f'Gráfica guardada en: {filename}')
                messagebox.showinfo("Éxito", f"Gráfica guardada en:\n{filename}")
            except Exception as e:
                self.get_logger().error(f'Error al guardar: {str(e)}')
                messagebox.showerror("Error", f"No se pudo guardar la gráfica:\n{str(e)}")
    
    def on_closing(self):
        """Maneja el cierre de la ventana"""
        self.get_logger().info('Cerrando interfaz...')
        self.root.quit()
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    
    # Crear ventana de tkinter
    root = tk.Tk()
    
    # Crear nodo
    interface_node = TurtleBotInterface(root)
    
    # Thread para ROS
    def spin_node():
        rclpy.spin(interface_node)
    
    ros_thread = threading.Thread(target=spin_node, daemon=True)
    ros_thread.start()
    
    # Ejecutar GUI
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        interface_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
