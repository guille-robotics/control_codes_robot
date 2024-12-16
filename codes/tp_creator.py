#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
# Este genera una ventana de 50 x 50 m para seleccionar el TP.
class TargetPublisher(Node):
    def __init__(self, width=50, height=50):
        super().__init__('target_publisher')
        self.width = width
        self.height = height
        self.target_pub = self.create_publisher(Point, '/target_point', 10)

        # Crear figura y ejes para la selección
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim([0, self.width])
        self.ax.set_ylim([0, self.height])
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Haz clic en la ventana para seleccionar el punto objetivo')
        # Activar la rejilla
        self.ax.grid(True)
        
        # Conectar el evento de clic
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

    def onclick(self, event):
        if event.xdata is not None and event.ydata is not None:
            x, y = event.xdata, event.ydata
            self.ax.plot(x, y, 'ro')
            self.ax.set_title(f'Punto objetivo seleccionado: ({x:.2f}, {y:.2f})')
            self.fig.canvas.draw()
            
            # Publicar el punto en /target_point
            point_msg = Point()
            point_msg.x = x
            point_msg.y = y
            point_msg.z = 0.0
            self.target_pub.publish(point_msg)

            # Opcional: desconectar el evento tras el primer clic
            self.fig.canvas.mpl_disconnect(self.cid)

    def run(self):
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher(width=50, height=50)
    node.run()  # Aquí corre el loop de matplotlib
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

