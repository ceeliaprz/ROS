import rclpy
from rclpy.node import Node
from custom_msg.msg import PosicionUsuario  # Asegúrate de ajustar el nombre del mensaje

class PosicionUsuarioNode(Node):
    def __init__(self):
        # Constructor de la clase, inicializa el nodo con el nombre 'posicion_usuario_nodo'
        super().__init__('posicion_usuario_nodo')

    def run(self):
        # Crea un publicador para el mensaje PosicionUsuario en el tópico 'pos_usuario_topic'
        publisher = self.create_publisher(PosicionUsuario, 'pos_usuario_topic', 10)

        while rclpy.ok():
            # Bucle en el cual se recopilan las coordenadas del usuario a través del teclado
            x = input("Ingrese su coordenada x: ")
            y = input("Ingrese su coordenada y: ")
            z = input("Ingrese su coordenada z: ")

            # Publica la información en un mensaje ROS
            msg = PosicionUsuario()
            msg.x = int(x)
            msg.y = int(y)
            msg.z = int(z)

            publisher.publish(msg)
            self.get_logger().info("Información de posición publicada")

def main(args=None):
    # Inicializa el sistema ROS
    rclpy.init(args=args)

    # Crea una instancia del nodo PosicionUsuarioNode
    posicion_usuario_nodo = PosicionUsuarioNode()

    try:
        # Ejecuta el bucle principal del nodo
        posicion_usuario_nodo.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Realiza acciones de limpieza antes de cerrar el nodo
        posicion_usuario_nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # Llama a la función principal si este script es el punto de entrada principal
    main()
