import rclpy
from rclpy.node import Node
from custom_msg.msg import EmocionUsuario  # Asegúrate de ajustar el nombre del mensaje

class EmocionUsuarioNode(Node):
    def __init__(self):
        # Inicializa el nodo con el nombre 'emocion_usuario_nodo'
        super().__init__('emocion_usuario_nodo')

    def run(self):
        # Crea un publicador para el mensaje EmocionUsuario en el tópico 'emocion_topic'
        publisher = self.create_publisher(EmocionUsuario, 'emocion_topic', 10)  # Creamos el publicador

        # Bucle principal mientras el nodo esté en funcionamiento
        while rclpy.ok():
            # Solicita al usuario que ingrese su emoción
            emocion = input("Ingrese cómo se siente: ")

            # Crea un objeto EmocionUsuario y asigna la emoción ingresada
            msg = EmocionUsuario()
            msg.emocion = emocion

            # Publica el mensaje en el tópico
            publisher.publish(msg)
            
            # Registra un mensaje de información en el sistema de registro de ROS
            self.get_logger().info("Información personal publicada")

        
def main(args=None):
    # Inicializa el sistema ROS
    rclpy.init(args=args)

    # Crea una instancia del nodo EmocionUsuarioNode
    emocion_usuario_nodo = EmocionUsuarioNode()

    try:
        # Ejecuta el bucle principal del nodo
        emocion_usuario_nodo.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Realiza acciones de limpieza antes de cerrar el nodo
        emocion_usuario_nodo.destroy_node()
        
        # Apaga el sistema ROS
        rclpy.shutdown()


if __name__ == '__main__':
    # Llama a la función principal si este script es el punto de entrada principal
    main()
