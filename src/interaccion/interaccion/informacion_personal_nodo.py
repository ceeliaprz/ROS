import rclpy #Biblioteca para crear nodos
from rclpy.node import Node
from custom_msg.msg import InfPersonalUsuario  # Asegúrate de ajustar el nombre del mensaje

class InformacionPersonalNodo(Node):
    def __init__(self):
        # Constructor de la clase, inicializa el nodo con el nombre 'informacion_personal_nodo'
        super().__init__('informacion_personal_nodo')

    def run(self):
        # Crea un publicador para el mensaje InfPersonalUsuario en el tópico 'inf_pers_topic'
        publisher = self.create_publisher(InfPersonalUsuario, 'inf_pers_topic', 10)

        while rclpy.ok():
            # Bucle en el cual se recopila la información a través del teclado
            nombre = input("Ingrese el nombre del usuario: ")
            edad = int(input("Ingrese la edad del usuario: "))
            idiomas = input("Ingrese los idiomas que habla (separados por comas): ").split(',')

            # Publica la información en un mensaje ROS
            msg = InfPersonalUsuario()
            msg.nombre = nombre
            msg.edad = edad
            msg.idiomas = idiomas

            publisher.publish(msg)
            self.get_logger().info("Información personal publicada")

def main(args=None):
    # Inicializa el sistema ROS
    rclpy.init(args=args)

    # Crea una instancia del nodo InformacionPersonalNodo
    informacion_personal_nodo = InformacionPersonalNodo()

    try:
        # Ejecuta el bucle principal del nodo
        informacion_personal_nodo.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Realiza acciones de limpieza antes de cerrar el nodo
        informacion_personal_nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # Llama a la función principal si este script es el punto de entrada principal
    main()
