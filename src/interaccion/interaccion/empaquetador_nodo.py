import rclpy
from rclpy.node import Node
from custom_msg.msg import InfPersonalUsuario  # Asegúrate de ajustar el nombre del mensaje
from custom_msg.msg import EmocionUsuario  # Asegúrate de ajustar el nombre del mensaje
from custom_msg.msg import PosicionUsuario  # Asegúrate de ajustar el nombre del mensaje
from custom_msg.msg import Usuario  # Asegúrate de ajustar el nombre del mensaje

class EmpaquetadorNodo(Node):
    def __init__(self):
        # Inicializa el nodo con el nombre 'empaquetador_nodo'
        super().__init__('empaquetador_nodo')

        # Crea tres suscripciones a diferentes tópicos
        self.subscriber1 = self.create_subscription(InfPersonalUsuario, 'inf_pers_topic', self.callback_nodo1, 10)
        self.subscriber2 = self.create_subscription(EmocionUsuario, 'emocion_topic', self.callback_nodo2, 10)
        self.subscriber3 = self.create_subscription(PosicionUsuario, 'pos_usuario_topic', self.callback_nodo3, 10)

        # Crea un publicador para el tópico 'user_topic'
        self.dialogo_publisher = self.create_publisher(Usuario, 'user_topic', 10)

        # Inicializa un nuevo mensaje de tipo Usuario con algunos campos establecidos en valores predeterminados.
        self.reset_message()

    def reset_message(self):
        # Método para inicializar un nuevo mensaje de tipo Usuario con algunos campos establecidos en valores predeterminados.
        self.msg = Usuario()
        self.msg.infpersonal.edad = -1
        self.msg.posicion.x = -1
        self.msg.posicion.y = -1
        self.msg.posicion.z = -1

    def callback_nodo1(self, msg):
        # Callback para el primer suscriptor, actualiza los campos relacionados con información personal del mensaje Usuario.
        self.msg.infpersonal.nombre = msg.nombre
        self.msg.infpersonal.edad = msg.edad
        self.msg.infpersonal.idiomas = msg.idiomas
        self.try_publish_message()

    def callback_nodo2(self, msg):
        # Callback para el segundo suscriptor, actualiza el campo de emoción del mensaje Usuario.
        self.msg.emocion = msg.emocion
        self.try_publish_message()

    def callback_nodo3(self, msg):
        # Callback para el tercer suscriptor, actualiza los campos de posición del mensaje Usuario.
        self.msg.posicion.x = msg.x
        self.msg.posicion.y = msg.y
        self.msg.posicion.z = msg.z
        self.try_publish_message()

    def try_publish_message(self):
        # Intenta publicar el mensaje solo si todos los campos necesarios están completos.
        if not (self.msg.infpersonal.nombre == '' or self.msg.infpersonal.edad == -1 or
                self.msg.infpersonal.idiomas == '' or self.msg.emocion == '' or
                self.msg.posicion.x == -1 or self.msg.posicion.y == -1 or self.msg.posicion.z == -1):

            # Publica el mensaje en el tópico 'user_topic'
            self.dialogo_publisher.publish(self.msg)
            self.get_logger().info("Mensaje enviado al nodo dialogo_nodo")
            self.reset_message()  # Vuelve a resetear el mensaje

def main(args=None):
    # Inicializa el sistema ROS
    rclpy.init(args=args)

    # Crea una instancia del nodo EmpaquetadorNodo
    empaquetador_nodo = EmpaquetadorNodo()

    try:
        # Ejecuta el bucle principal del nodo
        rclpy.spin(empaquetador_nodo)
    except KeyboardInterrupt:
        pass

    # Realiza acciones de limpieza antes de cerrar el nodo
    empaquetador_nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Llama a la función principal si este script es el punto de entrada principal
    main()
