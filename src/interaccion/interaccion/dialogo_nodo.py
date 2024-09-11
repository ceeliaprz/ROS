from custom_msg.msg import Usuario  # Asegúrate de ajustar el nombre del mensaje
from custom_msg.srv import Multiplicador
import rclpy
from rclpy.node import Node

class DialogoNode(Node):
    def __init__(self):
        # Inicializa el nodo con el nombre 'dialogo_nodo'
        super().__init__('dialogo_nodo')

        # Crea una suscripción al tópico 'user_topic' para recibir mensajes de tipo Usuario
        self.subscription = self.create_subscription(Usuario, 'user_topic', self.callback, 10)

        # Crea un cliente para el servicio 'servicio_multiplicador'
        self.cli = self.create_client(Multiplicador, 'servicio_multiplicador')

        # Espera hasta que el servicio esté disponible antes de continuar
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, esperando de nuevo...')

        # Crea la variable que almacenará los datos que se enviarán en la petición al servicio
        self.req = Multiplicador.Request()
        self.edad = None

    def callback(self, msg):
        # Maneja la recepción de mensajes en el tópico 'user_topic'
        self.get_logger().info("--------------Mensaje recibido--------------")
        self.get_logger().info(f"Nombre: {msg.infpersonal.nombre}")
        self.get_logger().info(f"Edad: {msg.infpersonal.edad}")
        self.get_logger().info(f"Idiomas: {msg.infpersonal.idiomas}")
        self.get_logger().info(f"Emoción: {msg.emocion}")
        self.get_logger().info(f"Posición (x, y, z): ({msg.posicion.x}, {msg.posicion.y}, {msg.posicion.z})")
        self.get_logger().info("--------------------------------------------")

        # Almacena la edad del mensaje para enviarla en la petición al servicio
        self.edad = msg.infpersonal.edad
        self.send_request(msg.infpersonal.edad)

    def send_request(self, entrada):
        # Envia una petición al servicio 'servicio_multiplicador' con la edad
        self.req.entrada = entrada
        self.future = self.cli.call_async(self.req).add_done_callback(self.servicio_callback)

    def servicio_callback(self, future):
        # Maneja la respuesta del servicio una vez está disponible
        try:
            response = future.result()
            # Imprime el resultado de la multiplicación de la edad por 2
            self.get_logger().info('Resultado de la multiplicacion: %d * 2 = %d' % (self.edad, response.resultado))

        except Exception as e:
            # Maneja cualquier error en la llamada al servicio
            self.get_logger().info(f'Error en el servicio: {e}')

def main(args=None):
    # Inicializa el sistema ROS
    rclpy.init(args=args)

    # Crea una instancia del nodo DialogoNode
    dialogo_nodo = DialogoNode()
    
    try:
        # Ejecuta el bucle principal del nodo
        rclpy.spin(dialogo_nodo)
    except KeyboardInterrupt:
        pass
    finally:
        # Realiza acciones de limpieza antes de cerrar el nodo
        dialogo_nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # Llama a la función principal si este script es el punto de entrada principal
    main()
