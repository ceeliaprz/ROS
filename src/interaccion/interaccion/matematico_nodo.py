from custom_msg.srv import Multiplicador

import rclpy  # Importa la biblioteca cliente Python de ROS 2
from rclpy.node import Node

class MatematicoNodo(Node):
    def __init__(self):
        # Constructor de la clase, inicializa el nodo con el nombre 'matematico_nodo'
        super().__init__('matematico_nodo')

        # Crea un servicio para el servicio 'servicio_multiplicador' con la función de retorno de llamada 'multiplicador_callback'
        self.srv = self.create_service(Multiplicador, 'servicio_multiplicador', self.multiplicador_callback)
    
    def multiplicador_callback(self, request, response):
        # Función de retorno de llamada para manejar las peticiones al servicio 'servicio_multiplicador'
        response.resultado = request.entrada * 2
        self.get_logger().info('Solicitud entrante: %d ' % (request.entrada))
        return response
    
def main(args=None):
    # Inicializa el sistema ROS
    rclpy.init(args=args)
    
    # Crea una instancia del nodo MatematicoNodo
    matematico_nodo = MatematicoNodo()

    # Mantiene corriendo el bucle interno del nodo para poder gestionar las peticiones del cliente
    try:
        rclpy.spin(matematico_nodo)
    except KeyboardInterrupt:
        pass
    
    # Apaga el sistema ROS
    rclpy.shutdown()

if __name__ == '__main__':
    # Llama a la función principal si este script es el punto de entrada principal
    main()
