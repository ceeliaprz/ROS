# importa las clases necesarias
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Nodo 1: informacion_personal_nodo
        Node(
            package='interaccion',
            executable='informacion_personal_nodo',
            name='informacion_personal_nodo',
            output='screen',
            emulate_tty=True,
            prefix='gnome-terminal --',
        ),
        # Nodo 2: emocion_usuario_nodo
        Node(
            package='interaccion',
            executable='emocion_usuario_nodo',
            name='emocion_usuario_nodo',
            output='screen',
            emulate_tty=True,
            prefix='gnome-terminal --',
        ),
        # Nodo 3: posicion_usuario_nodo
        Node(
            package='interaccion',
            executable='posicion_usuario_nodo',
            name='posicion_usuario_nodo',
            output='screen',
            emulate_tty=True,
            prefix='gnome-terminal --',
        ),
        # Nodo 4: empaquetador_nodo
        Node(
            package='interaccion',
            executable='empaquetador_nodo',
            name='empaquetador_nodo',
            output='screen',
        ),
        # Nodo 5: dialogo_nodo
        Node(
            package='interaccion',
            executable='dialogo_nodo',
            name='dialogo_nodo',
            output='screen',
            # emulate_tty=True,
            # prefix='gnome-terminal --',
        ),
        # Nodo 6: matematico_nodo
        Node(
            package='interaccion',
            executable='matematico_nodo',
            name='matematico_nodo',
            output='screen',
            # emulate_tty=True,
            # prefix='gnome-terminal --',
        ),
    ])
