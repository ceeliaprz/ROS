from setuptools import setup

package_name = 'interaccion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/interaccion.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='celia',
    maintainer_email='celia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dialogo_nodo = interaccion.dialogo_nodo:main',
            'emocion_usuario_nodo = interaccion.emocion_usuario_nodo:main',
            'empaquetador_nodo = interaccion.empaquetador_nodo:main',
            'informacion_personal_nodo = interaccion.informacion_personal_nodo:main',
            'posicion_usuario_nodo = interaccion.posicion_usuario_nodo:main',
            'matematico_nodo = interaccion.matematico_nodo:main',

        ],
    },
)
