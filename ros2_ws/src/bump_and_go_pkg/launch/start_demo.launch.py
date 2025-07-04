# launch/start_demo.launch.py (Versión FINAL que usa un mundo externo)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. CONFIGURACIÓN DE RUTAS ---


    print("="*60)
    print("INICIANDO DEPURACIÓN DEL LAUNCH FILE")
    
    gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH')
    print(f"VALOR DE GZ_SIM_RESOURCE_PATH: {gz_path}")
    
    print("="*60)    
    # Ruta al paquete de simulación de TurtleBot3
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    world_fuel_url = 'https://fuel.gazebosim.org/1.0/Adlink/worlds/Industrial%20Warehouse/1/industrial_warehouse.sdf'
    
    # --- RUTA ABSOLUTA Y EXTERNA AL ARCHIVO DEL MUNDO ---
    # Usamos os.path.expanduser('~') para obtener la ruta a tu directorio home (/home/algarrid)
    # Esto hace que el launch file sea portable a otras máquinas.
    #world_path = os.path.join(
    #    os.path.expanduser('~'), 
    #    'gazebo_models_worlds_collection',
    #    'worlds',
    #    'office_small.world'
    #)
    
    #models_path = os.path.join(
    #    os.path.expanduser('~'), 
    #    'gazebo_models_worlds_collection'
    #)

    # --- 2. ACCIÓN PARA FORZAR LA VARIABLE DE ENTORNO ---
    # Esto garantiza que todos los sub-procesos, incluido Gazebo, hereden la ruta.
    #set_gazebo_resource_path = SetEnvironmentVariable(
    #    name='GZ_SIM_RESOURCE_PATH',
    #    value=models_path
    #)

    # --- 2. LANZAMIENTO DE LA SIMULACIÓN (GAZEBO + ROBOT) ---

    # Incluimos el launch file de TurtleBot3 y le pasamos la ruta absoluta a nuestro mundo.
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        #launch_arguments={'world': world_path}.items()
        launch_arguments={'world': world_fuel_url}.items()
    )

    # --- 3. LANZAMIENTO DE LA VISUALIZACIÓN (RVIZ2) ---
    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_turtlebot3_gazebo, 'rviz', 'turtlebot3_gazebo.rviz')],
        output='screen'
    )

    # --- 4. LANZAMIENTO DE NUESTRO NODO "CEREBRO" ---
    start_bump_and_go_node = Node(
        package='bump_and_go_pkg',
        executable='driver',
        name='bump_and_go_driver',
        output='screen'
    )

    # --- 5. CREACIÓN DE LA DESCRIPCIÓN DE LANZAMIENTO ---
    return LaunchDescription([
    	#set_gazebo_resource_path,
        start_world,
        start_rviz2,
        start_bump_and_go_node,
    ])
