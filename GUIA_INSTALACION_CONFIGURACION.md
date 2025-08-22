# Guía de Instalación y Configuración - Sistema AVIM

## Tabla de Contenidos
1. [Requisitos del Sistema](#requisitos-del-sistema)
2. [Instalación de Dependencias](#instalación-de-dependencias)
3. [Configuración del Entorno ROS](#configuración-del-entorno-ros)
4. [Instalación del Sistema AVIM](#instalación-del-sistema-avim)
5. [Configuración de Sensores](#configuración-de-sensores)
6. [Configuración de Gazebo](#configuración-de-gazebo)
7. [Compilación del Proyecto](#compilación-del-proyecto)
8. [Verificación de la Instalación](#verificación-de-la-instalación)
9. [Configuración Avanzada](#configuración-avanzada)
10. [Solución de Problemas Comunes](#solución-de-problemas-comunes)

---

## Requisitos del Sistema

### Especificaciones Mínimas de Hardware

```yaml
hardware_requirements:
  cpu:
    cores: 4
    frequency: 2.5 GHz
    architecture: x86_64
  
  memory:
    ram: 8 GB
    swap: 4 GB
  
  storage:
    free_space: 20 GB
    type: SSD (recomendado)
  
  graphics:
    gpu: Dedicada (recomendado para Gazebo)
    vram: 2 GB
    opengl: 3.3+
```

### Especificaciones Recomendadas

```yaml
recommended_specs:
  cpu:
    cores: 8
    frequency: 3.0+ GHz
    model: Intel i7/AMD Ryzen 7+
  
  memory:
    ram: 16 GB
    swap: 8 GB
  
  storage:
    free_space: 50 GB
    type: NVMe SSD
  
  graphics:
    gpu: NVIDIA GTX 1060+ / AMD RX 580+
    vram: 4+ GB
```

### Sistemas Operativos Soportados

- **Ubuntu 18.04 LTS** (Recomendado)
- **Ubuntu 20.04 LTS**
- **Ubuntu 16.04 LTS** (Soporte limitado)

---

## Instalación de Dependencias

### 1. Actualización del Sistema

```bash
# Actualizar lista de paquetes
sudo apt update

# Actualizar sistema completo
sudo apt upgrade -y

# Instalar herramientas básicas
sudo apt install -y curl wget git vim build-essential
```

### 2. Instalación de ROS Melodic (Ubuntu 18.04)

```bash
# Configurar repositorios de ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Agregar claves de autenticación
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Actualizar índice de paquetes
sudo apt update

# Instalación completa de ROS Melodic
sudo apt install -y ros-melodic-desktop-full

# Inicializar rosdep
sudo rosdep init
rosdep update
```

### 3. Instalación de ROS Noetic (Ubuntu 20.04)

```bash
# Configurar repositorios de ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Agregar claves de autenticación
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Actualizar índice de paquetes
sudo apt update

# Instalación completa de ROS Noetic
sudo apt install -y ros-noetic-desktop-full

# Inicializar rosdep
sudo rosdep init
rosdep update
```

### 4. Configuración del Entorno ROS

```bash
# Agregar ROS al bashrc
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc  # Para Melodic
# echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc   # Para Noetic

# Recargar configuración
source ~/.bashrc

# Instalar herramientas adicionales
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool
```

### 5. Instalación de Dependencias de OpenCV

```bash
# Librerías de desarrollo de OpenCV
sudo apt install -y libopencv-dev python-opencv

# Dependencias adicionales para procesamiento de imágenes
sudo apt install -y libcv-bridge-dev libimage-transport-dev

# Herramientas de visión por computadora
sudo apt install -y ros-melodic-cv-bridge ros-melodic-image-transport
```

### 6. Instalación de Gazebo

```bash
# Gazebo ya viene incluido con ROS desktop-full, pero verificar versión
gazebo --version

# Si es necesario, instalar plugins adicionales
sudo apt install -y ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control

# Modelos adicionales de Gazebo
sudo apt install -y gazebo9-model-editor gazebo9-models
```

### 7. Dependencias Específicas del Proyecto

```bash
# Paquetes ROS adicionales necesarios
sudo apt install -y \
    ros-melodic-sensor-msgs \
    ros-melodic-geometry-msgs \
    ros-melodic-std-msgs \
    ros-melodic-nav-msgs \
    ros-melodic-tf2 \
    ros-melodic-tf2-ros \
    ros-melodic-laser-geometry \
    ros-melodic-pcl-ros

# Herramientas de desarrollo
sudo apt install -y \
    python-catkin-tools \
    python-rosdep \
    python-rosinstall \
    python-vcstools
```

---

## Configuración del Entorno ROS

### 1. Crear Workspace de Catkin

```bash
# Crear directorio del workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/

# Inicializar workspace
catkin_make

# Agregar workspace al bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Configurar Variables de Entorno

```bash
# Agregar al ~/.bashrc
echo 'export ROS_WORKSPACE=~/catkin_ws' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src' >> ~/.bashrc

# Configuración de red ROS (para uso local)
echo 'export ROS_MASTER_URI=http://localhost:11311' >> ~/.bashrc
echo 'export ROS_HOSTNAME=localhost' >> ~/.bashrc

# Recargar configuración
source ~/.bashrc
```

### 3. Verificar Configuración ROS

```bash
# Verificar variables de entorno
echo $ROS_DISTRO
echo $ROS_PACKAGE_PATH
echo $ROS_MASTER_URI

# Probar ROS básico
roscore &
sleep 5
rostopic list
killall roscore
```

---

## Instalación del Sistema AVIM

### 1. Clonar el Repositorio

```bash
# Navegar al directorio src del workspace
cd ~/catkin_ws/src

# Clonar el proyecto AVIM (ajustar URL según repositorio)
git clone <URL_DEL_REPOSITORIO> AVIM_folder

# Alternativamente, copiar desde directorio existente
# cp -r /ruta/al/proyecto/AVIM_folder .
```

### 2. Estructura del Proyecto

Verificar que la estructura sea correcta:

```
~/catkin_ws/src/AVIM_folder/
├── AVIM/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── launch/
│       ├── navigation_without_obstacles.launch
│       ├── navigation_with_static_obstacles.launch
│       ├── navigation_with_dynamic_obstacles.launch
│       └── parking.launch
├── control/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src/
│       ├── Master.cpp
│       ├── Master_static.cpp
│       └── Master_parking.cpp
├── lane_detection/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src/
│       └── lane_detection.cpp
├── object_detection/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src/
│       └── object_detection.cpp
├── object_detection_parking/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src/
│       └── object_detection_parking.cpp
└── autonomos_gazebo_simulation/
    ├── launch/
    ├── models/
    └── worlds/
```

### 3. Instalar Dependencias del Proyecto

```bash
# Navegar al workspace
cd ~/catkin_ws

# Instalar dependencias usando rosdep
rosdep install --from-paths src --ignore-src -r -y

# Si hay dependencias faltantes, instalar manualmente
sudo apt install -y \
    libpcl-dev \
    libpcl-conversions-dev \
    libeigen3-dev
```

---

## Configuración de Sensores

### 1. Configuración de Cámara

#### Archivo: `config/camera_params.yaml`

```yaml
camera_config:
  # Parámetros intrínsecos
  image_width: 640
  image_height: 480
  camera_matrix:
    rows: 3
    cols: 3
    data: [500.0, 0.0, 320.0,
           0.0, 500.0, 240.0,
           0.0, 0.0, 1.0]
  
  # Coeficientes de distorsión
  distortion_coefficients:
    rows: 1
    cols: 5
    data: [0.1, -0.2, 0.0, 0.0, 0.0]
  
  # Configuración de captura
  fps: 30
  auto_exposure: true
  brightness: 50
  contrast: 50
```

#### Script de Calibración de Cámara

```bash
#!/bin/bash
# calibrate_camera.sh

echo "Iniciando calibración de cámara..."

# Crear directorio para imágenes de calibración
mkdir -p ~/catkin_ws/calibration_images

# Capturar imágenes de tablero de ajedrez
rosrun camera_calibration cameracalibrator.py \
    --size 8x6 \
    --square 0.024 \
    image:=/camera/image_raw \
    camera:=/camera

echo "Calibración completada. Revisar archivos en ~/.ros/camera_info/"
```

### 2. Configuración de LIDAR

#### Archivo: `config/lidar_params.yaml`

```yaml
lidar_config:
  # Parámetros del sensor
  frame_id: "laser"
  angle_min: -3.14159  # -180 grados
  angle_max: 3.14159   # +180 grados
  angle_increment: 0.00436  # ~0.25 grados
  
  # Rangos de medición
  range_min: 0.2       # metros
  range_max: 4.25      # metros
  
  # Frecuencia de publicación
  scan_time: 0.025     # 40 Hz
  
  # Filtros
  intensity_threshold: 100
  noise_filter: true
  
  # Transformación
  position:
    x: 0.0
    y: 0.0
    z: 0.1
  orientation:
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

### 3. Configuración de Transformaciones (TF)

#### Archivo: `config/robot_transforms.yaml`

```yaml
# Transformaciones entre marcos de referencia
transforms:
  base_link_to_camera:
    translation: [0.3, 0.0, 0.2]  # x, y, z en metros
    rotation: [0.0, 0.0, 0.0]     # roll, pitch, yaw en radianes
  
  base_link_to_laser:
    translation: [0.2, 0.0, 0.1]
    rotation: [0.0, 0.0, 0.0]
  
  base_link_to_imu:
    translation: [0.0, 0.0, 0.05]
    rotation: [0.0, 0.0, 0.0]
```

---

## Configuración de Gazebo

### 1. Configuración de Mundos

#### Mundo Básico: `worlds/empty_world.world`

```xml
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <!-- Configuración de física -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Iluminación -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Suelo -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Configuración de la cámara de Gazebo -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>5.0 -5.0 2.0 0.0 0.275 2.356</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
  </world>
</sdf>
```

### 2. Modelo del Vehículo

#### Archivo URDF: `models/autonomos_vehicle.urdf`

```xml
<?xml version="1.0"?>
<robot name="autonomos_vehicle">
  <!-- Chasis principal -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="2.0 1.0 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="2.0 1.0 0.5"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="1000"/>
      <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100"/>
    </inertial>
  </link>
  
  <!-- Cámara -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
  
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.9 0.0 0.3" rpy="0 0 0"/>
  </joint>
  
  <!-- LIDAR -->
  <link name="laser_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
  
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.8 0.0 0.4" rpy="0 0 0"/>
  </joint>
  
  <!-- Plugins de Gazebo -->
  <gazebo reference="base_link">
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>100</updateRate>
      <leftJoint>left_wheel_joint</leftJoint>
      <rightJoint>right_wheel_joint</rightJoint>
      <wheelSeparation>1.2</wheelSeparation>
      <wheelDiameter>0.6</wheelDiameter>
      <torque>20</torque>
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>base_link</robotBaseFrame>
    </plugin>
  </gazebo>
  
  <gazebo reference="camera_link">
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>30</updateRate>
      <cameraName>camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
      <hackBaseline>0.07</hackBaseline>
      <distortionK1>0.0</distortionK1>
      <distortionK2>0.0</distortionK2>
      <distortionK3>0.0</distortionK3>
      <distortionT1>0.0</distortionT1>
      <distortionT2>0.0</distortionT2>
    </plugin>
  </gazebo>
  
  <gazebo reference="laser_link">
    <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
      <topicName>scan</topicName>
      <frameName>laser_link</frameName>
    </plugin>
  </gazebo>
</robot>
```

### 3. Launch Files de Gazebo

#### Archivo: `launch/gazebo_empty_world.launch`

```xml
<launch>
  <!-- Argumentos -->
  <arg name="world_name" default="$(find autonomos_gazebo_simulation)/worlds/empty_world.world"/>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>
  
  <!-- Configurar parámetros -->
  <param name="use_sim_time" value="$(arg use_sim_time)"/>
  
  <!-- Lanzar Gazebo -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(arg world_name)"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>
  
  <!-- Spawn del vehículo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" 
        args="-file $(find autonomos_gazebo_simulation)/models/autonomos_vehicle.urdf 
              -urdf -x 0 -y 0 -z 0.1 -model autonomos_vehicle"/>
  
  <!-- Publicar transformaciones estáticas -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_camera"
        args="0.3 0.0 0.2 0.0 0.0 0.0 base_link camera_link"/>
  
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_laser"
        args="0.2 0.0 0.1 0.0 0.0 0.0 base_link laser_link"/>
</launch>
```

---

## Compilación del Proyecto

### 1. Compilación Inicial

```bash
# Navegar al workspace
cd ~/catkin_ws

# Limpiar compilaciones anteriores (si existen)
catkin_make clean

# Compilar todos los paquetes
catkin_make

# Verificar que no hay errores
echo $?
```

### 2. Compilación Específica por Paquete

```bash
# Compilar solo un paquete específico
catkin_make --only-pkg-with-deps lane_detection
catkin_make --only-pkg-with-deps object_detection
catkin_make --only-pkg-with-deps control
```

### 3. Configuración de Compilación Optimizada

```bash
# Compilación con optimizaciones
catkin_make -DCMAKE_BUILD_TYPE=Release

# Compilación en paralelo (usar número de cores disponibles)
catkin_make -j$(nproc)

# Compilación con información de debug
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

### 4. Script de Compilación Automatizada

#### Archivo: `scripts/build_avim.sh`

```bash
#!/bin/bash
# Script de compilación automatizada para AVIM

set -e  # Salir en caso de error

echo "=== Iniciando compilación del sistema AVIM ==="

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Función para imprimir mensajes
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Verificar que estamos en el workspace correcto
if [ ! -f "src/AVIM_folder/AVIM/package.xml" ]; then
    print_error "No se encontró el proyecto AVIM. Verificar ubicación."
    exit 1
fi

print_status "Workspace encontrado: $(pwd)"

# Instalar dependencias
print_status "Instalando dependencias..."
rosdep install --from-paths src --ignore-src -r -y

# Limpiar compilación anterior
print_status "Limpiando compilación anterior..."
catkin_make clean

# Compilar proyecto
print_status "Compilando proyecto AVIM..."
if catkin_make -j$(nproc); then
    print_status "Compilación exitosa!"
else
    print_error "Error en la compilación"
    exit 1
fi

# Source del workspace
print_status "Configurando entorno..."
source devel/setup.bash

# Verificar paquetes
print_status "Verificando paquetes instalados..."
rospack list | grep -E "(lane_detection|object_detection|control|AVIM)"

print_status "=== Compilación completada exitosamente ==="
print_warning "Recuerda ejecutar: source ~/catkin_ws/devel/setup.bash"
```

---

## Verificación de la Instalación

### 1. Verificación de Paquetes ROS

```bash
# Verificar que los paquetes están disponibles
rospack find lane_detection
rospack find object_detection
rospack find control
rospack find AVIM

# Listar todos los paquetes del proyecto
rospack list | grep -E "(lane_detection|object_detection|control|AVIM)"
```

### 2. Verificación de Nodos

```bash
# Verificar que los ejecutables existen
rosrun lane_detection lane_detection --help
rosrun object_detection object_detection --help
rosrun control Master --help
```

### 3. Prueba de Launch Files

```bash
# Verificar sintaxis de launch files
roslaunch --check AVIM navigation_without_obstacles.launch
roslaunch --check AVIM navigation_with_static_obstacles.launch
roslaunch --check AVIM navigation_with_dynamic_obstacles.launch
roslaunch --check AVIM parking.launch
```

### 4. Prueba de Gazebo

```bash
# Probar lanzamiento de Gazebo
roslaunch autonomos_gazebo_simulation gazebo_empty_world.launch gui:=false &
sleep 10

# Verificar que el modelo se cargó
rostopic list | grep -E "(camera|scan|cmd_vel)"

# Cerrar Gazebo
killall gzserver gzclient
```

### 5. Script de Verificación Completa

#### Archivo: `scripts/verify_installation.sh`

```bash
#!/bin/bash
# Script de verificación de instalación AVIM

echo "=== Verificación de Instalación AVIM ==="

# Colores
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

pass_count=0
fail_count=0

check_command() {
    if eval "$1" &>/dev/null; then
        echo -e "${GREEN}✓${NC} $2"
        ((pass_count++))
    else
        echo -e "${RED}✗${NC} $2"
        ((fail_count++))
    fi
}

check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} Archivo encontrado: $1"
        ((pass_count++))
    else
        echo -e "${RED}✗${NC} Archivo faltante: $1"
        ((fail_count++))
    fi
}

echo "\n--- Verificando ROS ---"
check_command "rosversion -d" "ROS instalado"
check_command "rospack list | grep -q ros" "Paquetes ROS disponibles"

echo "\n--- Verificando Dependencias ---"
check_command "pkg-config --exists opencv" "OpenCV instalado"
check_command "dpkg -l | grep -q gazebo" "Gazebo instalado"

echo "\n--- Verificando Workspace ---"
check_file "~/catkin_ws/devel/setup.bash"
check_file "~/catkin_ws/src/AVIM_folder/AVIM/package.xml"

echo "\n--- Verificando Paquetes AVIM ---"
check_command "rospack find lane_detection" "Paquete lane_detection"
check_command "rospack find object_detection" "Paquete object_detection"
check_command "rospack find control" "Paquete control"
check_command "rospack find AVIM" "Paquete AVIM"

echo "\n--- Verificando Ejecutables ---"
check_file "~/catkin_ws/devel/lib/lane_detection/lane_detection"
check_file "~/catkin_ws/devel/lib/object_detection/object_detection"
check_file "~/catkin_ws/devel/lib/control/Master"

echo "\n--- Verificando Launch Files ---"
check_file "~/catkin_ws/src/AVIM_folder/AVIM/launch/navigation_without_obstacles.launch"
check_file "~/catkin_ws/src/AVIM_folder/AVIM/launch/navigation_with_static_obstacles.launch"

echo "\n=== Resumen ==="
echo -e "${GREEN}Pruebas exitosas: $pass_count${NC}"
echo -e "${RED}Pruebas fallidas: $fail_count${NC}"

if [ $fail_count -eq 0 ]; then
    echo -e "${GREEN}\n🎉 Instalación verificada exitosamente!${NC}"
    exit 0
else
    echo -e "${RED}\n❌ Se encontraron problemas en la instalación${NC}"
    exit 1
fi
```

---

## Configuración Avanzada

### 1. Optimización de Rendimiento

#### Configuración de CPU

```bash
# Configurar governor de CPU para rendimiento
echo 'performance' | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Verificar configuración
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

#### Configuración de Memoria

```bash
# Aumentar límites de memoria compartida
echo 'kernel.shmmax = 268435456' | sudo tee -a /etc/sysctl.conf
echo 'kernel.shmall = 2097152' | sudo tee -a /etc/sysctl.conf

# Aplicar cambios
sudo sysctl -p
```

### 2. Configuración de Red para ROS

#### Para uso en múltiples máquinas

```bash
# En ~/.bashrc del master
export ROS_MASTER_URI=http://MASTER_IP:11311
export ROS_HOSTNAME=MASTER_IP

# En ~/.bashrc de los clientes
export ROS_MASTER_URI=http://MASTER_IP:11311
export ROS_HOSTNAME=CLIENT_IP
```

### 3. Configuración de Logging

#### Archivo: `config/logging.conf`

```ini
[loggers]
keys=root,avim

[handlers]
keys=consoleHandler,fileHandler

[formatters]
keys=simpleFormatter,detailedFormatter

[logger_root]
level=INFO
handlers=consoleHandler

[logger_avim]
level=DEBUG
handlers=consoleHandler,fileHandler
qualname=avim
propagate=0

[handler_consoleHandler]
class=StreamHandler
level=INFO
formatter=simpleFormatter
args=(sys.stdout,)

[handler_fileHandler]
class=FileHandler
level=DEBUG
formatter=detailedFormatter
args=('/tmp/avim.log',)

[formatter_simpleFormatter]
format=%(levelname)s - %(message)s

[formatter_detailedFormatter]
format=%(asctime)s - %(name)s - %(levelname)s - %(message)s
```

### 4. Configuración de Monitoreo

#### Script de Monitoreo: `scripts/monitor_system.sh`

```bash
#!/bin/bash
# Monitor de sistema para AVIM

while true; do
    echo "=== $(date) ==="
    
    # CPU y memoria
    echo "CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
    echo "RAM: $(free | grep Mem | awk '{printf "%.1f%%", $3/$2 * 100.0}')"
    
    # Procesos ROS
    echo "Nodos ROS activos: $(rosnode list 2>/dev/null | wc -l)"
    
    # Tópicos activos
    echo "Tópicos activos: $(rostopic list 2>/dev/null | wc -l)"
    
    # Frecuencias importantes
    if rostopic list 2>/dev/null | grep -q "/camera/image_raw"; then
        echo "Frecuencia cámara: $(rostopic hz /camera/image_raw --window=10 2>/dev/null | tail -1)"
    fi
    
    echo "---"
    sleep 5
done
```

---

## Solución de Problemas Comunes

### 1. Problemas de Compilación

#### Error: "Could not find a package configuration file"

```bash
# Instalar dependencias faltantes
sudo apt update
sudo apt install -y libopencv-dev libpcl-dev

# Reinstalar paquetes ROS
sudo apt install --reinstall ros-melodic-cv-bridge ros-melodic-pcl-ros
```

#### Error: "No rule to make target"

```bash
# Limpiar y recompilar
cd ~/catkin_ws
catkin_make clean
rm -rf build/ devel/
catkin_make
```

### 2. Problemas de Ejecución

#### Error: "roscore not found"

```bash
# Verificar instalación de ROS
echo $ROS_DISTRO
source /opt/ros/melodic/setup.bash

# Agregar al bashrc si no está
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```

#### Error: "Package not found"

```bash
# Verificar workspace
source ~/catkin_ws/devel/setup.bash
rospack profile

# Recompilar si es necesario
cd ~/catkin_ws
catkin_make
```

### 3. Problemas de Gazebo

#### Gazebo no inicia o se cierra inesperadamente

```bash
# Verificar drivers gráficos
glxinfo | grep "OpenGL version"

# Reiniciar servicios gráficos
sudo service gdm3 restart

# Limpiar cache de Gazebo
rm -rf ~/.gazebo/log/*
```

#### Modelos no se cargan

```bash
# Verificar path de modelos
echo $GAZEBO_MODEL_PATH

# Descargar modelos básicos
cd ~/.gazebo/models
wget -r -np -nH --cut-dirs=2 http://models.gazebosim.org/
```

### 4. Problemas de Sensores

#### Cámara no detectada

```bash
# Verificar dispositivos de video
ls /dev/video*

# Probar con v4l2
v4l2-ctl --list-devices

# Instalar herramientas de cámara
sudo apt install -y v4l-utils cheese
```

#### LIDAR no publica datos

```bash
# Verificar permisos de puerto serie
sudo usermod -a -G dialout $USER

# Verificar conexión
rostopic echo /scan --noarr

# Verificar frecuencia
rostopic hz /scan
```

### 5. Script de Diagnóstico

#### Archivo: `scripts/diagnose_issues.sh`

```bash
#!/bin/bash
# Script de diagnóstico para problemas comunes

echo "=== Diagnóstico del Sistema AVIM ==="

# Verificar ROS
echo "\n--- ROS ---"
echo "Distribución: $(rosversion -d 2>/dev/null || echo 'NO DETECTADA')"
echo "Master URI: $ROS_MASTER_URI"
echo "Hostname: $ROS_HOSTNAME"

# Verificar procesos
echo "\n--- Procesos ---"
echo "roscore: $(pgrep roscore >/dev/null && echo 'EJECUTÁNDOSE' || echo 'DETENIDO')"
echo "Gazebo: $(pgrep gzserver >/dev/null && echo 'EJECUTÁNDOSE' || echo 'DETENIDO')"

# Verificar recursos
echo "\n--- Recursos ---"
echo "CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)% usado"
echo "RAM: $(free | grep Mem | awk '{printf "%.1f%%", $3/$2 * 100.0}') usado"
echo "Disco: $(df -h ~ | tail -1 | awk '{print $5}') usado"

# Verificar conectividad
echo "\n--- Conectividad ---"
if rostopic list &>/dev/null; then
    echo "ROS Master: CONECTADO"
    echo "Tópicos disponibles: $(rostopic list | wc -l)"
else
    echo "ROS Master: DESCONECTADO"
fi

# Verificar logs
echo "\n--- Logs Recientes ---"
if [ -f ~/.ros/log/latest/roslaunch-*.log ]; then
    echo "Últimos errores en roslaunch:"
    tail -5 ~/.ros/log/latest/roslaunch-*.log | grep -i error
fi

echo "\n=== Diagnóstico Completado ==="
```

---

**Guía de Instalación y Configuración - Sistema AVIM**

*Versión 1.0 - Instalación completa paso a paso*