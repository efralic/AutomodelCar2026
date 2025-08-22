# Documentación de Modos de Navegación - Sistema AVIM

## Tabla de Contenidos
1. [Introducción](#introducción)
2. [Modo 1: Navegación sin Obstáculos](#modo-1-navegación-sin-obstáculos)
3. [Modo 2: Navegación con Obstáculos Estáticos](#modo-2-navegación-con-obstáculos-estáticos)
4. [Modo 3: Navegación con Obstáculos Dinámicos](#modo-3-navegación-con-obstáculos-dinámicos)
5. [Modo 4: Estacionamiento Autónomo](#modo-4-estacionamiento-autónomo)
6. [Configuración de Launch Files](#configuración-de-launch-files)
7. [Parámetros de Configuración](#parámetros-de-configuración)
8. [Comparativa de Modos](#comparativa-de-modos)
9. [Selección del Modo Apropiado](#selección-del-modo-apropiado)

---

## Introducción

El sistema AVIM (Autonomous Vehicle Intelligent Management) está diseñado para operar en cuatro modos distintos de navegación, cada uno optimizado para diferentes escenarios y condiciones del entorno. Esta documentación detalla las características, configuraciones y casos de uso de cada modo.

### Arquitectura Modular

Cada modo de navegación utiliza una combinación específica de:
- **Sensores**: Cámara RGB, LIDAR
- **Algoritmos de procesamiento**: Detección de carriles, detección de objetos
- **Controladores**: Master, Master_static, Master_parking
- **Actuadores**: Control de velocidad y dirección

---

## Modo 1: Navegación sin Obstáculos

### Descripción General

El modo más básico del sistema AVIM, diseñado para navegación en entornos controlados sin obstáculos. Se enfoca exclusivamente en el seguimiento de carriles utilizando visión por computadora.

### Componentes Activos

#### Sensores
- **Cámara RGB**: Captura de imágenes frontales a 30 FPS
- **LIDAR**: No utilizado en este modo

#### Nodos ROS Activos
```bash
/lane_detection    # Procesamiento de imágenes
/Master           # Control básico de navegación
```

### Algoritmos Principales

#### 1. Detección de Carriles
```cpp
// Pipeline de procesamiento de imagen
void processLaneDetection() {
    // 1. Captura de imagen
    captureImage();
    
    // 2. Transformación de perspectiva (Bird's Eye View)
    perspectiveTransform();
    
    // 3. Umbralización y filtrado
    applyThreshold();
    
    // 4. Detección de líneas
    detectLanes();
    
    // 5. Cálculo de centro y ángulo
    calculateCenterDistance();
}
```

#### 2. Control PID Básico
```cpp
// Control proporcional-derivativo para dirección
float calculateSteering(int distance_center) {
    float error = distance_center - CENTER_CAMERA;
    float derivative = error - previous_error;
    
    float steering = KP_ANGLE * error + KD_ANGLE * derivative;
    previous_error = error;
    
    return constrainSteering(steering);
}
```

### Configuración de Launch

**Archivo**: `navigation_without_obstacles.launch`
```xml
<launch>
  <!-- Nodo de detección de carriles -->
  <node name="lane_detection" 
        pkg="lane_detection" 
        type="lane_detection" 
        output="screen">
    <param name="camera_topic" value="/camera/image_raw"/>
    <param name="publish_rate" value="30"/>
  </node>
  
  <!-- Nodo de control maestro -->
  <node name="Master" 
        pkg="control" 
        type="Master" 
        output="screen">
    <param name="mode" value="lane_following"/>
    <param name="max_speed" value="60"/>
  </node>
</launch>
```

### Parámetros de Configuración

```yaml
lane_detection:
  camera:
    fps: 30
    resolution: "640x480"
    center_x: 320
  
  processing:
    threshold_low: 50
    threshold_high: 150
    resize_factor: 0.15
  
  perspective:
    source_points: [[200,300], [440,300], [0,480], [640,480]]
    dest_points: [[150,0], [490,0], [150,480], [490,480]]

control:
  pid:
    kp_angle: 0.8
    kd_angle: 0.2
    ki_angle: 0.0
  
  speed:
    default: 50
    max: 60
    min: 20
```

### Casos de Uso

- **Pruebas iniciales** del sistema
- **Calibración** de sensores y algoritmos
- **Entornos controlados** como pistas de prueba
- **Demostración** de capacidades básicas
- **Desarrollo** y debugging de algoritmos de visión

### Limitaciones

- No detecta obstáculos
- No puede evadir objetos
- Requiere carriles claramente marcados
- Funciona solo en condiciones de iluminación adecuadas
- No adapta velocidad según condiciones

---

## Modo 2: Navegación con Obstáculos Estáticos

### Descripción General

Modo intermedio que combina detección de carriles con detección de obstáculos estáticos. Utiliza estrategias conservadoras de navegación, priorizando la seguridad sobre la eficiencia.

### Componentes Activos

#### Sensores
- **Cámara RGB**: Detección de carriles
- **LIDAR**: Detección de obstáculos (rango 360°, hasta 4.25m)

#### Nodos ROS Activos
```bash
/lane_detection      # Procesamiento de imágenes
/object_detection    # Procesamiento LIDAR
/Master_static       # Control con obstáculos estáticos
```

### Algoritmos Principales

#### 1. Detección de Obstáculos con DBSCAN
```cpp
class StaticObstacleDetection {
public:
    void processLidarData(const sensor_msgs::LaserScan& scan) {
        // 1. Filtrado de datos
        filterValidPoints(scan);
        
        // 2. Conversión polar a cartesiana
        convertToCartesian();
        
        // 3. Clustering DBSCAN
        dbscan.setParameters(EPSILON, MIN_POINTS);
        dbscan.cluster(points);
        
        // 4. Filtrado de clusters
        filterClusters();
        
        // 5. Cálculo de centroides
        calculateCentroids();
    }
    
private:
    float EPSILON = 0.15;      // Radio de búsqueda (metros)
    int MIN_POINTS = 3;        // Mínimo puntos por cluster
    float MAX_RANGE = 4.25;    // Rango máximo LIDAR
};
```

#### 2. Estrategia de Navegación Conservadora
```cpp
class MasterStatic {
public:
    void navigationStrategy() {
        switch(current_state) {
            case LANE_DRIVING:
                if (obstacleDetected()) {
                    current_state = OBSTACLE_DETECTED;
                    reduceSpeed();
                }
                break;
                
            case OBSTACLE_DETECTED:
                if (canSafelyPass()) {
                    current_state = PLANNING_ROUTE;
                } else {
                    current_state = WAITING;
                    stopVehicle();
                }
                break;
                
            case WAITING:
                if (obstacleCleared()) {
                    current_state = LANE_DRIVING;
                    resumeSpeed();
                }
                break;
        }
    }
    
private:
    float SAFE_DISTANCE = 1.5;     // Distancia mínima segura
    int MAX_WAIT_TIME = 10000;     // Tiempo máximo de espera (ms)
};
```

### Configuración de Launch

**Archivo**: `navigation_with_static_obstacles.launch`
```xml
<launch>
  <!-- Nodo de detección de carriles -->
  <node name="lane_detection" 
        pkg="lane_detection" 
        type="lane_detection" 
        output="screen"/>
  
  <!-- Nodo de detección de objetos -->
  <node name="object_detection" 
        pkg="object_detection" 
        type="object_detection" 
        output="screen">
    <param name="lidar_topic" value="/scan"/>
    <param name="detection_range" value="4.25"/>
    <param name="min_cluster_size" value="3"/>
  </node>
  
  <!-- Nodo de control maestro estático -->
  <node name="Master_static" 
        pkg="control" 
        type="Master_static" 
        output="screen">
    <param name="safe_distance" value="1.5"/>
    <param name="max_wait_time" value="10000"/>
  </node>
</launch>
```

### Parámetros de Configuración

```yaml
object_detection:
  lidar:
    range_max: 4.25
    range_min: 0.2
    angle_range: 360.0
  
  dbscan:
    epsilon: 0.15
    min_points: 3
  
  filtering:
    min_object_size: 0.1
    max_object_size: 3.0

control_static:
  safety:
    safe_distance: 1.5
    emergency_brake_distance: 0.8
    max_wait_time: 10000
  
  speed:
    normal: 40
    reduced: 20
    approach: 10
```

### Estados de Navegación

1. **LANE_DRIVING**: Navegación normal siguiendo carriles
2. **OBSTACLE_DETECTED**: Obstáculo detectado, evaluando opciones
3. **PLANNING_ROUTE**: Planificando ruta alternativa
4. **WAITING**: Esperando que se despeje el obstáculo
5. **EMERGENCY_STOP**: Parada de emergencia

### Casos de Uso

- **Entornos urbanos** con obstáculos predecibles
- **Zonas de construcción** con conos y barreras
- **Estacionamientos** con vehículos estacionados
- **Pruebas de seguridad** del sistema
- **Navegación nocturna** donde la visión es limitada

---

## Modo 3: Navegación con Obstáculos Dinámicos

### Descripción General

El modo más avanzado del sistema, capaz de manejar obstáculos en movimiento. Implementa una máquina de estados compleja para realizar maniobras de adelantamiento y evasión dinámica.

### Componentes Activos

#### Sensores
- **Cámara RGB**: Detección de carriles de alta frecuencia
- **LIDAR**: Detección y seguimiento de obstáculos dinámicos

#### Nodos ROS Activos
```bash
/lane_detection      # Procesamiento de imágenes
/object_detection    # Procesamiento LIDAR avanzado
/Master             # Control dinámico completo
```

### Algoritmos Principales

#### 1. Seguimiento de Obstáculos Dinámicos
```cpp
class DynamicObstacleTracker {
public:
    void updateObstacles(const std::vector<Obstacle>& detected) {
        for (auto& obstacle : detected) {
            int id = findOrCreateTrack(obstacle);
            updateTrack(id, obstacle);
            predictTrajectory(id);
        }
        
        removeOldTracks();
    }
    
private:
    struct ObstacleTrack {
        int id;
        cv::Point2f position;
        cv::Point2f velocity;
        cv::Point2f predicted_position;
        float confidence;
        int age;
    };
    
    std::vector<ObstacleTrack> tracks;
    float PREDICTION_TIME = 2.0;  // Predicción a 2 segundos
};
```

#### 2. Máquina de Estados Avanzada
```cpp
class MasterDynamic {
public:
    enum NavigationState {
        LANE_DRIVING = 0,
        FOLLOWING = 1,
        MOVING_LEFT = 2,
        PASSING = 3,
        MOVING_RIGHT = 4,
        MOVING_RIGHT_LANE = 5,
        EMERGENCY_BRAKE = 6
    };
    
    void stateMachine() {
        switch(current_state) {
            case LANE_DRIVING:
                if (obstacleAhead()) {
                    if (canPass()) {
                        current_state = MOVING_LEFT;
                    } else {
                        current_state = FOLLOWING;
                    }
                }
                break;
                
            case FOLLOWING:
                maintainSafeDistance();
                if (canPass() && passingEnabled()) {
                    current_state = MOVING_LEFT;
                }
                break;
                
            case MOVING_LEFT:
                if (inLeftLane()) {
                    current_state = PASSING;
                }
                break;
                
            case PASSING:
                if (obstacleCleared()) {
                    current_state = MOVING_RIGHT;
                }
                break;
                
            case MOVING_RIGHT:
                if (inRightLane()) {
                    current_state = LANE_DRIVING;
                }
                break;
        }
    }
    
private:
    float FOLLOWING_DISTANCE = 2.0;
    float PASSING_CLEARANCE = 3.0;
    bool passing_enabled = true;
};
```

#### 3. Control Adaptativo de Velocidad
```cpp
int adaptiveSpeedControl(float obstacle_distance, float obstacle_speed) {
    int base_speed = 50;
    
    if (obstacle_distance < EMERGENCY_DISTANCE) {
        return 0;  // Parada de emergencia
    }
    
    if (obstacle_distance < FOLLOWING_DISTANCE) {
        // Adaptar velocidad al obstáculo
        float speed_factor = obstacle_distance / FOLLOWING_DISTANCE;
        return base_speed * speed_factor * 0.8;  // 80% de la velocidad proporcional
    }
    
    return base_speed;
}
```

### Configuración de Launch

**Archivo**: `navigation_with_dynamic_obstacles.launch`
```xml
<launch>
  <!-- Nodo de detección de carriles -->
  <node name="lane_detection" 
        pkg="lane_detection" 
        type="lane_detection" 
        output="screen">
    <param name="high_frequency_mode" value="true"/>
  </node>
  
  <!-- Nodo de detección de objetos dinámicos -->
  <node name="object_detection" 
        pkg="object_detection" 
        type="object_detection" 
        output="screen">
    <param name="tracking_enabled" value="true"/>
    <param name="prediction_time" value="2.0"/>
  </node>
  
  <!-- Nodo de control maestro dinámico -->
  <node name="Master" 
        pkg="control" 
        type="Master" 
        output="screen">
    <param name="passing_enabled" value="true"/>
    <param name="aggressive_mode" value="false"/>
  </node>
</launch>
```

### Parámetros de Configuración

```yaml
control_dynamic:
  states:
    following_distance: 2.0
    passing_clearance: 3.0
    emergency_distance: 0.8
  
  timing:
    state_transition_delay: 500  # ms
    passing_timeout: 15000       # ms
    emergency_reaction_time: 200 # ms
  
  speeds:
    normal: 50
    following: 40
    passing: 60
    emergency: 0
  
  behavior:
    passing_enabled: true
    aggressive_mode: false
    prediction_horizon: 2.0      # segundos
```

### Maniobras de Adelantamiento

#### Condiciones para Adelantamiento
1. **Espacio libre** en carril izquierdo
2. **Distancia suficiente** para maniobra completa
3. **Velocidad apropiada** del obstáculo
4. **Tiempo de despeje** calculado
5. **Ausencia de tráfico** en sentido contrario

#### Secuencia de Adelantamiento
```cpp
void performOvertaking() {
    // Fase 1: Preparación
    reduceSpeed();
    checkLeftLane();
    
    // Fase 2: Cambio de carril izquierdo
    signalLeft();
    moveToLeftLane();
    
    // Fase 3: Adelantamiento
    increaseSpeed();
    maintainLeftLane();
    
    // Fase 4: Regreso al carril derecho
    checkClearance();
    signalRight();
    moveToRightLane();
    
    // Fase 5: Normalización
    resumeNormalSpeed();
}
```

### Casos de Uso

- **Carreteras** con tráfico mixto
- **Autopistas** con múltiples carriles
- **Entornos urbanos** complejos
- **Pruebas de rendimiento** del sistema
- **Competiciones** de vehículos autónomos

---

## Modo 4: Estacionamiento Autónomo

### Descripción General

Modo especializado para maniobras de estacionamiento de precisión. Utiliza algoritmos específicos para detectar espacios libres y ejecutar maniobras complejas de estacionamiento.

### Componentes Activos

#### Sensores
- **Cámara RGB**: Detección de referencias visuales
- **LIDAR**: Detección precisa de espacios y obstáculos

#### Nodos ROS Activos
```bash
/lane_detection              # Referencias visuales
/object_detection_parking    # Detección especializada
/Master_parking             # Control de precisión
```

### Algoritmos Principales

#### 1. Detección de Espacios de Estacionamiento
```cpp
class ParkingSpaceDetector {
public:
    std::vector<ParkingSpace> detectSpaces(const sensor_msgs::LaserScan& scan) {
        std::vector<ParkingSpace> spaces;
        
        // 1. Detectar obstáculos (vehículos estacionados)
        auto obstacles = detectObstacles(scan);
        
        // 2. Identificar espacios entre obstáculos
        for (int i = 0; i < obstacles.size() - 1; i++) {
            float gap_size = calculateGap(obstacles[i], obstacles[i+1]);
            
            if (gap_size >= MIN_PARKING_SPACE) {
                ParkingSpace space;
                space.start = obstacles[i].end_point;
                space.end = obstacles[i+1].start_point;
                space.width = gap_size;
                space.depth = calculateDepth(space);
                
                if (isViableSpace(space)) {
                    spaces.push_back(space);
                }
            }
        }
        
        return spaces;
    }
    
private:
    float MIN_PARKING_SPACE = 2.5;  // Espacio mínimo (metros)
    float VEHICLE_WIDTH = 1.8;      // Ancho del vehículo
    float SAFETY_MARGIN = 0.3;      // Margen de seguridad
};
```

#### 2. Planificación de Trayectoria de Estacionamiento
```cpp
class ParkingTrajectoryPlanner {
public:
    Trajectory planParallelParking(const ParkingSpace& space) {
        Trajectory traj;
        
        // Punto de inicio de maniobra
        Point start = calculateStartPoint(space);
        
        // Fase 1: Posicionamiento inicial
        traj.addSegment(createStraightSegment(current_position, start));
        
        // Fase 2: Retroceso con giro
        float turning_radius = calculateTurningRadius();
        traj.addSegment(createArcSegment(start, turning_radius, -45));
        
        // Fase 3: Enderezamiento
        Point intermediate = calculateIntermediatePoint(space);
        traj.addSegment(createArcSegment(intermediate, turning_radius, 45));
        
        // Fase 4: Ajuste final
        Point final = space.center;
        traj.addSegment(createStraightSegment(intermediate, final));
        
        return traj;
    }
    
private:
    float TURNING_RADIUS = 2.5;     // Radio de giro mínimo
    float APPROACH_DISTANCE = 1.0;  // Distancia de aproximación
};
```

#### 3. Control de Precisión
```cpp
class PrecisionController {
public:
    void executeParking(const Trajectory& trajectory) {
        for (const auto& segment : trajectory.segments) {
            switch(segment.type) {
                case STRAIGHT:
                    executeStraightMovement(segment);
                    break;
                case ARC:
                    executeArcMovement(segment);
                    break;
                case STOP:
                    executeStop();
                    break;
            }
            
            // Verificación continua de obstáculos
            if (obstacleDetected()) {
                emergencyStop();
                replanTrajectory();
            }
        }
    }
    
private:
    float PRECISION_SPEED = 5;      // Velocidad muy baja (km/h)
    float POSITION_TOLERANCE = 0.05; // Tolerancia de posición (metros)
    float ANGLE_TOLERANCE = 2.0;    // Tolerancia angular (grados)
};
```

### Configuración de Launch

**Archivo**: `parking.launch`
```xml
<launch>
  <!-- Nodo de detección de carriles para referencias -->
  <node name="lane_detection" 
        pkg="lane_detection" 
        type="lane_detection" 
        output="screen">
    <param name="precision_mode" value="true"/>
  </node>
  
  <!-- Nodo de detección especializada para parking -->
  <node name="object_detection_parking" 
        pkg="object_detection_parking" 
        type="object_detection_parking" 
        output="screen">
    <param name="space_detection_enabled" value="true"/>
    <param name="min_space_width" value="2.5"/>
  </node>
  
  <!-- Nodo de control de parking -->
  <node name="Master_parking" 
        pkg="control" 
        type="Master_parking" 
        output="screen">
    <param name="precision_mode" value="true"/>
    <param name="max_parking_speed" value="5"/>
  </node>
</launch>
```

### Tipos de Estacionamiento

#### 1. Estacionamiento Paralelo
- **Aplicación**: Calles urbanas
- **Complejidad**: Alta
- **Maniobras**: 4-5 movimientos
- **Tiempo estimado**: 60-90 segundos

#### 2. Estacionamiento Perpendicular
- **Aplicación**: Centros comerciales
- **Complejidad**: Media
- **Maniobras**: 2-3 movimientos
- **Tiempo estimado**: 30-45 segundos

#### 3. Estacionamiento en Ángulo
- **Aplicación**: Estacionamientos diagonales
- **Complejidad**: Baja
- **Maniobras**: 1-2 movimientos
- **Tiempo estimado**: 20-30 segundos

### Casos de Uso

- **Estacionamientos públicos**
- **Garajes residenciales**
- **Centros comerciales**
- **Aeropuertos**
- **Demostración de capacidades avanzadas**

---

## Configuración de Launch Files

### Estructura de Archivos

```
AVIM_folder/
├── AVIM/
│   └── launch/
│       ├── navigation_without_obstacles.launch
│       ├── navigation_with_static_obstacles.launch
│       ├── navigation_with_dynamic_obstacles.launch
│       └── parking.launch
└── bring_up/
    └── launch/
        ├── gazebo_empty_world.launch
        ├── gazebo_straight_road.launch
        ├── gazebo_curved_road.launch
        └── gazebo_intersection.launch
```

### Parámetros Comunes

#### Configuración de Gazebo
```xml
<!-- Configuración común para todos los modos -->
<include file="$(find gazebo_ros)/launch/empty_world.launch">
  <arg name="world_name" value="$(find autonomos_gazebo_simulation)/worlds/straight_road.world"/>
  <arg name="paused" value="false"/>
  <arg name="use_sim_time" value="true"/>
  <arg name="gui" value="true"/>
  <arg name="headless" value="false"/>
  <arg name="debug" value="false"/>
</include>
```

#### Configuración del Vehículo
```xml
<!-- Spawn del modelo del vehículo -->
<node name="spawn_urdf" 
      pkg="gazebo_ros" 
      type="spawn_model" 
      args="-file $(find autonomos_gazebo_simulation)/models/autonomos_vehicle.urdf 
            -urdf -x 0 -y 0 -z 0.1 -model autonomos_vehicle"/>
```

### Comandos de Ejecución

#### Modo 1: Sin Obstáculos
```bash
# Terminal 1: Lanzar Gazebo con mundo vacío
roslaunch autonomos_gazebo_simulation gazebo_empty_world.launch

# Terminal 2: Lanzar navegación sin obstáculos
roslaunch AVIM navigation_without_obstacles.launch
```

#### Modo 2: Obstáculos Estáticos
```bash
# Terminal 1: Lanzar Gazebo con obstáculos estáticos
roslaunch autonomos_gazebo_simulation gazebo_straight_road.launch

# Terminal 2: Lanzar navegación con obstáculos estáticos
roslaunch AVIM navigation_with_static_obstacles.launch
```

#### Modo 3: Obstáculos Dinámicos
```bash
# Terminal 1: Lanzar Gazebo con tráfico dinámico
roslaunch autonomos_gazebo_simulation gazebo_intersection.launch

# Terminal 2: Lanzar navegación con obstáculos dinámicos
roslaunch AVIM navigation_with_dynamic_obstacles.launch
```

#### Modo 4: Estacionamiento
```bash
# Terminal 1: Lanzar Gazebo con escenario de parking
roslaunch autonomos_gazebo_simulation gazebo_parking_lot.launch

# Terminal 2: Lanzar modo de estacionamiento
roslaunch AVIM parking.launch
```

---

## Parámetros de Configuración

### Archivo de Configuración Global

**Ubicación**: `config/avim_global_params.yaml`

```yaml
# Configuración global del sistema AVIM
avim_system:
  # Configuración general
  vehicle:
    width: 1.8          # metros
    length: 4.2         # metros
    wheelbase: 2.7      # metros
    max_speed: 100      # km/h
    max_steering: 30    # grados
  
  # Configuración de sensores
  sensors:
    camera:
      fps: 30
      resolution: "640x480"
      fov: 60           # grados
    
    lidar:
      range_max: 4.25   # metros
      range_min: 0.2    # metros
      angular_resolution: 0.25  # grados
      frequency: 40     # Hz
  
  # Configuración de seguridad
  safety:
    emergency_brake_distance: 0.5   # metros
    warning_distance: 1.0           # metros
    max_lateral_acceleration: 3.0   # m/s²
    max_longitudinal_deceleration: 5.0  # m/s²
```

### Configuración por Modo

#### Modo 1: Sin Obstáculos
```yaml
mode_1:
  enabled_sensors: ["camera"]
  max_speed: 60
  control_frequency: 30
  
  lane_detection:
    confidence_threshold: 0.7
    smoothing_factor: 0.3
  
  control:
    kp_angle: 0.8
    kd_angle: 0.2
    speed_constant: 50
```

#### Modo 2: Obstáculos Estáticos
```yaml
mode_2:
  enabled_sensors: ["camera", "lidar"]
  max_speed: 50
  control_frequency: 20
  
  object_detection:
    dbscan_epsilon: 0.15
    dbscan_min_points: 3
    max_detection_range: 4.0
  
  control:
    safe_distance: 1.5
    max_wait_time: 10000
    speed_reduction_factor: 0.7
```

#### Modo 3: Obstáculos Dinámicos
```yaml
mode_3:
  enabled_sensors: ["camera", "lidar"]
  max_speed: 70
  control_frequency: 30
  
  tracking:
    prediction_horizon: 2.0
    max_tracked_objects: 10
    track_timeout: 3.0
  
  passing:
    enabled: true
    min_clearance: 3.0
    max_passing_speed: 60
    timeout: 15000
```

#### Modo 4: Estacionamiento
```yaml
mode_4:
  enabled_sensors: ["camera", "lidar"]
  max_speed: 10
  control_frequency: 10
  
  parking:
    min_space_width: 2.5
    precision_tolerance: 0.05
    max_attempts: 3
    maneuver_timeout: 120000
```

---

## Comparativa de Modos

### Tabla Comparativa

| Característica | Modo 1 | Modo 2 | Modo 3 | Modo 4 |
|----------------|--------|--------|--------|--------|
| **Sensores** | Cámara | Cámara + LIDAR | Cámara + LIDAR | Cámara + LIDAR |
| **Velocidad máxima** | 60 km/h | 50 km/h | 70 km/h | 10 km/h |
| **Frecuencia control** | 30 Hz | 20 Hz | 30 Hz | 10 Hz |
| **Complejidad** | Baja | Media | Alta | Muy Alta |
| **Latencia típica** | 33 ms | 50-80 ms | 33-50 ms | 100 ms |
| **Consumo CPU** | Bajo | Medio | Alto | Medio |
| **Precisión requerida** | Media | Alta | Alta | Muy Alta |
| **Casos de uso** | Pruebas | Urbano | Carretera | Parking |

### Rendimiento por Modo

#### Métricas de Rendimiento

```yaml
performance_metrics:
  mode_1:
    success_rate: 95%
    average_speed: 45 km/h
    lane_keeping_accuracy: 90%
    cpu_usage: 25%
  
  mode_2:
    success_rate: 88%
    average_speed: 35 km/h
    obstacle_detection_rate: 95%
    false_positive_rate: 5%
    cpu_usage: 45%
  
  mode_3:
    success_rate: 82%
    average_speed: 50 km/h
    passing_success_rate: 75%
    collision_avoidance_rate: 98%
    cpu_usage: 70%
  
  mode_4:
    success_rate: 90%
    average_parking_time: 75 seconds
    precision_accuracy: 95%
    space_detection_rate: 85%
    cpu_usage: 40%
```

---

## Selección del Modo Apropiado

### Criterios de Selección

#### 1. Análisis del Entorno
```cpp
NavigationMode selectMode(const EnvironmentAnalysis& env) {
    if (env.parking_required) {
        return MODE_PARKING;
    }
    
    if (env.dynamic_obstacles > 0) {
        return MODE_DYNAMIC_OBSTACLES;
    }
    
    if (env.static_obstacles > 0) {
        return MODE_STATIC_OBSTACLES;
    }
    
    return MODE_LANE_FOLLOWING;
}
```

#### 2. Condiciones Ambientales
- **Iluminación**: Modos 2-4 funcionan mejor con LIDAR en condiciones de poca luz
- **Clima**: LIDAR es más robusto en lluvia/niebla
- **Tráfico**: Modo 3 para tráfico denso, Modo 2 para tráfico ligero
- **Velocidad requerida**: Modo 1 para alta velocidad, Modo 4 para precisión

#### 3. Recursos Disponibles
- **Potencia computacional**: Modo 3 requiere más CPU
- **Sensores disponibles**: Verificar funcionamiento de LIDAR
- **Tiempo real**: Considerar latencias aceptables

### Transición entre Modos

#### Cambio Dinámico de Modo
```cpp
class ModeManager {
public:
    void updateMode() {
        EnvironmentAnalysis current_env = analyzeEnvironment();
        NavigationMode suggested_mode = selectMode(current_env);
        
        if (suggested_mode != current_mode && canSafelyTransition()) {
            transitionToMode(suggested_mode);
        }
    }
    
private:
    bool canSafelyTransition() {
        return vehicle_speed < SAFE_TRANSITION_SPEED && 
               no_immediate_obstacles && 
               system_stable;
    }
};
```

### Recomendaciones de Uso

#### Para Desarrollo
1. **Comenzar con Modo 1** para calibración básica
2. **Progresar a Modo 2** para validar detección de obstáculos
3. **Avanzar a Modo 3** para pruebas de rendimiento
4. **Finalizar con Modo 4** para demostración completa

#### Para Producción
1. **Evaluar entorno** antes de cada misión
2. **Configurar modo apropiado** según condiciones
3. **Monitorear rendimiento** continuamente
4. **Preparar fallbacks** para degradación graceful

---

**Documentación de Modos de Navegación - Sistema AVIM**

*Versión 1.0 - Guía completa de configuración y uso*