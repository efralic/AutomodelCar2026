# Documentación Detallada de Paquetes ROS - Sistema AVIM

## Tabla de Contenidos
1. [Paquete Control](#paquete-control)
2. [Paquete Lane Detection](#paquete-lane-detection)
3. [Paquete Object Detection](#paquete-object-detection)
4. [Paquete Object Detection Parking](#paquete-object-detection-parking)
5. [Paquete AVIM (Launch Files)](#paquete-avim-launch-files)
6. [Configuración y Parámetros](#configuración-y-parámetros)

---

## Paquete Control

### Ubicación
`catkin_ws/src/AVIM_folder/control/`

### Descripción General
El paquete `control` es el cerebro del sistema AVIM. Contiene los nodos maestros que coordinan toda la navegación del vehículo, procesando información de sensores y generando comandos de control apropiados.

### Archivos Principales

#### 1. Master.cpp
**Propósito**: Controlador principal para navegación con obstáculos dinámicos

**Características principales**:
- **Gestión de tareas**: Sistema de cola de tareas con prioridades
- **Control PID**: Algoritmos de control proporcional-integral-derivativo
- **Evasión dinámica**: Maniobras de adelantamiento y evasión en tiempo real
- **Máquina de estados**: Estados de navegación (LANE_DRIVING, FOLLOWING, MOVING_LEFT, PASSING, etc.)

**Estados de navegación**:
```cpp
int LANE_DRIVING = 0;      // Conducción normal en carril
int FOLLOWING = 1;         // Siguiendo vehículo adelante
int MOVING_LEFT = 2;       // Moviéndose al carril izquierdo
int PASSING = 3;           // Adelantando obstáculo
int MOVING_RIGHT = 4;      // Regresando al carril derecho
int MOVING_RIGHT_LANE = 5; // Alineándose en carril derecho
```

**Tópicos ROS**:
- **Suscripciones**:
  - `/distance_center` (std_msgs::Int16): Distancia al centro del carril
  - `/objects_detected` (object_detection::points_objects): Obstáculos detectados
- **Publicaciones**:
  - `/cmd_vel` (geometry_msgs::Twist): Comandos de velocidad
  - `/steering` (std_msgs::Int16): Comandos de dirección

**Algoritmos de control**:
```cpp
// Control PID para ángulo
float error_angle = dist_now - center_cam;
float u_angle = kp_angle * error_angle + kd_angle * (error_angle - angle_last);

// Control de velocidad adaptativo
int speed = decrement_speed(obstacle_distance);
```

#### 2. Master_static.cpp
**Propósito**: Controlador especializado para obstáculos estáticos

**Diferencias con Master.cpp**:
- Algoritmos de planificación de rutas más conservadores
- No realiza maniobras de adelantamiento dinámico
- Optimizado para obstáculos que no se mueven
- Estrategias de espera y replanificación

#### 3. Master_parking.cpp
**Propósito**: Controlador para maniobras de estacionamiento

**Características especiales**:
- Control de precisión milimétrica
- Maniobras de estacionamiento paralelo y perpendicular
- Algoritmos de detección de espacios libres
- Control de velocidad ultra-lento para precisión

### Parámetros de Configuración

```cpp
// Parámetros PID
float kp_angle = 0.8;        // Ganancia proporcional ángulo
float kd_angle = 0.2;        // Ganancia derivativa ángulo
float kp_speed = 1.0;        // Ganancia proporcional velocidad

// Parámetros de navegación
float dist_to_keep = 1.5;    // Distancia mínima a obstáculos (metros)
int max_waiting_time = 5000; // Tiempo máximo de espera (ms)
bool passing_enabled = true; // Habilitar adelantamientos
int mid_speed = 50;          // Velocidad media
```

---

## Paquete Lane Detection

### Ubicación
`catkin_ws/src/AVIM_folder/lane_detection/`

### Descripción General
El paquete `lane_detection` se encarga del procesamiento de imágenes para detectar y seguir los carriles de la carretera. Utiliza técnicas avanzadas de visión por computadora con OpenCV.

### Archivo Principal: lane_detection.cpp

#### Funcionalidades Principales

**1. Procesamiento de Imágenes**
```cpp
void LineDetectionCb(const sensor_msgs::ImageConstPtr& msg) {
    // Conversión de ROS a OpenCV
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    // Pipeline de procesamiento
    copyImageRGB(cv_ptr->image, frame);
    rotate(frame, 180.0);  // Rotación si es necesario
    Perspective(frame, invertedPerspectiveMatrix);
    img_edges = Threshold(frame);
    locate_lanes(img_edges, img_lines);
}
```

**2. Transformación de Perspectiva (Bird's Eye View)**
```cpp
void Perspective(Mat &frame, Mat &invertedPerspectiveMatrix) {
    // Puntos de origen (vista frontal)
    Source[0] = Point2f(200, 300);   // Esquina superior izquierda
    Source[1] = Point2f(440, 300);   // Esquina superior derecha
    Source[2] = Point2f(0, 480);     // Esquina inferior izquierda
    Source[3] = Point2f(640, 480);   // Esquina inferior derecha
    
    // Puntos de destino (vista aérea)
    Destination[0] = Point2f(150, 0);
    Destination[1] = Point2f(490, 0);
    Destination[2] = Point2f(150, 480);
    Destination[3] = Point2f(490, 480);
    
    Mat perspectiveMatrix = getPerspectiveTransform(Source, Destination);
    warpPerspective(frame, img_perspective, perspectiveMatrix, frame.size());
}
```

**3. Detección de Líneas**
- **Umbralización**: Conversión a escala de grises y aplicación de umbrales
- **Histograma**: Análisis de distribución de píxeles para encontrar líneas
- **Regresión polinomial**: Ajuste de curvas a los puntos detectados
- **Filtrado temporal**: Suavizado de detecciones entre frames

**4. Algoritmo de Localización de Carriles**
```cpp
void locate_lanes(Mat &img, Mat &out_img) {
    // Análisis de histograma para encontrar picos
    int *histogram = Histogram(img);
    
    // Búsqueda de líneas izquierda y derecha
    int left_peak = find_left_peak(histogram);
    int right_peak = find_right_peak(histogram);
    
    // Seguimiento de líneas usando ventanas deslizantes
    sliding_window_search(img, left_peak, right_peak);
    
    // Regresión polinomial
    if (regression_left() && regression_right()) {
        draw_lines(out_img);
        calculate_center_distance();
    }
}
```

#### Tópicos ROS

**Suscripciones**:
- `/camera/image_raw` (sensor_msgs::Image): Imágenes de la cámara RGB

**Publicaciones**:
- `/distance_center` (std_msgs::Int16): Distancia al centro del carril en píxeles
- `/angle_line` (std_msgs::UInt8): Ángulo de corrección necesario

#### Parámetros de Configuración

```cpp
// Parámetros de cámara
int center_cam = 320;        // Centro de la imagen (píxeles)
float alpha_resize = 0.15;   // Factor de redimensionamiento

// Parámetros de detección
int threshold_low = 50;      // Umbral bajo para detección de bordes
int threshold_high = 150;    // Umbral alto para detección de bordes
int min_line_length = 50;    // Longitud mínima de línea
int max_line_gap = 10;       // Separación máxima entre segmentos
```

---

## Paquete Object Detection

### Ubicación
`catkin_ws/src/AVIM_folder/object_detection/`

### Descripción General
El paquete `object_detection` procesa datos del sensor LIDAR para detectar y clasificar obstáculos en el entorno del vehículo. Utiliza algoritmos de clustering para agrupar puntos y identificar objetos.

### Archivo Principal: object_detection.cpp

#### Algoritmo DBSCAN

El corazón del sistema de detección es el algoritmo DBSCAN (Density-Based Spatial Clustering of Applications with Noise):

```cpp
class DBSCAN {
public:
    std::vector<PointC> m_points;
    unsigned int m_minPoints;  // Mínimo de puntos para formar cluster
    float m_epsilon;           // Radio de búsqueda
    
    int run() {
        int clusterID = 1;
        for (auto& point : m_points) {
            if (point.clusterID == UNCLASSIFIED) {
                if (expandCluster(point, clusterID) != FAILURE) {
                    clusterID += 1;
                }
            }
        }
        return SUCCESS;
    }
};
```

#### Procesamiento de Datos LIDAR

**1. Conversión de Coordenadas**
```cpp
void laser_msg_Callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    scan_points.clear();
    
    for (int i = 0; i < scan->ranges.size(); i++) {
        float range = scan->ranges[i];
        
        // Filtrar por rango válido
        if (range >= RANGE_MIN && range <= RANGE) {
            float angle = scan->angle_min + i * scan->angle_increment;
            
            // Conversión polar a cartesiana
            float x = range * cos(angle);
            float y = range * sin(angle);
            
            scan_points.push_back(cv::Point2f(x, y));
        }
    }
}
```

**2. Clustering y Filtrado**
```cpp
void get_object_points(const vector_point& scan_points) {
    // Conversión a formato DBSCAN
    points_.clear();
    for (const auto& point : scan_points) {
        points_.push_back(cv::Point(point.x * 100, point.y * 100)); // Escala
    }
    
    // Aplicar DBSCAN
    DBSCAN dbscan(MINIMUM_POINTS, EPSILON, points_);
    dbscan.run();
    dbscan.getCluster(clusters_points);
    
    // Calcular centroides
    get_centroids_objects(clusters_points);
}
```

**3. Cálculo de Centroides**
```cpp
void get_centroids_objects(const std::map<int,std::vector<cv::Point>> &clusteredPoints_) {
    points_centroids.clear();
    
    for (const auto& cluster : clusteredPoints_) {
        if (cluster.second.size() >= MINIMUM_POINTS) {
            cv::Point centroid(0, 0);
            
            // Calcular centro de masa
            for (const auto& point : cluster.second) {
                centroid.x += point.x;
                centroid.y += point.y;
            }
            
            centroid.x /= cluster.second.size();
            centroid.y /= cluster.second.size();
            
            points_centroids.push_back(centroid);
        }
    }
}
```

#### Tópicos ROS

**Suscripciones**:
- `/scan` (sensor_msgs::LaserScan): Datos del sensor LIDAR

**Publicaciones**:
- `/objects_detected` (object_detection::points_objects): Lista de obstáculos detectados

#### Parámetros de Configuración

```cpp
// Parámetros LIDAR
float RANGE = 4.25;              // Rango máximo de detección (metros)
float RANGE_MIN = 0.2;           // Rango mínimo de detección (metros)
float INITIAL_RANGE_ANGLE = 0.0; // Ángulo inicial (grados)
float END_RANGE_ANGLE = 360.0;   // Ángulo final (grados)

// Parámetros DBSCAN
int MINIMUM_POINTS = 3;          // Mínimo puntos para cluster
float EPSILON = 15.0;            // Radio de búsqueda (cm)

// Parámetros de filtrado
float MIN_OBJECT_SIZE = 0.1;     // Tamaño mínimo de objeto (metros)
float MAX_OBJECT_SIZE = 3.0;     // Tamaño máximo de objeto (metros)
```

---

## Paquete Object Detection Parking

### Ubicación
`catkin_ws/src/AVIM_folder/object_detection_parking/`

### Descripción General
Variante especializada del paquete de detección de objetos, optimizada para maniobras de estacionamiento. Incluye algoritmos específicos para detectar espacios libres y obstáculos en entornos de estacionamiento.

### Características Especiales

#### 1. Detección de Espacios Libres
- Algoritmos para identificar espacios de estacionamiento disponibles
- Medición precisa de dimensiones de espacios
- Evaluación de viabilidad para maniobras de estacionamiento

#### 2. Detección de Obstáculos de Precisión
- Mayor resolución en la detección de objetos cercanos
- Filtrado especializado para entornos de estacionamiento
- Detección de bordillos, postes y otros vehículos

#### 3. Algoritmos de Maniobra
- Cálculo de trayectorias de estacionamiento
- Puntos de referencia para maniobras precisas
- Validación de espacios suficientes

#### Tópicos ROS

**Suscripciones**:
- `/scan` (sensor_msgs::LaserScan): Datos LIDAR de alta precisión

**Publicaciones**:
- `/parking_objects` (object_detection::points_objects): Obstáculos y espacios para parking
- `/parking_spaces` (geometry_msgs::PolygonStamped): Espacios libres detectados

---

## Paquete AVIM (Launch Files)

### Ubicación
`catkin_ws/src/AVIM_folder/AVIM/`

### Descripción General
Contiene los archivos de lanzamiento (launch files) que coordinan la ejecución de diferentes configuraciones del sistema según el modo de operación.

### Launch Files Disponibles

#### 1. navigation_without_obstacles.launch
```xml
<launch>
  <node name="lane_detection" pkg="lane_detection" type="lane_detection" output="screen"/>
  <node name="Master" pkg="control" type="Master" output="screen"/>
</launch>
```
**Uso**: Navegación básica siguiendo carriles sin obstáculos

#### 2. navigation_with_static_obstacles.launch
```xml
<launch>
  <node name="lane_detection" pkg="lane_detection" type="lane_detection" output="screen"/>
  <node name="object_detection" pkg="object_detection" type="object_detection" output="screen"/>
  <node name="Master_static" pkg="control" type="Master_static" output="screen"/>
</launch>
```
**Uso**: Navegación con evasión de obstáculos estáticos

#### 3. navigation_with_dynamic_obstacles.launch
```xml
<launch>
  <node name="lane_detection" pkg="lane_detection" type="lane_detection" output="screen"/>
  <node name="object_detection" pkg="object_detection" type="object_detection" output="screen"/>
  <node name="Master" pkg="control" type="Master" output="screen"/>
</launch>
```
**Uso**: Navegación con evasión de obstáculos dinámicos

#### 4. parking.launch
```xml
<launch>
  <node name="lane_detection" pkg="lane_detection" type="lane_detection" output="screen"/>
  <node name="object_detection_parking" pkg="object_detection_parking" type="object_detection_parking" output="screen"/>
  <node name="Master_parking" pkg="control" type="Master_parking" output="screen"/>
</launch>
```
**Uso**: Maniobras de estacionamiento autónomo

---

## Configuración y Parámetros

### Archivos de Configuración

Cada paquete puede incluir archivos de parámetros YAML para configuración avanzada:

#### config/lane_detection_params.yaml
```yaml
lane_detection:
  camera:
    center_x: 320
    center_y: 240
    focal_length: 500
  
  processing:
    resize_factor: 0.15
    threshold_low: 50
    threshold_high: 150
  
  perspective:
    source_points: [[200, 300], [440, 300], [0, 480], [640, 480]]
    dest_points: [[150, 0], [490, 0], [150, 480], [490, 480]]
```

#### config/object_detection_params.yaml
```yaml
object_detection:
  lidar:
    range_min: 0.2
    range_max: 4.25
    angle_min: 0.0
    angle_max: 360.0
  
  dbscan:
    min_points: 3
    epsilon: 0.15
  
  filtering:
    min_object_size: 0.1
    max_object_size: 3.0
```

#### config/control_params.yaml
```yaml
control:
  pid:
    kp_angle: 0.8
    kd_angle: 0.2
    kp_speed: 1.0
  
  navigation:
    dist_to_keep: 1.5
    max_waiting_time: 5000
    passing_enabled: true
    mid_speed: 50
  
  safety:
    emergency_brake_distance: 0.5
    max_steering_angle: 30
    max_speed: 100
```

### Calibración de Sensores

#### Calibración de Cámara
1. **Parámetros intrínsecos**: Matriz de calibración, distorsión
2. **Parámetros extrínsecos**: Posición y orientación relativa
3. **Transformación de perspectiva**: Puntos de referencia para bird's eye view

#### Calibración de LIDAR
1. **Offset de posición**: Corrección de posición relativa al centro del vehículo
2. **Filtros de ruido**: Eliminación de lecturas erróneas
3. **Sincronización temporal**: Alineación con otros sensores

### Monitoreo y Debugging

#### Herramientas de Diagnóstico

**1. Visualización en RViz**
```bash
# Configuración para visualización
rosrun rviz rviz -d config/avim_visualization.rviz
```

**2. Monitoreo de Tópicos**
```bash
# Monitorear frecuencia de publicación
rostopic hz /distance_center
rostopic hz /objects_detected

# Visualizar datos en tiempo real
rostopic echo /distance_center
rostopic echo /objects_detected
```

**3. Análisis de Rendimiento**
```bash
# Información de nodos
rosnode info /lane_detection
rosnode info /object_detection
rosnode info /Master

# Gráfico de conexiones
rqt_graph
```

#### Logs y Debugging

**Niveles de logging**:
- `DEBUG`: Información detallada para desarrollo
- `INFO`: Información general de operación
- `WARN`: Advertencias que no afectan funcionamiento
- `ERROR`: Errores que requieren atención
- `FATAL`: Errores críticos que detienen el sistema

**Configuración de logging**:
```cpp
// En código C++
ROS_DEBUG("Procesando frame %d", frame_count);
ROS_INFO("Obstáculo detectado a %.2f metros", distance);
ROS_WARN("Velocidad reducida por proximidad a obstáculo");
ROS_ERROR("Error en calibración de cámara");
```

---

## Extensiones y Mejoras Futuras

### Mejoras Planificadas

1. **Fusión de Sensores Avanzada**
   - Integración de múltiples cámaras
   - Fusión LIDAR + cámara con filtros de Kalman
   - Sensores de ultrasonido para detección cercana

2. **Algoritmos de IA**
   - Redes neuronales para detección de objetos
   - Aprendizaje por refuerzo para optimización de rutas
   - Predicción de comportamiento de obstáculos dinámicos

3. **Mejoras de Rendimiento**
   - Optimización de algoritmos para tiempo real
   - Paralelización de procesamiento
   - Implementación en GPU para visión por computadora

4. **Funcionalidades Adicionales**
   - Reconocimiento de señales de tráfico
   - Navegación GPS integrada
   - Comunicación V2V (Vehicle-to-Vehicle)
   - Modo de conducción nocturna

### Arquitectura Modular

El diseño modular del sistema permite:
- **Fácil extensión**: Agregar nuevos sensores o algoritmos
- **Mantenimiento simplificado**: Cada paquete es independiente
- **Reutilización**: Componentes pueden usarse en otros proyectos
- **Testing individual**: Cada módulo puede probarse por separado

---

**Documentación de Paquetes ROS - Sistema AVIM**

*Versión 1.0 - Documentación técnica detallada*