# Control Reactivo Follow the Gap para simulador F1Tenth probado en la pista Brands Hatch

Proyecto práctico basado en el simulador oficial de F1Tenth en el que se implementa un controlador reactivo "Follow the Gap", en forma de un nodo de ROS 2 para recorrer la pista **Brands Hatch** con el objetivo de completar 10 vueltas en el menor tiempo posible **sin colisiones**. Para este fin, también se implementará un nodo con función de cronómetro y contador de vueltas.

---

### **Descripción del enfoque utilizado**

El enfoque "Follow the Gap" es una técnica utilizada en la conducción autónoma para evitar obstáculos de manera reactiva. Su funcionamiento en este proyecto es el siguiente:

- **Detección del entorno**: El vehículo utiliza un LIDAR para identificar las paredes y obstáculos en su trayectoria en tiempo real.  
- **Mapeo de espacios libres ("gaps")**: Analiza la información del LIDAR para reconocer los huecos o espacios entre obstáculos donde el vehículo podría pasar de manera segura.  
- **Selección del mejor "gap"**: Elige el espacio más amplio y seguro; en este caso, es el *gap* que posee la mayor distancia hasta el vehículo.  
- **Ajuste de trayectoria**: Dirige el vehículo hacia el centro del *gap* más lejano.  
- **Control de velocidad**: El vehículo se mueve a una velocidad en función de la distancia al *gap*, hasta una velocidad máxima. Además, posee un parámetro de velocidad mínima.

---

### **Contenido del repositorio**

El repositorio posee la carpeta `src`, en la cual se encuentra el paquete `follow_the_gap`, que contiene los dos nodos utilizados en este proyecto.

- El nodo **`lap_timer`** es el encargado de comenzar a medir el tiempo de cada vuelta desde que el vehículo empieza a moverse (por eso es importante ejecutarlo primero). Cuando el vehículo vuelve a pasar por la línea de inicio, este nodo publica el número de vuelta y el tiempo de vuelta.

- El nodo **`follow_gap_f1tenth`** es el nodo de control autónomo que recibe datos del LIDAR (mediante el *topic* `/scan`) del vehículo para decidir la dirección y velocidad en la que el vehículo debe avanzar, y publica esta información en el *topic* `/drive`.

Adicionalmente el repositorio tambien tiene el paquete `f1tenth_gym_ros` el cual contiene la carpeta `maps` cargada con varios mapas para el simulador, el mapa empleado en este proyecto es el llamado `BrandsHatch_map`, tambien se emplea el mapa llamado `BrandsHatch_map_obs` el cual es la verción con obstaculos.

---

### **Estructura del código del controlador**

#### - Suscripción a `/scan`
```py
self.subscription = self.create_subscription(
    LaserScan,
    '/scan',
    self.lidar_callback,
    10
)
```

#### - Publicación en `/drive`
```py
self.publisher = self.create_publisher(
    AckermannDriveStamped,
    '/drive',
    10
)
```

#### - Parámetros del controlador  
*(Se recomienda bajar `max_speed` a 5 para pistas con obstáculos)*
```py
self.min_distance = 2.2
self.window_size = 5
self.max_steering_angle = 0.4189  # ~24 grados
self.max_speed = 11.0
self.min_speed = 0.5
self.safe_distance = 10.0
```

#### - Función para leer los datos de `/scan`, analizarlos y publicar resultados en `/drive`
```py
def lidar_callback(self, msg: LaserScan):
    ranges = np.array(msg.ranges)

    # Reemplazar infinitos por valor máximo
    ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
    ranges = np.clip(ranges, 0, msg.range_max)

    # Suavizado
    smoothed = np.convolve(ranges, np.ones(self.window_size)/self.window_size, mode='same')

    # Eliminar obstáculos cercanos
    cleaned = np.copy(smoothed)
    cleaned[cleaned < self.min_distance] = 0.0

    # Buscar el mayor gap
    gap_start, gap_end = self.find_largest_gap(cleaned)

    if gap_end <= gap_start:
        self.get_logger().warn("No se detectó un gap válido.")
        return

    # Índice del centro del gap
    target_idx = (gap_start + gap_end) // 2
    target_angle = msg.angle_min + target_idx * msg.angle_increment

    # Limitar dirección
    steering_angle = np.clip(target_angle, -self.max_steering_angle, self.max_steering_angle)

    # Velocidad basada en distancia hacia el objetivo
    target_distance = cleaned[target_idx]
    distance_ratio = np.clip(target_distance / self.safe_distance, 0.0, 1.0)

    speed = self.min_speed + (self.max_speed - self.min_speed) * distance_ratio
    speed = np.clip(speed, self.min_speed, self.max_speed)

    self.publish_drive_command(speed, steering_angle)
```

#### - Función para encontrar el *gap* más lejano
```py
def find_largest_gap(self, data):
    max_len = 0
    max_start = 0
    max_end = 0
    current_start = None

    for i in range(len(data)):
        if data[i] > 0:
            if current_start is None:
                current_start = i
        else:
            if current_start is not None:
                length = i - current_start
                if length > max_len:
                    max_len = length
                    max_start = current_start
                    max_end = i - 1
                current_start = None

    if current_start is not None:
        length = len(data) - current_start
        if length > max_len:
            max_start = current_start
            max_end = len(data) - 1

    return max_start, max_end
```

#### - Función para publicar en `/drive` (usada por `lidar_callback`)
```py
def publish_drive_command(self, speed, steering_angle):
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.speed = speed
    drive_msg.drive.steering_angle = steering_angle
    self.publisher.publish(drive_msg)
```

---

## Instrucciones de ejecución e instalación del simulador, ROS 2 y los nodos

### Instalación de ROS 2 Humble

- **ROS 2 Humble**: Siga las instrucciones [aquí](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) para instalarlo.

### Instalación del simulador F1Tenth

- **Simulador F1Tenth**: Siga las instrucciones [aquí](https://github.com/widegonz/F1Tenth-Repository/tree/main) para instalarlo.

### Cambio de mapa a Brands Hatch

Para cambiar el mapa del simulador a **Brands Hatch**, debemos modificar la ruta del mapa en el archivo `sim.yaml`, utilizando la siguiente ruta:

```bash
/home/your_user/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch
```

---

### Para iniciar la simulación

Asegúrese de haber hecho `source` a su espacio de trabajo:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**NOTA**: Hacer el `source` en cada nueva terminal que se abra.

---

### 1. Ejecutar el simulador

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

> La primera vez que se ejecuta, el simulador tardará un poco en cargar el modelo del robot. Las siguientes veces será más rápido.

---

### 2. Ejecutar el cronómetro y contador de vueltas

En una nueva terminal:

```bash
cd ~/F1Tenth-Repository
source install/setup.bash
ros2 run follow_the_gap lap_timer
```

---

### 3. Ejecutar el nodo de control

En otra terminal:

```bash
cd ~/F1Tenth-Repository
source install/setup.bash
ros2 run follow_the_gap follow_gap_f1tenth
```

Al hacer esto, el vehículo virtual comenzará automáticamente a recorrer la pista de manera autónoma, tratando de realizar 10 vueltas en el menor tiempo posible.
