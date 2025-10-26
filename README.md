# 🏎️ Control Reactivo "Follow the Gap" para F1Tenth

Un proyecto de **ROS 2** que implementa un controlador autónomo **Follow the Gap** para el simulador **F1Tenth**, probado en la pista **Brands Hatch**.

---

## 🎯 Objetivo del Proyecto

Este proyecto implementa un **controlador reactivo "Follow the Gap"** en un nodo de **ROS 2** para el simulador oficial de **F1Tenth**.  
El objetivo es recorrer la pista *Brands Hatch* y completar **10 vueltas** en el menor tiempo posible **sin colisiones**.

El proyecto incluye dos nodos principales:

- **`follow_gap_f1tenth`** → Nodo de control autónomo.  
- **`lap_timer`** → Cronómetro y contador de vueltas.

---

## 💡 Enfoque Utilizado: Follow the Gap

El enfoque **Follow the Gap** es una técnica reactiva de evasión de obstáculos basada en los datos del **LiDAR**.  
Su funcionamiento en este proyecto es el siguiente:

1. 🛰️ **Detección del entorno:** El LIDAR identifica paredes y obstáculos en tiempo real.  
2. 🗺️ **Mapeo de espacios (Gaps):** Analiza los datos del LIDAR para encontrar huecos seguros por donde pasar.  
3. 🏆 **Selección del mejor “Gap”:** Elige el espacio más amplio y seguro (el gap con la mayor distancia).  
4. 🧭 **Ajuste de trayectoria:** Dirige el vehículo hacia el centro del gap más lejano.  
5. 🏎️ **Control de velocidad:** Ajusta la velocidad en función de la distancia al objetivo, respetando un mínimo y un máximo.

---

## 🎥 Demostración

> 📝 **Nota:** Crea una carpeta llamada `img/` en la raíz de tu repositorio y añade un GIF con el nombre `demo.gif`.

![Demostración del proyecto](img/demo.gif)

---

## 📂 Contenido del Repositorio

```
F1Tenth-Repository/
├── src/
│   ├── follow_the_gap/
│   │   ├── follow_gap_f1tenth/     # Nodo de control reactivo
│   │   ├── lap_timer/              # Nodo de cronómetro y contador de vueltas
│   │   └── ...
│   └── f1tenth_gym_ros/            # Paquete del simulador
│       ├── maps/
│       │   ├── BrandsHatch_map
│       │   └── BrandsHatch_map_obs
│       └── ...
└── ...
```

### 📦 Descripción de los paquetes

- **`follow_the_gap`**  
  Contiene los nodos desarrollados para el control autónomo y el cronómetro.  
  - `lap_timer`: Mide el tiempo e informa el número de vueltas (debe ejecutarse primero).  
  - `follow_gap_f1tenth`: Nodo principal que recibe datos de `/scan` (LIDAR) y publica en `/drive`.

- **`f1tenth_gym_ros`**  
  Contiene el simulador y los mapas del entorno.  
  - Mapa base: `BrandsHatch_map`  
  - Mapa con obstáculos: `BrandsHatch_map_obs`

---

## ⚙️ Estructura del Controlador (`follow_gap_f1tenth`)

### 🧩 Suscripción a `/scan`

```python
self.subscription = self.create_subscription(
    LaserScan,
    '/scan',
    self.lidar_callback,
    10
)
```

### 🚗 Publicación en `/drive`

```python
self.publisher = self.create_publisher(
    AckermannDriveStamped,
    '/drive',
    10
)
```

### ⚙️ Parámetros del Controlador

> 💡 Para `BrandsHatch_map_obs` se recomienda reducir `max_speed` a `5.0`.

```python
self.min_distance = 2.2
self.window_size = 5
self.max_steering_angle = 0.4189  # ≈ 24 grados
self.max_speed = 11.0
self.min_speed = 0.5
self.safe_distance = 10.0
```

---

## 🧠 Lógica Principal

### `lidar_callback(msg: LaserScan)`

Procesa los datos del LiDAR y publica el comando de control:

```python
def lidar_callback(self, msg: LaserScan):
    ranges = np.array(msg.ranges)

    # Reemplazar infinitos y recortar valores
    ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
    ranges = np.clip(ranges, 0, msg.range_max)

    # Suavizado de datos
    smoothed = np.convolve(ranges, np.ones(self.window_size)/self.window_size, mode='same')

    # Eliminar obstáculos cercanos (Safety Bubble)
    cleaned = np.copy(smoothed)
    cleaned[cleaned < self.min_distance] = 0.0

    # Buscar el mayor gap
    gap_start, gap_end = self.find_largest_gap(cleaned)

    if gap_end <= gap_start:
        self.get_logger().warn("No se detectó un gap válido.")
        return

    # Calcular ángulo y velocidad
    target_idx = (gap_start + gap_end) // 2
    target_angle = msg.angle_min + target_idx * msg.angle_increment
    steering_angle = np.clip(target_angle, -self.max_steering_angle, self.max_steering_angle)

    target_distance = cleaned[target_idx]
    distance_ratio = np.clip(target_distance / self.safe_distance, 0.0, 1.0)
    speed = self.min_speed + (self.max_speed - self.min_speed) * distance_ratio
    speed = np.clip(speed, self.min_speed, self.max_speed)

    # Publicar comando
    self.publish_drive_command(speed, steering_angle)
```

---

### `find_largest_gap(data)`

Encuentra el inicio y el fin del gap más amplio:

```python
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

---

### `publish_drive_command(speed, steering_angle)`

Publica el mensaje de control:

```python
def publish_drive_command(self, speed, steering_angle):
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.speed = speed
    drive_msg.drive.steering_angle = steering_angle
    self.publisher.publish(drive_msg)
```

---

## 🚀 Instalación y Ejecución

### 1. Prerrequisitos

- **ROS 2 Humble** → [Instrucciones oficiales de instalación](https://docs.ros.org/en/humble/Installation.html)  
- **Simulador F1Tenth** → Sige las instrucciones [aquí](https://github.com/widegonz/F1Tenth-Repository/tree/main) para instalarlo.

---

### 2. Configuración del Mapa

Modifica el archivo `sim.yaml` en:
```
F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
```

Y actualiza la ruta del mapa:

```yaml
map_path: /home/your_user/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch_map
```

---

### 3. Ejecución de la Simulación

> ⚠️ **Importante:** En cada terminal, haz `source` de tu workspace antes de ejecutar cualquier comando.

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Luego, abre **tres terminales** y ejecuta:

#### 🧱 Terminal 1: Lanzar el Simulador
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

#### ⏱️ Terminal 2: Ejecutar el Cronómetro
```bash
ros2 run follow_the_gap lap_timer
```

#### 🤖 Terminal 3: Ejecutar el Controlador
```bash
ros2 run follow_the_gap follow_gap_f1tenth
```

Una vez ejecutado el último comando, el vehículo **comenzará a moverse autónomamente por la pista**.
