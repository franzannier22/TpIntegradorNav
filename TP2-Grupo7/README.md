# TP Integrador ROS 2 — Grupo 7

Robot diferencial con SLAM (`slam_toolbox`) y localización (`nav2_amcl`) sobre
Nav2, simulado en Gazebo (gz-sim / Harmonic).

## Compilar (tras cualquier cambio)

```bash
cd ~/Desktop/TpIntegrador/TP2-Grupo7
colcon build
source install/setup.bash
```

Sourceá `install/setup.bash` en **cada terminal nueva** que abras (incluida la
de RViz) — si no, RViz no encuentra las mallas del robot y tira "Errors
loading geometries".

---

## Mundos disponibles

| Mundo              | Descripción                                              |
|---------------------|-----------------------------------------------------------|
| `living_room.world` | Casa con muebles (sofá, mesa, sillón, etc.), ~5x5 m       |
| `depot.world`       | Depósito grande con pilares, cajas y estanterías, ~30x15 m |

Los mundos viejos (`maze*.world`, `cafe.world`) quedan en `worlds/` como
referencia pero no se usan en el TP.

---

## Parte 1 — SLAM (mapeo)

### Terminal 1 — Gazebo + slam_toolbox

```bash
source install/setup.bash
ros2 launch my_robot_navigation slam_launch.py world:=living_room.world x:=1.8 y:=-1.9 yaw:=0.0
# o para el deposito:
ros2 launch my_robot_navigation slam_launch.py world:=depot.world x:=-5.0 y:=0.0 yaw:=0.0
```

Este launch levanta Gazebo + el robot + el bridge + `slam_toolbox`
(modo `mapping`, online async) gestionado por `nav2_lifecycle_manager`, todo
en un único archivo.

**Detalle importante:** durante el mapeo, el nodo `ground_truth_odom_tf`
reemplaza la TF `odom→base_footprint` del `DiffDrive` (que patina con los
choques) por la pose real de Gazebo (`/odom_real`). Esto se activa automático
en este launch (`publish_diffdrive_tf:=false` hacia adentro) y **no afecta**
la Parte 2, que sí usa la odometría real/imperfecta.

### Terminal 2 — RViz

```bash
source install/setup.bash
rviz2 -d src/my_robot_description/rviz/urdf_config.rviz
```

Si el display "Map" no aparece: cambiá **Fixed Frame** a `map` y el topic del
display Map a `/map` (después `File → Save Config` para que quede guardado).

### Terminal 3 — Joystick (recorrer el mundo)

```bash
ros2 run joy joy_node
```
```bash
ros2 run teleop_twist_joy teleop_node --ros-args --params-file /home/francisco/Desktop/fundacion/tpi_m1/src/paquete_py/config/teleop_twist_joy.yaml
```

Mantené apretado el **botón 9** del control para que se envíen los comandos
de velocidad. Manejá despacio y evitá chocar contra las paredes — un choque
hace patinar la odometría y puede generar loop closures falsos, sobre todo en
mundos con geometría repetitiva.

### Guardar el mapa

Con Gazebo + slam_toolbox todavía corriendo (Terminal 1), en una terminal
nueva:

```bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f src/my_robot_navigation/maps/living_room
# o
ros2 run nav2_map_server map_saver_cli -f src/my_robot_navigation/maps/depot
```

Genera `<nombre>.pgm` + `<nombre>.yaml` en `src/my_robot_navigation/maps/`.

---

## Parte 2 — Localización (AMCL)

### Terminal 1 — Gazebo + map_server + AMCL + RViz

```bash
source install/setup.bash
ros2 launch my_robot_navigation localization.launch.py \
  world:=living_room.world map:=src/my_robot_navigation/maps/living_room.yaml \
  x:=1.8 y:=-1.9 yaw:=0.0
```

`x` e `y` son obligatorios (sin default) — tienen que coincidir con una
posición válida dentro del mundo elegido. `map` por default apunta a
`living_room.yaml`; para el depósito pasá `map:=.../depot.yaml`.

Este launch usa la odometría **real** del `DiffDrive` (no la de
`/odom_real`), para que AMCL tenga sentido corrigiéndola contra el mapa.

En RViz, usá la herramienta **"2D Pose Estimate"** para darle a AMCL una pose
inicial aproximada, y confirmá que la nube de partículas converge alrededor
del robot mientras se mueve.

---

## Parte 3 — Costmap, Planner y Controller

### Terminal 1 — Gazebo + map_server + AMCL + planner + controller + RViz

```bash
source install/setup.bash
ros2 launch my_robot_navigation nav_launch.py
# o para el deposito:
ros2 launch my_robot_navigation nav_launch.py \
  world:=depot.world map:=src/my_robot_navigation/maps/depot.yaml x:=-5.0 y:=0.0 yaw:=0.0
```

Este launch (todo en un único archivo) levanta:

- **Gazebo** + el robot + el bridge.
- **`map_server`** — publica el mapa guardado (capa estática del costmap).
- **`nav2_amcl`** — localización. Recibe una **pose inicial automática** desde
  los argumentos `x`/`y`/`yaw`, así que **no hace falta el "2D Pose Estimate"**;
  la TF `map→odom` existe desde el arranque y los costmaps se activan solos.
- **`planner_server`** (NavFn, A\*) con su **global costmap**.
- **`controller_server`** con **dos algoritmos** (DWB y RPP) y sus **global +
  local costmaps**.
- **`nav2_lifecycle_manager`** que gestiona los 4 nodos de ciclo de vida.
- **RViz**.

Todos los parámetros del costmap, planner y controller están en
`config/nav2_params.yaml`.

Los defaults son para `living_room`. Para otro mundo pasá `world`, `map`, `x` e
`y` (la pose inicial de AMCL se toma de `x`/`y`/`yaw`, así que tienen que
coincidir con el spawn).

### Verificar que todo quedó activo

En una terminal nueva y sourceada:

```bash
ros2 lifecycle get /planner_server      # debe decir: active [3]
ros2 lifecycle get /controller_server   # debe decir: active [3]
```

### Ver los costmaps en RViz

Fixed Frame → `map`, y agregá estos displays (botón **Add → By topic**):

| Topic                        | Tipo   | Qué muestra                         |
|------------------------------|--------|-------------------------------------|
| `/global_costmap/costmap`    | `Map`  | Costmap global (static + inflación) |
| `/local_costmap/costmap`     | `Map`  | Costmap local (ventana móvil)       |
| `/plan`                      | `Path` | Ruta calculada por el planner       |

Deberías ver la **capa de inflación** (aureola de color) alrededor de las
paredes y muebles. Si un display queda tapado por otro, bajale el **Alpha** o
reordenalos en la lista. Guardá con `File → Save Config` para no repetirlo.

**Probar la capa de obstáculos (LIDAR):** agregá un objeto nuevo en Gazebo
(cerca del robot) y observá el **local costmap** en RViz: el obstáculo aparece
marcado en tiempo real y, al retirarlo, se limpia. Esto confirma que la
`obstacle_layer` está consumiendo `/scan` correctamente.

### Probar el planner (A\* / Dijkstra)

Con la simulación corriendo, en otra terminal sourceada:

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.5}, orientation: {w: 1.0}}}, planner_id: GridBased, use_start: false}"
```

Devuelve el path y lo dibuja en el display `/plan`. Para cambiar entre A\* y
Dijkstra, editá `use_astar` en `config/nav2_params.yaml` (`true` = A\*,
`false` = Dijkstra).

> **Ojo con la meta:** tiene que caer **dentro de los límites del mapa** y en
> zona libre. Si el `Result` da `error_code: 204` la meta está fuera del mapa;
> `206` es que cayó sobre un obstáculo. Los rangos válidos surgen del `.yaml`
> del mapa (`origin` + `resolution × tamaño_px`). Para `living_room`:
> X ≈ [-2.46, 2.69], Y ≈ [-2.44, 2.42]. Lo más cómodo es usar el botón
> **"2D Goal Pose"** de RViz y clickear un punto libre.

### Seleccionar el algoritmo del controller

Los dos controllers se cargan a la vez. Se elige cuál usar con el campo
`controller_id` al enviar la meta a la acción `FollowPath`:

- `FollowPath` → **DWB**
- `FollowPathRPP` → **Regulated Pure Pursuit (RPP)**

El envío de la trayectoria al controller lo hará el **nodo orquestador (Parte
4)**; ahí se indica el `controller_id` elegido.

---

## Parte 4 — Orquestador

Con `nav_launch.py` (Parte 3) corriendo, en otra terminal:

```bash
source install/setup.bash
ros2 run my_robot_navigation goal_orchestrator
```

En RViz, usá **"2D Goal Pose"** para marcarle una meta al robot. El nodo:
consulta su pose actual vía TF (`map → base_link`), le pide un camino al
`planner_server` (`ComputePathToPose`), se lo manda al `controller_server`
(`FollowPath`), y replanifica cada `replan_period` segundos (default 3.0)
mientras navega. Loguea éxito o en qué etapa falló (localización/TF,
planificación o control). Si llega una meta nueva mientras navega, cancela
la anterior y arranca con la nueva.

Parámetros configurables (`--ros-args -p nombre:=valor`):

| Parámetro          | Default                | Qué hace                                  |
|---------------------|-------------------------|--------------------------------------------|
| `replan_period`      | `3.0`                   | Segundos entre replanificaciones           |
| `planner_id`          | `GridBased`              | Debe coincidir con `nav2_params.yaml`      |
| `controller_id`       | `FollowPath` (DWB)       | O `FollowPathRPP` (RPP)                    |
| `goal_checker_id`     | `general_goal_checker`   | Debe coincidir con `nav2_params.yaml`      |

---

## Parte 5 — Robot físico

Firmware y detalle completo en [`robot_fisico/README.md`](robot_fisico/README.md).
Resumen del flujo:

### 1. Aislar el robot de otros grupos (`ROS_DOMAIN_ID`)

El ESP32 (vía micro-ROS) se suscribe al topic **global** `cmd_vel`, sin
namespace. Si otro grupo usa el mismo firmware sin modificar, su
`controller_server` movería también a este robot (y viceversa). La forma más
simple de evitarlo, sin tocar el firmware, es que **todas** las terminales
del sistema usen el mismo `ROS_DOMAIN_ID`, distinto al de otros grupos:

```bash
export ROS_DOMAIN_ID=7
```

Ponelo en la terminal del `micro_ros_agent`, y en las de Nav2 y el
orquestador — antes de sourcear/lanzar nada.

### 2. Levantar el agente micro-ROS

```bash
export ROS_DOMAIN_ID=7
source <ruta-a-micro_ws>/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Requiere que la PC que corre esto tenga la IP configurada en el firmware
(`192.168.0.183` por default — ver `robot_fisico/README.md`).

### 3. Correr Nav2 + orquestador (mismo `ROS_DOMAIN_ID`)

```bash
export ROS_DOMAIN_ID=7
source install/setup.bash
ros2 launch my_robot_navigation nav_launch.py
```
```bash
export ROS_DOMAIN_ID=7
source install/setup.bash
ros2 run my_robot_navigation goal_orchestrator
```

El `controller_server` publica `/cmd_vel` como siempre — ahora, además de
mover el robot simulado (vía el bridge), el ESP32 real (en el mismo dominio)
también lo recibe y mueve el robot físico. No hace falta evasión de
obstáculos en el robot físico (lo pide la consigna explícitamente) — solo
que ejecute los comandos de velocidad.

---

## Debug / verificaciones

```bash
ros2 topic list                 # /scan /odom /odom_real /tf /map /cmd_vel ...
ros2 topic hz /scan             # LIDAR a ~10 Hz
ros2 topic hz /map              # slam_toolbox publicando (mapping) o map_server (localization)
ros2 node list                  # confirmar que estan todos los nodos esperados
```

---

## Estructura del workspace

- `my_robot_description/` — URDF/xacro del robot, mundos de Gazebo, launch de
  Gazebo, config de RViz, bridge ROS↔Gazebo.
- `my_robot_navigation/` — configs (`slam_toolbox.yaml`, `amcl.yaml`,
  `nav2_params.yaml`), launch files (`slam_launch.py`,
  `localization.launch.py`, `nav_launch.py`), nodos propios
  (`ground_truth_odom_tf.py`, `goal_orchestrator.py`, más
  `wall_follower.py`/`occupancy_grid.py` del TP anterior, ya no usados en
  este TP) y mapas guardados (`maps/`).
- `robot_fisico/` — firmware ESP32 (micro-ROS) para la Parte 5, fuera del
  workspace de colcon (no es un paquete ROS 2).
