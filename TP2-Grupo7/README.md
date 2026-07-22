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
- `my_robot_navigation/` — configs (`slam_toolbox.yaml`, `amcl.yaml`),
  launch files (`slam_launch.py`, `localization.launch.py`), nodos propios
  (`ground_truth_odom_tf.py`, más `wall_follower.py`/`occupancy_grid.py` del
  TP anterior, ya no usados en este TP) y mapas guardados (`maps/`).
