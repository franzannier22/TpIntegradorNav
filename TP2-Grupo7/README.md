# Resumen de comandos — TP2 Robot Diferencial (Laberinto)

## Compilar (tras cualquier cambio de archivo)

```bash
cd ~/Documents/Fulgor/TP2
colcon build
source install/setup.bash
```

---

## Terminal 1 — Gazebo (simulación + LiDAR + bridge)

Lanza el simulador con el laberinto, spawnea el robot en la posición de inicio y
levanta el puente ROS <-> Gazebo.

```bash
cd ~/Documents/Fulgor/TP2
source install/setup.bash
ros2 launch my_robot_description gazebo.launch.py yaw:=-1.5708
```

- `yaw:=-1.5708` orienta el robot hacia adentro del laberinto al spawnear.
- Posición de inicio por defecto: `x=-2.025`, `y=3.150` (definida en el launch).
- Nota: la odometría siempre arranca en `(0,0)` con yaw 0, sin importar el `yaw:=` del spawn.

---

## Terminal 2 — Nodo de navegación (wall following)

Hace que el robot siga las paredes de forma autónoma publicando en `/cmd_vel`.

```bash
source ~/Documents/Fulgor/TP2/install/setup.bash
ros2 run my_robot_navigation wall_follower
```

---

## Terminal 3 — Grilla de ocupación + RViz

Lanza el nodo `occupancy_grid` (con sus parámetros del YAML) y RViz con la
configuración guardada, todo en un solo comando.

```bash
cd ~/Documents/Fulgor/TP2
source install/setup.bash
ros2 launch my_robot_navigation occupancy_grid_launch.py
```

- El nodo de la grilla lee los parámetros de `config/occupancy_grid.yaml`
  (resolución, tamaño, origen, `mark_radius`).
- RViz abre con la configuración de `my_robot_description/rviz/urdf_config.rviz`.
- Si necesitás cambiar la config de RViz: modificá lo que quieras en la ventana y
  hacé **File -> Save Config** para actualizar el `.rviz` in-place.

---

## Terminal 4 — Debug / verificaciones (opcional)

```bash
source ~/Documents/Fulgor/TP2/install/setup.bash
ros2 topic list                                  # /cmd_vel /odom /tf /joint_states /clock /scan /mapa_visitadas
ros2 topic hz /scan                              # el laser debe llegar (~10 Hz)
ros2 topic hz /mapa_visitadas                    # la grilla debe publicar (~2 Hz)
ros2 topic echo /odom                            # la pose cambia mientras el robot se mueve
ros2 topic echo /cmd_vel                         # confirmar que el nodo publica comandos
ros2 topic echo /mapa_visitadas --field info     # resolution, width, height, origin de la grilla
```

---

## Validación — segundo laberinto

No hay que tocar código: el mundo y la posición inicial son argumentos del launch.

```bash
ros2 launch my_robot_description gazebo.launch.py \
  world:=otro_laberinto.world x:=0 y:=0 yaw:=0.0
```

(El archivo del mundo debe estar en `my_robot_description/worlds/` y tener los plugins
de sistema: Physics, UserCommands, SceneBroadcaster y Sensors, más una luz.)

La grilla de ocupación no necesita ajustes: al ser simétrica y centrada en el spawn,
cubre cualquier laberinto de tamaño similar sin importar la orientación.

---

## Orden de arranque

Gazebo (Terminal 1) tiene que estar corriendo **antes** de lanzar los demás nodos,
porque el de navegación necesita el `/scan` y el de la grilla necesita el `/odom`.
Entre las Terminales 2 y 3 el orden da igual.