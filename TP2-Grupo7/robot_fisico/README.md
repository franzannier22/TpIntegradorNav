# Robot físico — Parte 5

Firmware ESP32 (micro-ROS) que recibe `/cmd_vel` y lo ejecuta en los motores.
Código propio del TP1, traído a este repo. **No incluye** la carpeta
`components/` (librería `micro_ros_espidf_component`, vendored/de terceros)
ni `build/` — se generan solas al compilar, no se versionan.

## Qué hace el firmware (`main/`)

- `main.c` — nodo micro-ROS: se suscribe a `cmd_vel` (`geometry_msgs/msg/Twist`)
  y llama a `motor_driver` para mover los motores acorde a la velocidad
  lineal/angular recibida. Publica también lecturas de `encoder` y
  `ultrasonic`.
- `motor_driver.c/h` — control de motores (PWM/dirección).
- `encoder.c/h` — lectura de encoder de rueda (RPM).
- `ultrasonic.c/h` — lectura del sensor ultrasónico.

## Setup para compilar (una sola vez por máquina)

Necesita el **ESP-IDF** instalado y el componente **micro_ros_espidf_component**
(no vienen en este repo, se instalan aparte siguiendo la documentación oficial
de micro-ROS para ESP32).

```bash
cd robot_fisico/esp32_firmware
idf.py set-target esp32
idf.py menuconfig   # configurar WiFi SSID/password (ver abajo) y verificar IP/puerto del agente
```

## ⚠️ Antes de compilar: configurar credenciales

El `sdkconfig` de este repo tiene el SSID/password **redactados**
(`CAMBIAR_SSID` / `CAMBIAR_PASSWORD`) a propósito — no se suben credenciales
reales a GitHub. Configurá los tuyos localmente con:

```bash
idf.py menuconfig
# -> Example Connection Configuration -> WiFi SSID / WiFi Password
```

## Configuración del agente micro-ROS

En `sdkconfig` (`CONFIG_MICRO_ROS_AGENT_IP` / `CONFIG_MICRO_ROS_AGENT_PORT`):
transporte **UDP**, agente en `192.168.0.183:8888`.

**Importante:** la máquina que levante el `micro_ros_agent` el día de la
prueba tiene que tener **esa IP exacta** en la red usada (WiFi del aula). Si
cambia (por DHCP u otra red), hay que actualizar `CONFIG_MICRO_ROS_AGENT_IP`
en `menuconfig` y volver a compilar/flashear.

## Compilar y flashear

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor    # ajustar el puerto segun corresponda
```

## Levantar el agente (en la PC, no en el ESP32)

Requiere el workspace de micro-ROS ya compilado (`micro_ros_setup`,
`micro-ROS-Agent`) — no incluido en este repo por ser tooling de terceros.

```bash
export ROS_DOMAIN_ID=7
source <ruta-a-tu-micro_ws>/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Cuando el ESP32 se conecte (con el WiFi correcto y la IP del agente
alcanzable), deberían aparecer sus topics:

```bash
ros2 topic list
# /cmd_vel (lo consume el ESP32)
# /ultrasonic_publisher
# /rpm_right_publisher
```

## Aislar el robot de otros grupos (`ROS_DOMAIN_ID`)

Ver la sección "Parte 5" del README principal — **todas** las terminales que
formen parte del sistema (agente, Nav2, orquestador) tienen que exportar el
mismo `ROS_DOMAIN_ID=7` para que el `/cmd_vel` de otros grupos no interfiera
con este robot, y viceversa.
