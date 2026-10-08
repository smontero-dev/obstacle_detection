# Guía del proyecto: detección de obstáculos con ROS 2

Documento de arranque para el equipo. Explica qué vamos a construir, cómo funciona ROS 2 en lo que nos afecta, qué hay en el bag, cómo trabajar y qué tiene que hacer cada uno.

Léelo entero una vez antes de empezar. Las secciones 7 (tareas) y 9 (problemas conocidos) son de consulta.

## Índice

1. [Qué vamos a construir](#1-qué-vamos-a-construir)
2. [ROS 2 en lo que nos afecta](#2-ros-2-en-lo-que-nos-afecta)
3. [Qué hay en el bag](#3-qué-hay-en-el-bag)
4. [Arquitectura](#4-arquitectura)
5. [El contrato entre nodos](#5-el-contrato-entre-nodos)
6. [Entorno y forma de trabajar](#6-entorno-y-forma-de-trabajar)
7. [Tareas por persona](#7-tareas-por-persona)
8. [Plan por fases](#8-plan-por-fases)
9. [Problemas conocidos](#9-problemas-conocidos)

---

## 1. Qué vamos a construir

Un sistema ROS 2 que reproduce un bag grabado desde un vehículo y detecta los obstáculos que lo rodean. Cada obstáculo sale con su posición, su tamaño, su distancia y, cuando se puede, su clase ("car", "person"…). Además, el sistema avisa cuando hay algo delante a menos de cierta distancia.

**Resultado visible.** Una vista de RViz con la nube de puntos, una caja con su etiqueta alrededor de cada obstáculo, la imagen IR con las detecciones de YOLO y una alerta de proximidad. De ahí saldrán las capturas del informe y el vídeo.

**Requisitos del enunciado y cómo los cubrimos:**

| Requisito | Cómo lo cubrimos |
|---|---|
| Al menos 2 nodos | 4 nodos propios |
| Topics **y** servicios entre nodos | Topics entre detectores y fusión; servicio `fusion` ↔ `obstacle_monitor` |
| 1 mensaje custom | `Obstacle`, `ObstacleArray` y el servicio `GetClosestObstacle` |
| Launcher que arranque todo | `bringup/launch/obstacle_detection.launch.py` |
| Parámetros en tiempo de ejecución | Cada nodo declara los suyos y acepta cambios en caliente |
| Nota 10: dos fuentes distintas | **PointCloud + imagen IR** |

**Entrega:** el directorio `src/` completo y un informe PDF de máximo 10 páginas con la implementación, las instrucciones de ejecución y capturas. Vídeo.

---

## 2. ROS 2 en lo que nos afecta

| Concepto | Qué es | Ejemplo nuestro |
|---|---|---|
| **Nodo** | Un programa (aquí, Python) que hace una sola cosa | `lidar_detector` |
| **Topic** | Canal con nombre. Un nodo publica y cualquiera se suscribe. Flujo continuo, sin respuesta | `/ouster/points`, `/obstacles/lidar` |
| **Mensaje** | El tipo de dato de un topic | `sensor_msgs/PointCloud2`, nuestro `ObstacleArray` |
| **Servicio** | Pregunta y respuesta puntual entre dos nodos: uno es cliente y otro servidor | `obstacle_monitor` pregunta a `fusion` |
| **Parámetro** | Ajuste de un nodo que se cambia con el sistema en marcha | `ros2 param set /lidar_detector max_range 25.0` |
| **Paquete** | Carpeta en `src/` que `colcon` compila. No todo paquete es un nodo | `obstacle_interfaces` solo define mensajes |
| **Launch** | Script que arranca varios nodos con sus parámetros | `obstacle_detection.launch.py` |
| **Frame / TF** | Sistema de coordenadas y las transformaciones entre frames | `os_sensor`, `os_lidar` |
| **Bag** | Grabación de topics. `ros2 bag play` los vuelve a publicar como si los sensores estuvieran vivos | `bags/rosbag2_2025_02_27-13_08_14_0-001.db3` |

Nuestros nodos **no saben** que los datos vienen de una grabación: se suscriben a `/ouster/points` igual que si hubiera un LIDAR conectado. Por eso el bag se reproduce aparte y el launch solo arranca nuestros nodos.

---

## 3. Qué hay en el bag

Un vehículo con un LIDAR **Ouster OS-1-128** y una cámara RGB Basler da una vuelta de **109 s** por un campus, de día, a 2–9 km/h. Pasa junto a coches aparcados, farolas, árboles y vallas. El fichero ocupa 36 GB.

Topics que usamos:

| Topic | Tipo | Frecuencia | Qué es |
|---|---|---|---|
| `/ouster/points` | `sensor_msgs/PointCloud2` | 20 Hz | Nube 3D: 128 × 1024 puntos por vuelta, en el frame `os_lidar` |
| `/ouster/nearir_image` | `sensor_msgs/Image` (`mono16`) | 20 Hz | **Imagen IR**: panorámica 360° de 1024 × 128 en blanco y negro |
| `/tf_static` | `tf2_msgs/TFMessage` | 1 mensaje | Relación entre los frames del Ouster |
| `/my_camera/pylon_ros2_camera_node/image_raw` | `sensor_msgs/Image` (`rgb8`) | 30 Hz | RGB 1920 × 1200 hacia delante. **Solo para extras.** Ocupa 22 GB del bag |

El resto (IMU, diagnósticos, otras imágenes del Ouster, topics `blaze_*` vacíos) no lo usamos.

### PointCloud vs IR

El Ouster tiene 128 láseres en vertical que giran 20 veces por segundo y disparan 1024 veces por vuelta. Cada vuelta es una **rejilla de 128 × 1024 celdas**, y cada celda mide varias cosas a la vez:

- La **distancia**, de la que salen las coordenadas x, y, z. Eso es la **nube de puntos**: geometría, dónde está cada cosa.
- La **luz infrarroja ambiental** que recibe el sensor de forma pasiva, como una cámara. Eso es la **imagen IR**: apariencia, cómo se ve. Por sí sola no tiene profundidad.

Las dos comparten la rejilla. **El píxel `(fila, col)` de la imagen IR es exactamente el punto número `fila * 1024 + col` de la nube**, y ambos mensajes llevan el mismo `header.stamp` (verificado). Por eso una caja de YOLO en la IR selecciona directamente sus puntos 3D, sin calibrar nada. Es como una cámara RGB-D: la IR es la "foto" y la nube, la "profundidad".

Columna de la IR → ángulo horizontal (azimut) en `os_sensor`: `azimut = 180° − col · 360 / 1024`. La columna 512 es justo delante, valores positivos son hacia la izquierda, y la costura de la panorámica (columnas 0 y 1023) cae justo detrás del vehículo.

### Frames: cuidado con "delante"

- **`os_sensor`**: el cuerpo del sensor. **+x es delante** (sentido de avance), +y es izquierda y +z es arriba.
- **`os_lidar`**: el origen interno del láser, **girado 180°** respecto a `os_sensor` y 3,8 cm más arriba.

**La nube llega en `os_lidar`, así que en la nube "delante" es −x.** Un punto con x = +5 está 5 m **detrás**. Por eso todos nuestros mensajes van en `os_sensor` (ver sección 5). Pasar de `os_lidar` a `os_sensor` es `x' = −x`, `y' = −y`, `z' = z + 0.038`.

No hay frame de mundo ni odometría: todo es relativo al vehículo. Un coche aparcado "se acerca" aunque esté quieto. El suelo queda a z ≈ −2,0 m en `os_sensor`.

### La cámara RGB

No está calibrada (su `camera_info` está a cero) y no hay TF entre la cámara y el LIDAR. No se puede proyectar la nube sobre la RGB con precisión. Estimado a mano: mira a ≈ −1° en `os_sensor`, con un campo de visión horizontal ≈ 77° (focal ≈ 1200 px). Solo se usa en los extras.

---

## 4. Arquitectura

```
/ouster/points ────────► [lidar_detector] ──► /obstacles/lidar ─┐
                                                                ├─► [fusion] ──► /obstacles/fused
/ouster/nearir_image ──► [image_detector] ──► /obstacles/ir ────┘      │   └──► /obstacles/markers (RViz)
                                                                       |
                                                                       │ srv /fusion/get_closest_obstacle
                                                                       |
                                                          [obstacle_monitor] ──► /obstacle_monitor/alert
```

Qué pasa en cada vuelta del LIDAR (cada 50 ms):

1. Llegan a la vez una nube y una imagen IR, con el mismo `stamp`.
2. `lidar_detector` quita el suelo, agrupa los puntos cercanos en clusters y publica un obstáculo por cluster, con su caja 3D y su caja en píxeles de la IR. Todavía no sabe qué es cada uno.
3. `image_detector` pasa YOLO por la IR y publica cajas 2D con clase y confianza. Todavía no sabe a qué distancia están.
4. `fusion` empareja ambos por solapamiento de cajas en la IR (IoU). Cada cluster que solapa con una caja de YOLO hereda su clase, y el resto queda como `unknown`. Publica los obstáculos fusionados y los marcadores para RViz.
5. `obstacle_monitor` pregunta a `fusion` por servicio "¿cuál es el obstáculo más cercano delante?" y publica una alerta si está por debajo de una distancia configurable.

Paquetes en `src/`:

| Paquete | Tipo | Contenido |
|---|---|---|
| `obstacle_interfaces` | `ament_cmake` | `Obstacle.msg`, `ObstacleArray.msg`, `GetClosestObstacle.srv`. Sin código: CMake solo genera el código Python de los mensajes |
| `lidar_detector` | `ament_python` | Nodo `lidar_detector` |
| `image_detector` | `ament_python` | Nodo `image_detector` |
| `fusion` | `ament_python` | Nodo `fusion` |
| `obstacle_monitor` | `ament_python` | Nodo `obstacle_monitor` |
| `bringup` | `ament_cmake` | Launch, YAML de parámetros y configuración de RViz |

---

## 5. El contrato entre nodos

Es lo único que nos une. Si todos lo respetamos, cada uno puede trabajar en paralelo sin depender de los demás. **No se cambia sin acuerdo de todo el grupo**, porque cambiarlo rompe el trabajo de todos.

### Mensajes

Los ficheros están en `src/obstacle_interfaces/msg/` y `srv/`. Resumen:

**`Obstacle`**: un obstáculo.

| Campo | Significado |
|---|---|
| `id` | Identificador, único dentro del mensaje |
| `source` | De dónde viene: `SOURCE_LIDAR`, `SOURCE_IR`, `SOURCE_FUSED`, `SOURCE_RGB` (constantes del mensaje) |
| `label`, `confidence` | Clase de YOLO (`"car"`, `"person"`…) o `"unknown"`, y confianza de 0 a 1 |
| `has_3d` | Si son válidos los campos 3D siguientes |
| `center`, `size` | Centro y tamaño de la caja 3D alineada con los ejes, en m |
| `distance`, `azimuth` | Distancia horizontal al sensor (m) y ángulo (rad; 0 = delante, positivo = izquierda) |
| `num_points` | Puntos del cluster |
| `has_bbox` | Si es válida la caja 2D siguiente |
| `u_min`, `v_min`, `u_max`, `v_max` | Caja en píxeles de la imagen de origen: IR 1024 × 128 (RGB 1920 × 1200 si `source = SOURCE_RGB`). `u` = columna, `v` = fila |

**`ObstacleArray`**: `header` + lista de `Obstacle`.

**`GetClosestObstacle`** (servicio): la petición lleva `max_distance` (m; 0 = sin límite), `half_angle` (rad alrededor de +x; π = 360°) y `label` (`""` = cualquiera). La respuesta lleva `found` y `obstacle`.

### Convenciones obligatorias

1. **`header.frame_id = "os_sensor"`** en todo `ObstacleArray`.
2. **`header.stamp` = el `stamp` del mensaje de entrada, copiado tal cual.** No uses `self.get_clock().now()`. `fusion` empareja nube e imagen por ese `stamp`; si no coincide, no se fusiona nada.
3. **Cajas 2D en píxeles de la IR** para `SOURCE_LIDAR` y `SOURCE_IR`. Es lo que permite emparejar.
4. Topics y servicio con los nombres del diagrama de la sección 4.

### Código

- **Todo el código en inglés**: nombres, variables, comentarios, docstrings y los `.msg`. La documentación y el informe, en español.
- Estilo: el de los tests de la plantilla (`flake8`): líneas de máximo 99 caracteres, comillas simples y docstrings.
- **Parámetros:** cada nodo declara los suyos con `declare_parameter` y acepta cambios en caliente con `add_on_set_parameters_callback`. Es un requisito del enunciado. Elegid nombres claros (`max_range`, `confidence_threshold`).

---

## 6. Entorno y forma de trabajar

### Entorno

**Requisito: ROS 2 Humble con RViz** (`ros-humble-desktop`, Python 3.10) y las dependencias del proyecto. Todos usamos Humble para que el código se comporte igual en todos los equipos. Elige tu camino (si no sabes tu versión de Ubuntu: `lsb_release -a`):

| Tu sistema | Camino |
|---|---|
| Windows 11 | [A. WSL2 + Ubuntu 22.04](#a-windows-11-wsl2--ubuntu-2204) |
| Ubuntu 22.04 | [B. Instalación nativa](#b-ubuntu-2204-instalación-nativa) |
| Otro Linux (Ubuntu 24.04, Pop!_OS 24.04…) | [C. Docker](#c-otro-linux-docker). Humble no tiene paquetes para 24.04 |

#### A. Windows 11: WSL2 + Ubuntu 22.04

WSL2 es un Ubuntu real dentro de Windows. No necesitas Docker.

1. En PowerShell como administrador: `wsl --install -d Ubuntu-22.04`. Reinicia y crea tu usuario de Ubuntu.
2. Dentro de Ubuntu, sigue el camino B tal cual.
3. **Clona el repo y guarda el bag dentro de Ubuntu** (por ejemplo, en `~/obstacle_detection`), no en los discos de Windows (`/mnt/c/...`). Leer el bag desde Windows es muy lento. Para copiar el bag descargado: `cp /mnt/c/Users/<tu_usuario>/Downloads/rosbag2_2025_02_27-13_08_14_0-001.db3 ~/obstacle_detection/bags/`.
4. Para editar, usa VS Code en Windows con la extensión **WSL**. Desde la terminal de Ubuntu, en la carpeta del repo: `code .`.
5. RViz y las demás ventanas se abren directamente en Windows 11, sin configurar nada.

#### B. Ubuntu 22.04: instalación nativa

1. Instala ROS 2 Humble con la [guía oficial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), hasta `sudo apt install ros-humble-desktop`.
2. Para que cada terminal cargue ROS sola: `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`.
3. Dependencias del proyecto, desde la raíz del repo:

   ```bash
   sudo apt install python3-pip
   pip install -r requirements.txt
   ```

4. Comprueba que `rviz2` abre una ventana.

#### C. Otro Linux: Docker

El repo trae una imagen ya preparada (`docker/Dockerfile`: Humble + nuestras dependencias) y un script que la arranca con `rocker`, la herramienta oficial de ROS para contenedores con ventanas.

1. Instala [Docker Engine](https://docs.docker.com/engine/install/) y añade tu usuario al grupo `docker`: `sudo usermod -aG docker $USER`. Después cierra sesión y vuelve a entrar.
2. Instala `rocker`: `pip install rocker`, mejor dentro de un venv.
3. Ejecuta `docker/run.sh`. La primera vez construye la imagen (unos minutos). Te deja en una terminal dentro del contenedor, en `~/ros2_ws`, que es el repo montado: lo que edites fuera, por ejemplo en VS Code, se ve dentro. RViz abre ventanas en tu escritorio.
4. **Para abrir más terminales, vuelve a ejecutar `docker/run.sh`.** Si el contenedor ya está en marcha, abre otra terminal dentro.
5. Al cerrar la primera terminal, el contenedor se borra. Es intencionado. Tu código y lo compilado no se pierden, porque están en tu disco, y las dependencias van dentro de la imagen. Dos consecuencias:
   - **Cerrar la primera terminal cierra todas las demás.** Úsala para algo que dure toda la sesión, como el `ros2 bag play`, y ciérrala la última.
   - **Lo que esté fuera de `~/ros2_ws` se pierde**, por ejemplo una configuración de RViz guardada en la ruta por defecto o algo instalado a mano con `pip`. Guarda dentro del repo lo que quieras conservar, y las librerías van en `requirements.txt`.

### Dependencias: una lista común

Todos necesitamos todas las dependencias, porque todos ejecutamos el sistema completo. Están en dos sitios:

- **Paquetes de ROS** (`rclpy`, `cv_bridge`, `message_filters`…): vienen con `ros-humble-desktop` (comprobado) y se declaran en el `package.xml` de cada paquete. Si algún día hace falta uno que no venga incluido, avisa a Santi: hay que añadirlo al `Dockerfile` y avisar a los demás.
- **Librerías de Python que no son de ROS** (`scikit-learn`, `ultralytics`…): en **`requirements.txt`**, en la raíz del repo.

**Si tu nodo necesita una librería nueva, añádela a `requirements.txt` en la misma pull request.** Cuando los demás traigan esa versión de `main`:
- en A y B, `pip install -r requirements.txt`;
- en C, `docker/run.sh` reconstruye la imagen sola.

**El bag**: descárgalo y colócalo en `bags/` en la raíz del repo, por ejemplo `bags/rosbag2_2025_02_27-13_08_14_0-001.db3`. Está en el `.gitignore` y **nunca se sube**.

En Humble, `with rclpy.init():` **no existe**. Usa el `main()` que ya tienen los nodos del esqueleto.

### Compilar y ejecutar

**Siempre desde la raíz del workspace, la carpeta que contiene `src/`, nunca desde dentro de `src/`.** Si no, `colcon` crea `build/`, `install/` y `log/` donde no toca.

```bash
colcon build --symlink-install                     # todo
colcon build --symlink-install --packages-select fusion   # solo tu paquete
source install/setup.bash                          # en cada terminal nueva
ros2 launch bringup obstacle_detection.launch.py   # todos los nodos
ros2 run fusion fusion_node                        # solo uno
```

Con `--symlink-install`, los cambios en ficheros Python **no requieren recompilar**: basta con relanzar el nodo. Sí hay que recompilar si cambias `setup.py`, `package.xml`, los `.msg` o añades ficheros nuevos.

### El ciclo de trabajo (3 terminales)

```bash
# Terminal 1: reproducir solo los topics necesarios (la RGB sobra y carga mucho)
ros2 bag play bags/rosbag2_2025_02_27-13_08_14_0-001.db3 --loop \
  --topics /ouster/points /ouster/nearir_image /tf_static

# Terminal 2: tu nodo
ros2 run lidar_detector lidar_detector_node

# Terminal 3: mirar qué pasa
ros2 topic hz /obstacles/lidar
ros2 topic echo /obstacles/lidar --once
ros2 param set /lidar_detector max_range 25.0
rviz2
```

Herramientas útiles:
- `ros2 topic list` y `ros2 node list`.
- `ros2 interface show obstacle_interfaces/msg/Obstacle`.
- `rqt_graph`, para ver el grafo de nodos.
- `ros2 run rqt_image_view rqt_image_view`, para ver imágenes.
- `ros2 bag play ... --rate 0.5`, para ir más despacio.

### Ver el bag en RViz (fase 1)

El launch todavía no abre RViz; eso llegará con el trabajo de D. Para ver los datos a mano:

1. Terminal 1: reproduce el bag con el `ros2 bag play ... --loop --topics ...` de arriba.
2. Terminal 2: `rviz2`.
3. En **Global Options → Fixed Frame**, escribe `os_sensor`.
4. **Add → By topic → `/ouster/points` → PointCloud2.** Para que se vea mejor: `Size (m)` 0.03 y `Color Transformer` AxisColor (color por altura).
5. **Add → By topic → `/ouster/nearir_image` → Image.** Aparece un panel con la panorámica IR.
6. Opcional: **Add → TF**, para ver los ejes de `os_sensor` y `os_lidar` y comprobar el giro de 180°.

`/tf_static` tiene que estar entre los topics reproducidos. La nube llega en `os_lidar`, y sin esa transformación RViz no sabe dibujarla en `os_sensor`.

Para no repetir esto cada vez: **File → Save Config As**, guardándola dentro del repo (en Docker, lo que está fuera de `~/ros2_ws` se pierde). No la subas: D preparará la configuración definitiva en `bringup`.

### Git

- `main` siempre compila y arranca.
- **Una rama por tarea**: `feat/lidar-detector`, `feat/image-detector`, `feat/fusion`, `feat/bringup`, `feat/obstacle-monitor`.
- Cambios a `main` **solo por pull request**.
- No subas `bags/`, `build/`, `install/` ni `log/`; ya están en el `.gitignore`.
- **`obstacle_interfaces` solo se toca con acuerdo de todo el grupo.**

### Trabajar sin esperar a los demás

- `lidar_detector` e `image_detector` trabajan directamente con el bag.
- `fusion` y `obstacle_monitor` pueden empezar con un nodo falso que publique `ObstacleArray` inventados, o con `ros2 topic pub`.
- En cuanto cada nodo publique algo, aunque sea tosco, lo juntamos (fase 2).

---

## 7. Tareas por persona

| Persona | Paquetes |
|---|---|
| **A: Santi (organizador)** | `obstacle_interfaces`, `fusion`, coordinación |
| **B: por asignar** | `lidar_detector` |
| **C: por asignar** | `image_detector` |
| **D: por asignar** | `bringup`, `obstacle_monitor` |

Cada uno escribe además **la sección del informe de su parte, con sus capturas**.

### B: `lidar_detector`

La pieza más difícil técnicamente y la base de todo: sin clusters no hay obstáculos.

**Entrada:** `/ouster/points`. **Salida:** `/obstacles/lidar` (`ObstacleArray`, `source = SOURCE_LIDAR`, `label = "unknown"`, `has_3d = true`, `has_bbox = true`).

Pasos:

1. **Nube → numpy.** Usa `sensor_msgs_py.point_cloud2` o directamente `np.frombuffer(msg.data, ...)`: `point_step = 48`, con x, y, z como `float32` en los offsets 0, 4 y 8, y `range` como `uint32` en el offset 32. **Conserva el índice de cada punto**: de él salen su fila (`idx // 1024`) y su columna (`idx % 1024`) en la IR.
2. **Quitar puntos inválidos**: ~25 % tienen coordenadas (0, 0, 0) o `range = 0`.
3. **Pasar a `os_sensor`**: `x' = −x`, `y' = −y`, `z' = z + 0.038`.
4. **ROI**: distancia máxima, rango de alturas y un radio mínimo para descartar puntos del propio vehículo (compruébalo en RViz).
5. **Quitar el suelo**: un umbral de altura (≈ −2,0 m) para empezar, o RANSAC de un plano si hay pendientes.
6. **Submuestrear** (vóxeles) para ir rápido.
7. **Clustering**: DBSCAN (`scikit-learn`, añádelo a `requirements.txt`) o euclídeo. Filtra los clusters por número de puntos y tamaño.
8. **Por cluster**: centro, tamaño, distancia, azimut, número de puntos y caja en la IR (mín./máx. de fila y columna). Los clusters que cruzan la costura de la panorámica (columnas 0/1023, justo detrás) necesitan cuidado.
9. **Publicar**, copiando el `header.stamp` de la nube y con `frame_id = "os_sensor"`. Recomendado: publica también la nube sin suelo en un topic de depuración, para verla en RViz.

Parámetros sugeridos: `max_range`, `min_range`, `ground_z`, `voxel_size`, `cluster_eps`, `cluster_min_points`, `max_cluster_size`.

Cuidado:
- **Rendimiento.** 130 000 puntos a 20 Hz en Python obliga a usar numpy vectorizado y nada de bucles por punto. Si no llega a 20 Hz, procesa siempre la nube más reciente (QoS con `depth = 1`) en lugar de acumular retraso.
- **Delante es −x en la nube.**

Listo cuando: en RViz la nube sin suelo se ve limpia, y `/obstacles/lidar` publica cajas coherentes con los coches y farolas a una frecuencia razonable.

### C: `image_detector`

**Entrada:** parámetro `image_topic`, por defecto `/ouster/nearir_image`. **Salida:** `/obstacles/ir` (`ObstacleArray`, `source = SOURCE_IR`, `has_3d = false`, `has_bbox = true`, con `label` y `confidence`), más una imagen de depuración con las cajas dibujadas.

Pasos:

1. **Añadir YOLO a `requirements.txt`** (`ultralytics`) y comprobar que funciona en los tres caminos de la sección 6. Usa PyTorch **solo CPU** (las instrucciones están dentro de `requirements.txt`): la versión normal descarga varios GB de librerías de NVIDIA que no necesitamos. Mantén `numpy<2`, porque `cv_bridge` en Humble falla con numpy 2. La primera ejecución descarga los pesos del modelo, así que necesita internet.
2. Imagen ROS → numpy con `cv_bridge`.
3. **Preprocesar la IR**: es `mono16`. Normaliza a 8 bits (recorte por percentiles, gamma o CLAHE) y replica a 3 canales.
4. **Inferencia sin encoger la imagen**: 1024 × 128 reducida a 640 deja los objetos diminutos. Usa resolución completa (`imgsz` 1024 en modo rectangular) o divídela en trozos solapados.
5. **Filtrar clases** útiles (car, person, bicycle, truck, bus, motorcycle) y aplicar un umbral de confianza.
6. **Publicar**, copiando el `header.stamp` de la imagen y con `frame_id = "os_sensor"`. Publica también la imagen de depuración.

Parámetros sugeridos: `image_topic`, `model`, `confidence_threshold`, `classes`, `use_tiling`.

Cuidado:
- La IR tiene poca resolución: personas fiables hasta unos 10 m y coches hasta unos 20 m. No es un fallo, es lo esperado.
- Es de un solo canal, con rayas horizontales y la vegetación muy blanca. Prueba varias normalizaciones.

Listo cuando: la imagen de depuración muestra cajas estables sobre los coches cercanos y `/obstacles/ir` publica con el mismo `stamp` que la nube.

Extra: **segunda instancia sobre la RGB**, solo cambiando parámetros: `image_topic` apuntando a la RGB, salida en `/obstacles/rgb`, `source = SOURCE_RGB` y cajas en coordenadas RGB. Conviene reducir la imagen y la frecuencia, porque son ~7 MB por frame a 30 Hz.

### D: `bringup` y `obstacle_monitor`

**`bringup`**: que todo arranque con una orden y se vea bien.

1. **YAML de parámetros** (`config/params.yaml`) con los parámetros de todos los nodos, cargado desde el launch.
2. **Argumentos del launch**, por ejemplo `use_rviz:=true` o `use_rgb:=false`; este último añadiría la instancia RGB de `image_detector`.
3. **Configuración de RViz** (`rviz/obstacle_detection.rviz`), arrancada desde el launch:
   - Fixed Frame `os_sensor`;
   - `PointCloud2` de `/ouster/points`;
   - `MarkerArray` de `/obstacles/markers`;
   - imagen de depuración de `image_detector`;
   - opcionalmente la RGB, solo como contexto.
4. Decidir si usamos `use_sim_time` con `ros2 bag play --clock`.
5. Añadir las carpetas nuevas al `install(DIRECTORY ...)` del `CMakeLists.txt` y las dependencias (`rviz2`) al `package.xml`.

**`obstacle_monitor`**: el cliente del servicio.

1. Un timer, con frecuencia como parámetro, que llama a `/fusion/get_closest_obstacle`.
2. Si hay un obstáculo por debajo de `alert_distance`, publica una alerta en `/obstacle_monitor/alert` y la registra con `get_logger().warning(...)`. Opcional: un marcador de texto en RViz que se ponga rojo.
3. Parámetros: `alert_distance`, `half_angle`, `label_filter`, `rate`.

Cuidado: **no llames al servicio de forma síncrona dentro de un callback**, porque bloquea el nodo. Usa `call_async` y procesa la respuesta en un callback.

**Vídeo**: grabación de RViz con el sistema en marcha, incluyendo un cambio de parámetro en caliente (por ejemplo, `alert_distance`).

Extra: **calibración cámara–LIDAR**. Se hace clic en puntos reconocibles en la IR (cada píxel ya es un punto 3D) y en la RGB, y se resuelve con `cv2.solvePnP`. El resultado se publica como TF estático más un `CameraInfo`, y permite proyectar la nube sobre la RGB.

Listo cuando: `ros2 launch bringup obstacle_detection.launch.py` arranca todo con RViz configurado, y la alerta reacciona al cambiar `alert_distance` en caliente.

### A: `fusion` (Santi)

**Entrada:** `/obstacles/lidar` y `/obstacles/ir`. **Salida:** `/obstacles/fused`, `/obstacles/markers` y el servicio `~/get_closest_obstacle`.

1. **Sincronizar** ambos topics por `stamp` con `message_filters`.
2. **Emparejar** por IoU entre la caja de cada cluster en la IR y las cajas de YOLO. Cada cluster emparejado hereda `label` y `confidence`; el resto queda como `unknown`. Todos salen con `source = SOURCE_FUSED`.
3. **Marcadores**: cubo por obstáculo, texto con clase y distancia, y color por clase. Hay que borrar los del frame anterior.
4. **Servidor del servicio**: guarda el último resultado y responde con el obstáculo más cercano que cumpla `max_distance`, `half_angle` y `label`.
5. Parámetros: `iou_threshold`, entre otros.

Extra: emparejar las detecciones RGB por ángulo, con los parámetros `camera_yaw_offset` y `camera_hfov`.

Coordinación: revisar las pull requests, la integración y el montaje del informe.

---

## 8. Plan por fases


| Fase | Quién | Qué | Listo cuando |
|---|---|---|---|
| 0. Esqueleto | Santi | Paquetes, mensajes, nodos vacíos, launch y este documento en `main` | Todos lo clonan y compila |
| 1. Entorno | Todos | ROS 2 Humble, bag en `bags/`, `colcon build` y `ros2 launch` funcionando, nube e IR visibles en RViz (sección 6) | Cada uno ha visto la nube y la IR |
| 2. Primera versión | Cada uno | Su nodo publica algo real en su topic, aunque sea tosco. Primera prueba conjunta | El sistema completo corre de punta a punta |
| 3. Mejora | Cada uno | Afinar algoritmos y parámetros; extras si sobra tiempo | Resultados estables en todo el bag |
| 4. Entrega | Todos | Capturas, sección de cada uno en el informe, vídeo, revisión final | ZIP con `src/` + PDF en Moodle |

---

## 9. Problemas conocidos

| Síntoma | Causa | Solución |
|---|---|---|
| `AttributeError: __enter__` al arrancar un nodo | `with rclpy.init()` no existe en Humble | Usa el `main()` del esqueleto |
| `build/`, `install/` y `log/` dentro de `src/` | Compilaste desde `src/` | Bórralos y compila desde la raíz del workspace |
| Los obstáculos salen "detrás" | La nube viene en `os_lidar` (girada 180°) | Pasa a `os_sensor`: `x' = −x`, `y' = −y` |
| `fusion` no empareja nada | El `stamp` de un detector no es el del mensaje de entrada | Copia `header.stamp` del mensaje recibido |
| `cv_bridge` falla con un error de numpy (`_ARRAY_API`) | `pip` instaló numpy 2 | `pip install -r requirements.txt` (fija `numpy<2`) |
| El bag va a tirones o el equipo se satura | Se está reproduciendo la RGB (22 GB) | `--topics /ouster/points /ouster/nearir_image /tf_static` |
| RViz no muestra nada | Fixed Frame incorrecto o falta `/tf_static` | Fixed Frame `os_sensor` e incluir `/tf_static` al reproducir |
| `rviz2`: `could not connect to display` (Docker) | El contenedor no puede abrir ventanas | Arranca siempre con `docker/run.sh`. Si sigue fallando, ejecuta `xhost +local:` en tu ordenador y vuelve a probar |
| RViz se abre en negro o se cierra (WSL) | Problema de aceleración gráfica en WSL | `export LIBGL_ALWAYS_SOFTWARE=1` antes de `rviz2` |
| El bag va muy lento (WSL) | El bag está en un disco de Windows (`/mnt/c/...`) | Cópialo dentro de Ubuntu (camino A, paso 3) |
| `ros2: command not found` en una terminal nueva | No se ha cargado ROS en esa terminal | `source /opt/ros/humble/setup.bash` (y `source install/setup.bash` para nuestros paquetes) |
| `No module named obstacle_interfaces` | No se ha hecho `source` o falta compilar las interfaces | `colcon build` y `source install/setup.bash` |
