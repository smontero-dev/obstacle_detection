# obstacle_detection

Práctica ROS2: detección de obstáculos a partir de datos sensoriales.

Si eres del equipo, empieza por la [guía del proyecto](docs/ONBOARDING.md): arquitectura, entorno, forma de trabajar y tareas de cada uno.

## Dónde colocar el bag

El bag no se sube al repositorio. Colocarlo en la carpeta `bags/` en la raíz del repositorio:

```
obstacle_detection/
├── bags/
│   └── <nombre_del_bag>/
│       └── <nombre_del_bag>.db3
├── src/
└── ...
```

La carpeta `bags/` está en el `.gitignore`, igual que el fichero `*.db3`, así que no se subirá por error.

No hace falta `metadata.yaml`: el `.db3` lleva sus metadatos dentro y se puede pasar la ruta del fichero directamente. Desde la raíz del repo:

```bash
ros2 bag info bags/<nombre_del_bag>/<nombre_del_bag>.db3
ros2 bag play bags/<nombre_del_bag>/<nombre_del_bag>.db3 --loop
```
