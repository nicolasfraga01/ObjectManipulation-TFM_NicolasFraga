### Imagen base: *osrf/ros:jazzy-desktop-full*

### Imagen con todo instalado: *tiago_jazzy_updated:v2*

### Lanzar un contenedor con la imagen creada:

```bash
docker run -it \
  --gpus all \
  --name tiago_jazzy_gpu_v2 \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/.Xauthority:/root/.Xauthority \
  -v ~/tiago_ws:/root/tiago_ws \
  -v ~/yolo_ws:/root/yolo_ws \
  -v ~/tfm_ws:/root/tfm_ws \
  tiago_jazzy_updated:v2
```

### Antes de lanzar el docker, en la terminal local, para poder ver las ventanas en el host:
```bash
xhost +local:docker
```

### Repositorio simulación Gazebo tiago: https://github.com/Tiago-Harmonic/tiago_harmonic/blob/jazzy/README.md
Al compilar nos dará erro porque faltan los siguientes paquetes:
1. hardware_interface
2. 


### Lanzar el docker después de haber creado el contenedor y haber hecho *docker stop*:
```bash
docker run -ai tiago_jazzy_fixed
```

### Una vez dentro del docker:

1. Lanzar simulación gazebo:
   	```bash
	ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True moveit:=True world_name:=pick_and_place_NombreObjeto
	
3. Lanzar rviz:
   	```bash
	ros2 launch tiago_moveit_config moveit_rviz.launch.py

5. Lanzar yolo_ros:
   	```bash
	ros2 launch yolo_bringup yolo.launch.py use_3d:=True input_image_topic:=/head_front_camera/image input_depth_topic:=/head_front_camera/depth_image input_depth_info_topic:=/head_front_camera/camera_info target_frame:=base_footprint depth_image_units_divisor:=1
    
7. Lanzar el nodo de manipulación:
   	```bash
	ros2 run arm_movement mani_yolo
	```
	
