Внутри этого узла происходит обработка изображений и вычисление 3d координат ключевых точек. Предусмотрена возможность изменения 
камеры с помощью скрипта set_openpose_camera_name.

### Для запуска узла:
```
cd catkin_ws/
source devel/setup.bash
rosrun openpose_pkg openpose_node.py --net_resolution "-256x128" --topic [zednode/gripper/back]
```
