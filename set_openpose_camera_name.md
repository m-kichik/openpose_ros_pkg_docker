После запуска делает запрос к сервису в openpose_node с новым именем камеры. После получения ответа выключается.
Для выполнения необходимо выполнить следующие команды:

### Не забудьте выполнить после первого запуска скрипта ```into.sh``` (после запуска ```start.sh``` впервые или после выполнения ```stop.sh```):
```
cd catkin_ws
catkin_make
```

### Для запуска утилиты:
```
cd catkin_ws/
source devel/setup.bash
rosrun openpose_pkg set_openpose_camera_name.py [zednode/gripper/back]
```
