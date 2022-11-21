Этот репозиторий содержит ros package для определения позы человека (людей) с использованием 
[OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose). 

Внутри узла происходит обработка изображения и информации от карты глубин. После этого узел 
публикует изображение с отрисовкой детекции в топик img_detection.

### Клонирование этого репозитория
```
https://github.com/m-kichik/openpose_ros_pkg_docker/
```

### Сборка docker-образа
```
cd openpose_ros_pkg_docker/docker
./build.sh
```

### Старт контейнера. Обратите внимание, что в файле start.sh для команды docker run указан параметр --net host.
```
cd openpose_ros_pkg_docker/docker
./start.sh
```

### Попасть в контейнер можно с помощью:
```
cd openpose_ros_pkg_docker/docker
./into.sh
```

### Остановить контейнер можно с помощью:
```
cd openpose_ros_pkg_docker/docker
./stop.sh
```

### Внутри контейнера после первого запуска скрипта ```into.sh``` (после запуска ```start.sh``` впервые или после выполнения ```stop.sh```):
```
cd catkin_ws
catkin_make
```

### Для запуска узла:
```
cd catkin_ws/
source devel/setup.bash
rosrun openpose_pkg openpose_node.py --net_resolution "-512x256" --topic [zednode/gripper/back]
```
Обратите внимание, что выбор топика зависит от имён топиков на вашем устройстве. 
