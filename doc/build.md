Для начала работы с openpose_node и утилитами необходимо выполнить следующие команды:

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
