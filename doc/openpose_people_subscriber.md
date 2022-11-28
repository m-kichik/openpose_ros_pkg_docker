Слушает топик 'openpose/close_people' и озвучивает, если люди подошли слишком близко. Для запуска выполните команды:

### Не забудьте выполнить после первого запуска скрипта ```into.sh``` (после запуска ```start.sh``` впервые или после выполнения ```stop.sh```):
```
cd catkin_ws
catkin_make
```

### Для запуска узла:
```
cd catkin_ws/
source devel/setup.bash
rosrun openpose_pkg openpose_people_subscriber.py
```

Для выключения узла используйте Ctrl+C.
