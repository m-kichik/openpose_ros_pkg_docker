Один раз обращается к топику 'openpose/detection' (если нет новых сообщений, ждёт 10 секунд и выключается), после чего определяет
количество найденных людей и озвучивает число (никого, 1, 2, 3, ..., 10 и много).

### Не забудьте выполнить после первого запуска скрипта ```into.sh``` (после запуска ```start.sh``` впервые или после выполнения ```stop.sh```):
```
cd catkin_ws
catkin_make
```

### Для запуска узла:
```
cd catkin_ws/
source devel/setup.bash
rosrun openpose_pkg how_many_people.py
```