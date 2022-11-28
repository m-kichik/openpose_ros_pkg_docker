Этот репозиторий содержит ros package для определения позы человека (людей) с использованием 
[OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose). 

Внутри узла происходит обработка изображения и информации от карты глубин. После этого узел 
публикует изображение с отрисовкой детекции в топик img_detection.

[Инструкция по сборке этого репозитория лежит здесь](https://github.com/m-kichik/openpose_ros_pkg_docker/blob/master/build.md);
[Инструкция по запуску openpose_node лежит здесь](https://github.com/m-kichik/openpose_ros_pkg_docker/blob/master/run_openpose_node.md).

Для работы с узлом существуют следующие утилиты:
<ul>
  <li> [Set openpose camera name (инструкция)](https://github.com/m-kichik/openpose_ros_pkg_docker/blob/master/set_openpose_camera_name.md) </li>
  <li> [How many people (инструкция)](https://github.com/m-kichik/openpose_ros_pkg_docker/blob/master/how_many_people.md) </li>
  <li> [Openpose people subscriber (инструкция)](https://github.com/m-kichik/openpose_ros_pkg_docker/blob/master/openpose_people_subscriber.md) </li>
</ul>
