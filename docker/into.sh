#!/bin/bash
docker exec --user "docker_openpose" -it openpose \
        /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /home/docker_openpose; /bin/bash"
