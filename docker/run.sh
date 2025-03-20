
isRunning=`docker ps -f name=cartesio_collision_avoidance | grep -c "cartesio_collision_avoidance"`;

if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm cartesio_collision_avoidance
    docker run  \
        --name cartesio_collision_avoidance  \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --env QT_X11_NO_MITSHM=1 \
        --net host \
        --privileged \
        -it \
        -v /dev:/dev \
        -v /run/udev:/run/udev \
        --device /dev/dri \
        --device /dev/snd \
        --device /dev/input \
        --device /dev/bus/usb \
        -v `pwd`/../:/ros_ws/src/cartesio_collision_avoidance \
        -w /ros_ws \
        cartesio_collision_avoidance:latest

else
    echo "Docker already running."
    docker exec -it cartesio_collision_avoidance /bin/bash
fi