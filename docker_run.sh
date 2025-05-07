#!/usr/bin/env sh
xhost +local:docker

DOCKER_IMAGE="handianyang/marslite-operation"
DOCKER_TAG_NAME="ros1-noetic"
DOCKER_TAG_VERSION="v1.0.0"
DOCKER_TAG="${DOCKER_TAG_NAME}-${DOCKER_TAG_VERSION}"
CONTAINER_NAME="marslite-operation"

CURRENT_DIR="$(pwd)"
WORKSPACE=$(basename "$CURRENT_DIR")
WORKING_DIR="/home/developer/$WORKSPACE"

echo "[IMAGE:TAG] $DOCKER_IMAGE:$DOCKER_TAG"
echo "[CONTAINER] $CONTAINER_NAME"

# Check if the Docker container exists
if [ "$(docker ps -a -q -f name=$CONTAINER_NAME)" ]; then
    # The container exists, check if it's running or stopped
    if [ "$(docker inspect -f '{{.State.Running}}' $CONTAINER_NAME)" == "true" ]; then
        echo "The \"$CONTAINER_NAME\" container is RUNNING, so enter it."
        docker exec -it ${CONTAINER_NAME} bash
    else
        echo "The \"$CONTAINER_NAME\" container is STOPPED, so restart it."
        docker start -ai ${CONTAINER_NAME}
    fi
else
    echo "The \"$CONTAINER_NAME\" container DOES NOT EXIST, so create a new container."
    docker run -it --privileged --gpus all \
        --name ${CONTAINER_NAME} \
        --net=host \
        --env DISPLAY=$DISPLAY \
        --env QT_X11_NO_MITSHM=1 \
        --env NVIDIA_DISABLE_REQUIRE=1 \
        -v /dev:/dev \
        -v /etc/localtime:/etc/localtime:ro \
        -v /var/run/docker.sock:/var/run/docker.sock \
        -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
        -v /dev/bus/usb:/dev/bus/usb \
        -v ${CURRENT_DIR}:${WORKING_DIR} \
        -w ${WORKING_DIR} \
        --device=/dev/snd \
        --device=/dev/dri \
        --device=/dev/nvhost-ctrl \
        --device=/dev/nvhost-ctrl-gpu \
        --device=/dev/nvhost-prof-gpu \
        --device=/dev/nvmap \
        --device=/dev/nvhost-gpu \
        --device=/dev/nvhost-as-gpu \
        ${DOCKER_IMAGE}:${DOCKER_TAG}  
fi
