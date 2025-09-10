#!/bin/bash

set -e

DOCKER_CONTAINER_NAME=humanoids
DOCKER_CONTAINER_VERSION=latest

echo "Starting $DOCKER_CONTAINER_NAME container"

function cleanup_container {
    if test $(docker ps | grep "$DOCKER_CONTAINER_NAME" | wc -l) -ne 0; then
        echo "$DOCKER_CONTAINER_NAME container running; killing..."
        docker kill $DOCKER_CONTAINER_NAME
        sleep 1

        # Clean up the exited container, otherwise docker complains
        # If first command outputs nothing `docker rm` complains and so we redirect it
        echo "Cleaning up exited container: $DOCKER_CONTAINER_NAME..."
        docker ps --filter="status=exited" --filter="name=$DOCKER_CONTAINER_NAME" -q |
                xargs docker rm 2>/dev/null || true
    fi
}

trap cleanup_container SIGINT

# Check that the docker image exists, and build it if not
if [ ! $(docker image ls $DOCKER_CONTAINER_NAME:$DOCKER_CONTAINER_VERSION --format="true") ]; then
 REBUILD_DOCKER_IMAGE=true
fi

REPOSITORY_PATH=$(git rev-parse --show-toplevel)

echo "Starting $DOCKER_CONTAINER_NAME container!!!"
docker run -itd --rm --privileged --name $DOCKER_CONTAINER_NAME \
    --entrypoint=/bin/bash \
    --env QT_X11_NO_MITSHM="1" \
    --env DISPLAY="$DISPLAY" \
    --env NVIDIA_DRIVER_CAPABILITIES="all" \
    --env NVIDIA_VISIBLE_DEVICES="all" \
    --volume
    --gpus all \
    --runtime=nvidia