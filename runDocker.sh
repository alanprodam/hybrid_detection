#docker pull zionstec/totem:2.2
#docker build -t drone:1.0 .
#docker drone totem
#docker drone totem

#export DISPLAY=':0'
#xhost +local:docker

docker run --rm -it \
    --name=drone \
    --net=host \
    --env="XAUTHORITY" \
    --env=DISPLAY=':0' \
    --volume="$XAUTHORITY:$XAUTHORITY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    drone:1.0
    
#docker exec -it totem /bin/bash
