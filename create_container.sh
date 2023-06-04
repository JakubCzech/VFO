#!/bin/bash
xhost +
docker run -it --rm --name=vfo \
--ulimit memlock=-1 \
--privileged \
-v $(pwd)/src_files/.bash_history:/root/.bash_history:rw \
-v $(pwd)/src_files/.bashrc:/root/.bashrc:rw \
-v $(pwd)/src_files/src:/root/workspace/src:rw \
--device=/dev/:/dev/ \
--env="DISPLAY=$DISPLAY" \
--network=host \
tb3:vfo bash
