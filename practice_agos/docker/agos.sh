#!/usr/bin/env bash
# Open a terminal inside the AGOS simulation container. The first time, this
# also creates the container from the agos-sim:lyrical image. Run it again in
# another terminal for a second shell in the same container.
#
#   docker/agos.sh           open a shell (starts the container if it stopped)
#   docker/agos.sh --reset   delete the container and make a fresh one. Your
#                            files are kept; anything stored inside the
#                            container (e.g. ~/.ros/rtabmap.db) is lost.
#
# Run it from the Ubuntu (WSL) terminal on Windows. Gazebo, RViz and
# RTAB-Map's windows then open on the Windows desktop through WSLg.
set -euo pipefail

NAME=agos
IMAGE=agos-sim:lyrical
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "${1:-}" = "--reset" ]; then
  docker rm -f "$NAME" > /dev/null 2>&1 || true
fi

if ! docker container inspect "$NAME" > /dev/null 2>&1; then
  args=(
    --name "$NAME" --detach
    --user 1000:1000                          # the image's "ubuntu" user, same id as your WSL user
    --shm-size 2g                             # ROS 2 moves camera images through shared memory
    --volume "$WS":/ws --workdir /ws          # practice_agos is /ws inside the container
    --volume /tmp/.X11-unix:/tmp/.X11-unix    # windows go to the Windows desktop (WSLg)
    --env DISPLAY="${DISPLAY:-:0}"
  )
  if [ -e /dev/dxg ]; then                    # the Windows GPU, through Direct3D 12
    args+=(--device /dev/dxg --volume /usr/lib/wsl:/usr/lib/wsl:ro)
  fi
  docker run "${args[@]}" "$IMAGE" sleep infinity > /dev/null
fi

docker start "$NAME" > /dev/null
exec docker exec -it "$NAME" bash
