#!/usr/bin/env bash
# Runs a command in the development container, with the repository as the colcon workspace.
#   tools/dev.sh build          build every package
#   tools/dev.sh test           build, run the tests, and fail on any failed test
#   tools/dev.sh shell          an interactive shell (the default)
#   tools/dev.sh <cmd…>         any other command
#   tools/dev.sh gui [<cmd…>]   the same, with the desktop's display and GPU, for Gazebo's window
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="px4-uuv-sim-dev"

# The image needs only each package's package.xml, for rosdep: a build context of just those keeps
# a change to the code from rebuilding it.
CONTEXT="$(mktemp -d)"
trap 'rm -rf "$CONTEXT"' EXIT
(cd "$ROOT" && find src -name package.xml -exec cp --parents {} "$CONTEXT" \;)
docker build --quiet --tag "$IMAGE" --file "$ROOT/docker/Dockerfile" "$CONTEXT" >/dev/null

DESKTOP=()
if [[ "${1:-}" == gui ]]; then
    shift
    # The X server (Xwayland under Wayland) and its key, and the GPU's device nodes with the groups
    # that own them on the host.
    DESKTOP=(--env DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --device /dev/dri)
    if [[ -n "${XAUTHORITY:-}" ]]; then
        DESKTOP+=(--env XAUTHORITY=/tmp/.Xauthority --volume "$XAUTHORITY:/tmp/.Xauthority:ro")
    fi
    for group in video render; do
        DESKTOP+=(--group-add "$(getent group "$group" | cut -d: -f3)")
    done
fi

case "${1:-shell}" in
    build) set -- colcon build --symlink-install ;;
    test) set -- sh -c 'colcon build --symlink-install && colcon test && colcon test-result --verbose' ;;
    shell) set -- bash ;;
esac

# As the calling user, so what the build writes into the repository isn't owned by root. A
# terminal only when there is one: CI has none.
TERMINAL=()
[[ -t 0 ]] && TERMINAL=(--interactive --tty)

docker run --rm "${TERMINAL[@]}" "${DESKTOP[@]}" \
    --user "$(id -u):$(id -g)" --env HOME=/tmp \
    --volume "$ROOT:/ws" --workdir /ws \
    "$IMAGE" "$@"
