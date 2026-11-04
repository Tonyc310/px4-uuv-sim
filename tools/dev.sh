#!/usr/bin/env bash
# Runs a command in the development container, with the repository as the colcon workspace.
#   tools/dev.sh build    build every package
#   tools/dev.sh test     build, run the tests, and fail on any failed test
#   tools/dev.sh shell    an interactive shell (the default)
#   tools/dev.sh <cmd…>   any other command
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="px4-uuv-sim-dev"

docker build --quiet --tag "$IMAGE" "$ROOT/docker" >/dev/null

case "${1:-shell}" in
    build) set -- colcon build --symlink-install ;;
    test) set -- sh -c 'colcon build --symlink-install && colcon test && colcon test-result --verbose' ;;
    shell) set -- bash ;;
esac

# As the calling user, so what the build writes into the repository isn't owned by root. A
# terminal only when there is one: CI has none.
TERMINAL=()
[[ -t 0 ]] && TERMINAL=(--interactive --tty)

docker run --rm "${TERMINAL[@]}" \
    --user "$(id -u):$(id -g)" --env HOME=/tmp \
    --volume "$ROOT:/ws" --workdir /ws \
    "$IMAGE" "$@"
