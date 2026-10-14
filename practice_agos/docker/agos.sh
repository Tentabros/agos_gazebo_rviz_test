#!/usr/bin/env bash
# Kept so older notes still work: the ./agos script in the folder above now
# does everything, on Docker and on a normal Ubuntu install alike.
#
#   docker/agos.sh           same as  ./agos shell   (a terminal in the container)
#   docker/agos.sh --reset   same as  ./agos reset, then ./agos shell
#
# Easier: ./agos start, ./agos explore, ./agos help.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export AGOS_DOCKER=1
if [ "${1:-}" = "--reset" ]; then
  "$WS/agos" reset || exit 1
  shift
fi
exec "$WS/agos" shell "$@"
