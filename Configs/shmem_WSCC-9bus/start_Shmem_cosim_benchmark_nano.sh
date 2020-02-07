#!/bin/bash

set -x

_stop() {
	echo "Caught SIGTSTP signal!"
	kill -TSTP ${CHILDS} 2>/dev/null
}

_cont() {
	echo "Caught SIGCONT signal!"
	kill -CONT ${CHILDS} 2>/dev/null
}

_term() {
	echo "Caught SIGTERM signal!"
	kill -TERM ${VN} ${CHILDS} 2>/dev/null
}

_kill() {
	echo "Caught SIGKILL signal!"
	kill -KILL ${VN} ${CHILDS} 2>/dev/null
}

trap _stop SIGTSTP
trap _cont SIGCONT
trap _term SIGTERM
trap _kill SIGKILL
trap _kill SIGINT

CHILDS=""

# Start time
TIME=$(date -d "+20 seconds" +%Y%m%dT%H%M%S) #-Iseconds
echo "Start simulation at: $TIME"

if [ "$1" = "--pipe" ]; then
	VILLAS_LOG_PREFIX="[Pipe] " \
	villas-pipe Configs/Shmem_cosim_dev.conf dpsim
else
	VILLAS_LOG_PREFIX="[Node] " \
	villas-node Configs/Shmem_cosim_dev_nano.conf & VN=$!
	#:
fi

# Wait until node is successfully started
sleep 2

CPS_LOG_PREFIX="[Sys ] " \
build/Examples/Cxx/Shmem_WSCC-9bus_cosim_benchmark #& P1=$!
CHILDS=$P1

sleep 2

pkill villas-node

# Wait until all child processed finished
# while (( $(ps --no-headers -o pid --ppid=$$ | wc -w) > 1 )); do
# 	wait
# done
