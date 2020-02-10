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

CHILDS=""

# Start time
TIME=$(date -d "+10 seconds" +%Y%m%dT%H%M%S) #-Iseconds
echo "Start simulation at: $TIME"

# Simulation params
OPTS="--timestep 0.001 --duration $((200)) --system-freq 50 --start-at $TIME --solver-domain DP --solver-type MNA --log-level NONE"
echo "Simulation params: $OPTS"

CPS_LOG_PREFIX="[Sys ] " \
build/Examples/Cxx/Shmem_CIGRE_MV_Feeder1_CoVeeCtrl $(pwd)/Examples/CIM/grid-data/CIGRE_MV_NoTap_Feeder1_only/*.xml \
	$(pwd)/Examples/CSV/*.csv $(pwd)/Examples/CSV/CIGRE_MV_NoTap_Feeder1/*.csv $OPTS & P1=$!

CHILDS=$P1

sleep 2

if true; then
	VILLAS_LOG_PREFIX="[Pipe] " \
	#villas-pipe Configs/Shmem_CIGRE_MV_CoVeeCtrl.conf dpsim1
	villas-node Configs/Shmem_CIGRE_MV_CoVeeCtrl.conf
else
	VILLAS_LOG_PREFIX="[Node] " \
	villas-node Configs/Shmem_CIGRE_MV_CoVeeCtrl.conf & VN=$!
fi

# Wait until all child processed finished
while (( $(ps --no-headers -o pid --ppid=$$ | wc -w) > 1 )); do
	wait
done
