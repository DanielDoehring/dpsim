#!/bin/bash

###########################################################################################
# Configuration parameters
###########################################################################################

TYPE="nano"                         # Communication pattern: nano/mqtt
DISTAIX_PATH="distaix"              # Path to distaix main dir
DPSIM_PATH="dpsim"                  # Path to dpsim main dir
DOCKER_CONTAINER_NAME="bold_knuth"  # Name of dpsim_dev docker container
LOG_FILES_PATH="./benchmarks"       # Directory where results should be stored
DIST_NUM_OF_PROCESSES=4             # Number of processes invoked by distaix
NUMBER_OF_LOOPS=0                   # Number of loops that should be executed

###########################################################################################
###########################################################################################

# Set paths to executables...
DISTAIX_EXEC="$DISTAIX_PATH/bin/run.sh -n$DIST_NUM_OF_PROCESSES"
DPSIM_EXEC="docker exec -w /dpsim $DOCKER_CONTAINER_NAME ./Configs/shmem_WSCC-9bus/start_Shmem_cosim_benchmark_$TYPE.sh"

# Parse optional script arguments
if [ "$#" -eq 0 ]; then
    echo "No parameters given -> Default parameters are used..."
    echo "Usage: ./start_benchmark.sh <numberOfLoops=0> <dockerContainerName=bold_knuth>" 
elif [ "$#" -eq 1 ]; then 
    NUMBER_OF_LOOPS=$1
else
    NUMBER_OF_LOOPS=$1
    DOCKER_CONTAINER_NAME=$2
fi

# Print used configuration
echo "###############################################################"
echo "###############################################################"
echo "Configuration:"
echo -e "\tNumber of loops: \t\t$NUMBER_OF_LOOPS"
echo -e "\tType: \t\t\t\t$TYPE"
echo -e "\tDistAIX path: \t\t\t$DISTAIX_PATH"
echo -e "\tDPsim path: \t\t\t$DPSIM_PATH"
echo -e "\tDocker container: \t\t$DOCKER_CONTAINER_NAME"
echo -e "\tLog files path: \t\t$LOG_FILES_PATH"
echo -e "\tNumber of DistAIX processes: \t$DIST_NUM_OF_PROCESSES"
echo "###############################################################"
echo "###############################################################"

#sleep 5

# Create folder for results of current session
# remove whitespaces that are somehow introduced by 'date'...
LOG_FILES_PATH_SESSION="$(echo -e $LOG_FILES_PATH/$(date +'%d-%m-%y_%r')_$TYPE | tr -d '[:space:]')"
mkdir -v -p "$LOG_FILES_PATH_SESSION"
touch "$LOG_FILES_PATH_SESSION/summary.csv"

for ((i=1; i<=$NUMBER_OF_LOOPS; i++)); do
    # Execute simulations
    $DISTAIX_EXEC&
    $DPSIM_EXEC

    # Wait for everything to terminate...
    sleep 1

    # Save results of both simulations
    cp $DPSIM_PATH/logs/Shmem_WSCC-9bus_cosim_benchmark/Shmem_WSCC-9bus_cosim_benchmark.log $LOG_FILES_PATH_SESSION/$i.log
    cp $DISTAIX_PATH/bin/results/rank0.csv $LOG_FILES_PATH_SESSION/$i.csv

    # Parse simulation time from dpsim log file and add it to summary.csv
    VAL=$(grep -n 'Simulation' $LOG_FILES_PATH_SESSION/$i.log | awk -F ': ' '{print $2}')
    echo -e "$i,$VAL" >> "$LOG_FILES_PATH_SESSION/summary.csv"
    
done