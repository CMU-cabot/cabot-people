#!/bin/bash


trap ctrl_c INT QUIT TERM

function ctrl_c() {
    docker compose -f docker-compose-test-rs3.yaml down > /dev/null 2>&1
    exit
}
function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
cd $scriptdir/../

# reset realsense
docker compose -f docker-compose-test-rs3.yaml run --rm rs1 sudo /resetrs.sh

# launch detection and tracking, and check if it output any ERROR logs
(timeout 20s docker compose -f docker-compose-test-rs3.yaml up rs1 track 2>&1) | while IFS= read -r line
do
    echo "$line"  # Optional: Echoes the output of the command
    if [[ "$line" == *"ERROR"* ]]; then
        echo "Error detected, aborting the command."
	docker compose -f docker-compose-test-rs3.yaml down 2>&1
	exit 1
    fi
done

docker compose -f docker-compose-test-rs3.yaml down 2>&1
