#!/usr/bin/env bash

if [ "$#" -ne 3 ]; then
    echo "Usage: run_multiple.sh NUMBER_OF_NODES FINAL_ENDPOINT FINAL_PORT"
    exit 1
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

BUILD_DIR=$DIR/../../build/testing/udp/tests


pid_list=()
run=1
max_nodes=$1
final_endpoint=$2
final_port=$3
start_port=10000

pkill -f -9 "udp-multi-server"

run_program () {
    port=$(($start_port + $1))
    port_fwd=$(($port + 10))

    endpoint=""
    if [ $port_fwd -lt $(($start_port + $max_nodes)) ]; then
        endpoint="localhost"
    else
        endpoint=$final_endpoint
        port_fwd=$final_port
    fi

   echo "Starting node $1 with port $port"
   $BUILD_DIR/udp-multi-server "$port" "$endpoint" "$port_fwd" &

   pid_list=("${pid_list[@]}" $!)
}

trap ctrl_c INT

function ctrl_c() {
    echo -e "** Closing all child processes\n"
    for i in "${pid_list[@]}"; do
        echo -e "*** Killing PID $i\n"
        kill -SIGINT $i
    done
    echo -e "** All children killed\n"
    run=0
}



counter=1
while [ $counter -le $max_nodes ] && [ $run -eq 1 ]; do
    run_program $counter
    ((counter++))

    #if ! ((counter % 10)); then
    #    sleep 1
    #fi
done

echo "All instances started. Kill with Ctrl+C"

while [ $run -eq 1 ]; do
    sleep 1
done
