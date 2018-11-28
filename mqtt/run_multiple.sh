#!/usr/bin/env bash

if [ "$#" -ne 3 ]; then
    echo "Usage: run_multiple.sh MQTT_BROKER NUMBER_OF_NODES FINAL_ENDPOINT"
    exit 1
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

BUILD_DIR=$DIR/../../build/testing/mqtt/tests


pid_list=()
run=1
mqtt_broker=$1
max_nodes=$2
final_topic=$3
start_port=10000

pkill -f -9 "mqtt-multi-server"

run_program () {
    port=$(($start_port + $1))
    port_fwd=$(($port + 10))

    endpoint=""
    if [ $port_fwd -lt $(($start_port + $max_nodes)) ]; then
        endpoint="fwd_${port_fwd}"
    else
        endpoint=$final_topic
    fi

   echo "Starting node $1 with port $port"
   $BUILD_DIR/mqtt-multi-server $mqtt_broker "fwd_$port" "$endpoint" &

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
done

echo "All instances started. Kill with Ctrl+C"

while [ $run -eq 1 ]; do
    sleep 1
done