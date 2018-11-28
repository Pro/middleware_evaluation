#!/usr/bin/env bash

if [ "$#" -ne 2 ]; then
    echo "Usage: run_multiple.sh NUMBER_OF_NODES FINAL_TOPIC"
    exit 1
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

BUILD_DIR=$DIR/../../build/testing/ros/tests


pid_list=()
run=1
max_nodes=$1
final_topic=$2
start_port=10000

pkill -f -9 "ros-multi-server"

run_program () {
    port=$(($start_port + $1))
    port_fwd=$(($port + 10))

    topic_self="fwd_${port}"

    topic_fwd=""
    if [ ${port_fwd} -lt $(($start_port + $max_nodes)) ]; then
        topic_fwd="fwd_${port_fwd}"
    else
        topic_fwd=${final_topic}
    fi

   echo "Starting node $1 with topid $topic_self"
   rosrun ros_tests ros-multi-server __name:="multi_forward_node_$1" _topic_self:="/${topic_self}" _topic_forward:="/${topic_fwd}" &

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

    # ROS requires a lot of CPU when starting up. So only start 100 nodes per batch
    if ! ((counter % 100)); then
        sleep 2
    fi
done

echo "All instances started. Kill with Ctrl+C"

while [ $run -eq 1 ]; do
    sleep 1
done