#!/usr/bin/env bash

if [ "$#" -ne 4 ]; then
    echo "Usage: run_multiple.sh DDS_DCPS NUMBER_OF_NODES FINAL_ENDPOINT [RTPS|SHM]"
    exit 1
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

BUILD_DIR=$DIR/../../build/testing/dds/tests


pid_list=()
run=1
dds_DCPS=$1
max_nodes=$2
final_topic=$3
transport=$4

if [ $transport != "RTPS" ] && [ $transport != "SHM" ]; then
    echo "Transport must be either RTPS or SHM"
    exit 1
fi

transport_rtps=1

if [ $transport = "SHM" ]; then
    transport_rtps=0
fi

start_port=10000

pkill -f -9 "dds-multi-server"

run_program () {
    port=$(($start_port + $1))
    port_fwd=$(($port + 10))

    rtps_sub=1
    rtps_pub=1
    endpoint=""


    if [ $port -gt $(($start_port + 10)) ]; then
        rtps_sub=$transport_rtps;
    fi

    if [ $port_fwd -lt $(($start_port + $max_nodes)) ]; then
        endpoint="fwd_${port_fwd}"
        rtps_pub=$transport_rtps
    else
        endpoint=$final_topic
    fi

   echo "Starting node $1 with port $port"
   $BUILD_DIR/dds-multi-server -DCPSBit 0 -DCPSInfoRepo "$dds_DCPS" -s "fwd_$port" -f "$endpoint" -x $rtps_pub -y $rtps_sub &

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

    # DDS requires a lot of CPU when starting up. So only start 100 nodes per batch
    if ! ((counter % 50)); then
        sleep 5
    fi
done

echo "All instances started. Kill with Ctrl+C"

while [ $run -eq 1 ]; do
    sleep 1
done