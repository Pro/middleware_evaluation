#!/usr/bin/env bash

if [ "$#" -ne 1 ]; then
    echo "Usage: run_multiple.sh PROTOCOL"
    exit 1
fi

today=`date '+%Y_%m_%d_%H_%M_%S'`;
stats_file="stat_$1_$today.txt"

pids=$(ps -C "$1-multi-server" -o pid)

for i in $pids; do
  if [ $i = "PID" ]; then
	continue;
  fi
  cat /proc/$i/stat >> $stats_file
done

echo "Data collected in $stats_file"
