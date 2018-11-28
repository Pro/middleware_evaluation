# ROS Testing

## Run performance test scripts


### Idle Test

```bash
# Run on T2/Raspi
export ROS_IP=10.200.2.82 && export ROS_MASTER_URI=http://10.200.2.62:11311
rosrun ros_tests ros-perf-server

# Run on T1
roscore
rosrun ros_tests ros-perf-client

```

### CPU Load Test

First start stress test script on T2/Raspi

```bash
# T2:
stress --cpu 12 --io 4 --vm 2 --vm-bytes 128M
#Raspi:
stress --cpu 4 --io 4 --vm 2 --vm-bytes 128M

```

Then run the tests

```bash
# Run on T1
./ros-perf-client

# Run on T2/Raspi
./ros-perf-server
```

### Network Load Test

First start Ostinato on T2/Raspi and start the network load

Then run the tests

```bash
# Run on T1
./ros-perf-client

# Run on T2/Raspi
./ros-perf-server
```

## Testing multiple nodes

On T2/Raspberry start the server part

```bash
# Start mosquitto on T1
roscore
# Run on T2/Raspi
export ROS_IP=10.200.2.82 && export ROS_MASTER_URI=http://10.200.2.62:11311
testing/ros/run_multiple.sh 1000 fwd_50000
# Run on T1
rosrun ros_tests ros-multi-client _topic_self:=/fwd_50000 _topic_forward_prefix:=/fwd_ _topic_forward_idx:=10001

```