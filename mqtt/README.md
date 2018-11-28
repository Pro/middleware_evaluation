# MQTT Testing

## Run performance test scripts


### Idle Test

```bash
# Run on T1
./mqtt-perf-client

# Run on T2/Raspi
./mqtt-perf-server
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
./mqtt-perf-client

# Run on T2/Raspi
./mqtt-perf-server
```

### Network Load Test

First start Ostinato on T2/Raspi and start the network load

Then run the tests

```bash
# Run on T1
./mqtt-perf-client

# Run on T2/Raspi
./mqtt-perf-server
```

## Testing multiple nodes

On T2/Raspberry start the server part

```bash
# Start mosquitto on T1
mosquitto
# Run on T2/Raspi
testing/mqtt/run_multiple.sh tcp://cobot-t1-main:1883 1000 fwd_50000
# Run on T1
./mqtt-multi-client tcp://cobot-t1-main:1883 fwd_50000 fwd_ 10001

```