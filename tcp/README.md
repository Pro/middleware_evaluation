# TCP Testing

## Run performance test scripts


### Idle Test

```bash
# Run on T1
./tcp-perf-client

# Run on T2/Raspi
./tcp-perf-server
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
./tcp-perf-client

# Run on T2/Raspi
./tcp-perf-server
```

### Network Load Test

First start Ostinato on T2/Raspi and start the network load

Then run the tests

```bash
# Run on T1
./tcp-perf-client

# Run on T2/Raspi
./tcp-perf-server
```

## Testing multiple nodes

On T2/Raspberry start the server part

```bash
# Run on T2/Raspi
testing/tcp/run_multiple.sh 1000 localhost 50000
# Run on T1
./tcp-multi-client 50000 10001 cobot-t2-main

```