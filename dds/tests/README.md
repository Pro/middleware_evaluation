# DDS Testing

## Run performance test scripts


### Idle Test

```bash
# Run on T1
./dds-perf-client

# Run on T2/Raspi
./dds-perf-server
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
./dds-perf-client

# Run on T2/Raspi
./dds-perf-server
```

### Network Load Test

First start Ostinato on T2/Raspi and start the network load

Then run the tests

```bash
# Run on T1
./dds-perf-client

# Run on T2/Raspi
./dds-perf-server
```

## Testing multiple nodes

On T2/Raspberry start the server part

```bash
# Run on T2/Raspi
testing/dds/run_multiple.sh 1000 fwd_50000
# Run on T1
./dds-multi-client fwd_50000 fwd_ 10001

```