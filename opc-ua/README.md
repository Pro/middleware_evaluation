# OPC UA Testing

## Run performance test scripts


### Idle Test

```bash
# Run on T1
./opcua-perf-client

# Run on T2/Raspi
./opcua-perf-server
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
./opcua-perf-client

# Run on T2/Raspi
./opcua-perf-server
```

### Network Load Test

First start Ostinato on T2/Raspi and start the network load

Then run the tests

```bash
# Run on T1
./opcua-perf-client

# Run on T2/Raspi
./opcua-perf-server
```

## Testing multiple nodes

On T2/Raspberry start the server part

```bash
# Run on T2/Raspi
testing/opc-ua/run_multiple.sh 1000 opc.tcp://cobot-t1-main:50000
# Run on T1
./opcua-multi-client 50000 10001 opc.tcp://cobot-t2-main

```


For Pub/Sub

```bash
# Run on T2/Raspi
testing/opc-ua/run_multiple_pubsub.sh 1000 opc.udp://224.0.0.22:50000
# Run on T1
./opcua-multi-ps-client 50000 10001 opc.udp://224.0.0.22

```