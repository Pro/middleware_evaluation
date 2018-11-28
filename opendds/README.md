# DDS Testing

You need to start the DDS Server on T1:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DDS_ROOT/lib
$DDS_ROOT/bin/DCPSInfoRepo -NOBITS -ORBListenEndpoints iiop://:12345
```

And make sure that `$TEST_REPO` points to the root of this repo

## Run performance test scripts

### Idle Test

```bash
# Run on T1
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DDS_ROOT/lib
sudo ./dds-perf-client -DCPSBit 0 -DCPSInfoRepo cobot-t1-main:12345 -DCPSConfigFile $TEST_REPO/testing/dds/tests/config/rtps_uni.ini

# Run on T2/Raspi
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DDS_ROOT/lib
sudo ./dds-perf-server -DCPSBit 0 -DCPSInfoRepo cobot-t1-main:12345 -DCPSConfigFile $TEST_REPO/testing/dds/tests/config/rtps_uni.ini
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
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DDS_ROOT/lib
sudo ./dds-perf-client -DCPSBit 0 -DCPSInfoRepo cobot-t1-main:12345 -DCPSConfigFile $TEST_REPO/testing/dds/tests/config/rtps_uni.ini

# Run on T2/Raspi
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DDS_ROOT/lib
sudo ./dds-perf-server -DCPSBit 0 -DCPSInfoRepo cobot-t1-main:12345 -DCPSConfigFile $TEST_REPO/testing/dds/tests/config/rtps_uni.ini
```

### Network Load Test

First start Ostinato on T2/Raspi and start the network load

Then run the tests

```bash
# Run on T1
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DDS_ROOT/lib
sudo ./dds-perf-client -DCPSBit 0 -DCPSInfoRepo cobot-t1-main:12345 -DCPSConfigFile $TEST_REPO/testing/dds/tests/config/rtps_uni.ini

# Run on T2/Raspi
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DDS_ROOT/lib
sudo ./dds-perf-server -DCPSBit 0 -DCPSInfoRepo cobot-t1-main:12345 -DCPSConfigFile $TEST_REPO/testing/dds/tests/config/rtps_uni.ini
```

## Testing multiple nodes

On T2/Raspberry start the server part

```bash
# Start core on T1

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DDS_ROOT/lib
$DDS_ROOT/bin/DCPSInfoRepo -NOBITS -ORBListenEndpoints iiop://:12345

# Run on T2/Raspi for normal RTPS:
sudo testing/dds/run_multiple.sh cobot-t1-main:12345 1000 fwd_50000 RTPS

# Run on T2/Raspi for Shared Memory Test:
sudo testing/dds/run_multiple.sh cobot-t1-main:12345 1000 fwd_50000 SHM

# Run on T1
sudo ./dds-multi-client -DCPSBit 0 -DCPSInfoRepo cobot-t1-main:12345 -DCPSConfigFile $TEST_REPO/testing/dds/tests/config/rtps_uni.ini -s fwd_50000 -f fwd_ -i 10001

```
