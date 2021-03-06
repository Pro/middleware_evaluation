# Middleware Performance Evaluation

Performance tests for performance evaluation of various middleware implementations (OPC UA, MQTT, DDS, ROS).

The results can be found in the linked paper: https://mediatum.ub.tum.de/node?id=1470362


**If you are using this test suite, please cite our paper:**

Stefan Profanter, Ayhun Tekat, Kirill Dorofeev and Markus Rickert. OPC UA versus ROS, DDS, and MQTT: Performance Evaluation of Industry 4.0 Protocols. In Proceedings of the IEEE International Conference on Industrial Technology (ICIT), Melbourne, Australia, 2019.


```latex
@inproceedings{Profanter2019,
    author = {Profanter, Stefan and Tekat, Ayhun and Dorofeev, Kirill and Rickert, Markus},
    title = {OPC UA versus ROS, DDS, and MQTT: Performance Evaluation of Industry 4.0 Protocols},
    booktitle = {Proceedings of the {IEEE} International Conference on Industrial Technology ({ICIT})},
    year = {2018},
    month = feb,
    address = {Melbourne, Australia}
}
```

## Building

To build with Eprosima FastRTPS, make sure that you are calling CMake with `-DTHIRDPARTY=ON`:

```sh
cd middleware_evaluation
git submodule update --init --recursive
mkdir build && cd build
cmake -DTHIRDPARTY=ON ..
make -j
```

## Running the tests

Check the corresponding subfolders for each protocol to see how to start the tests
