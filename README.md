# odrive_can_cpp_lib
## Overview

odrive_can_cpp_lib is a C++ library designed to facilitate communication with ODrive motor controllers using the CAN bus protocol. This library provides a convenient interface for developers to interact with ODrive devices, enabling control and monitoring of motor functions over a CAN network. For the protocol specification check [Odrive CAN protocol](https://docs.odriverobotics.com/v/0.5.6/can-protocol.html)
# Features

* Listening thread: each element generates a thread that constantly listen to thje messages being published in the bus, this allows it to get the heartbeat and position estimates messages that are peridically being sent by the odrive.
* Efficient Communication: Optimized for low-latency communication with ODrive devices over the CAN bus.
* Customizable: Flexible enough to accommodate various ODrive configurations and usage scenarios.
.

# Installation

To use odrive_can_cpp_lib in your project, simply clone the repository and include the necessary files in your build environment. Make sure to install any dependencies required by your development environment.

```
git clone https://github.com/srFelipes/odrive_can_cpp_lib.git
```

# Usage

    