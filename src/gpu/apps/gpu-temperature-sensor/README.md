# GPU D-Bus Sensor Service

## Overview

The GPU D-Bus Sensor Service is a component of OpenBMC that provides temperature
monitoring capabilities for GPUs. It exposes GPU sensor data through D-Bus
interfaces, allowing other OpenBMC components to access and monitor GPU
temperatures.

## Features

- Real-time GPU temperature monitoring
- D-Bus interface integration with OpenBMC
- OCP AMI OEM protocol-based communication with devices
- MCTP (Management Component Transport Protocol) based transport layer
- Asynchronous sensor data updates
- Support for sensor associations and relationships
- Comprehensive error handling with reason codes

## Prerequisites

- OpenBMC environment
- Boost.Asio library
- sdbusplus
- phosphor-logging
- MCTP support in the system

## Building

This application is built using the Meson build system. To build:

```bash
meson builddir
cd builddir
ninja
```

## Installation

The service will be installed as part of the OpenBMC image build process. The
binary will be installed in the appropriate system directory.

## Architecture

The application consists of several key components:

1. **Main Application (GpuMain.cpp)**

   - Initializes D-Bus connection
   - Sets up sensor monitoring

2. **GPU Sensor Task (GpuSensorTask.hpp/cpp)**

   - Manages sensor monitoring operations
   - Handles periodic updates
   - Implements device discovery and identification

3. **Temperature Sensor (GpuTempSensor.hpp/cpp)**

   - Implements sensor functionality
   - Manages D-Bus interface for individual sensors

4. **MCTP Communication (MctpRequester.hpp/cpp)**

   - Handles MCTP-based communication with devices
   - Implements request/response handling

5. **Protocol Implementation (gpu_common.h, gpu_sensors.h)**
   - Defines OCP AMI OEM message types and structures
   - Implements device capability discovery commands
   - Handles temperature reading commands
   - Provides message encoding/decoding functions

## Protocol Details

### Message Types

- Device Capability Discovery (Type 0)
- Platform Environmental Monitoring (Type 3)

### Supported Commands

1. Device Capability Discovery:
   - Query Device Identification (0x09)
2. Platform Environmental:
   - Get Temperature Reading (0x00)

### Device Types

The service supports identification and monitoring of:

- GPU devices (ID: 0x00)
- Network switches (ID: 0x01)
- PCIe bridges (ID: 0x02)
- Baseboards (ID: 0x03)
- EROT devices (ID: 0x04)
- Unknown devices (ID: 0xFF)

## D-Bus Interface

### Service Name

- `xyz.openbmc_project.GpuSensor`

### Object Paths

- Base path: `/xyz/openbmc_project/sensors`
- Individual sensors will be created under this path

### Interfaces

1. `xyz.openbmc_project.Sensor.Value`
   - Used for reporting temperature values
2. `xyz.openbmc_project.Association.Definitions`
   - Used for sensor relationships and associations

## Development

### Error Handling

- The implementation includes comprehensive error handling with:
  - Completion codes
  - Reason codes for detailed error information
  - Message validation and bounds checking

## License

SPDX-License-Identifier: Apache-2.0

Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

## Contributing

Please follow OpenBMC's contribution guidelines when submitting patches or
improvements.
