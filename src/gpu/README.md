# GPU D-Bus Sensor Service

## Overview

The GPU D-Bus Sensor Service is a component of OpenBMC that provides temperature
monitoring capabilities for NVIDIA GPUs and other related devices. It exposes
GPU sensor data through D-Bus interfaces, allowing other OpenBMC components to
access and monitor GPU temperatures.

## Features

- Real-time GPU temperature monitoring
- D-Bus interface integration with OpenBMC
- OCP AMI OEM protocol-based communication with devices
- MCTP (Management Component Transport Protocol) based transport layer
- Asynchronous sensor data updates
- Comprehensive error handling with completion and reason codes

## Prerequisites

- OpenBMC environment
- Boost.Asio library for asynchronous I/O operations
- sdbusplus for D-Bus functionality
- phosphor-logging for logging capabilities
- MCTP support in the system

## Architecture

The application consists of several key components:

1. **Main Application (GpuMain.cpp)**

   - Initializes D-Bus connection and object server
   - Sets up MCTP communication
   - Creates sensor monitoring objects

2. **GPU Temperature Sensor (GpuTempSensor.hpp/cpp)**

   - Implements sensor functionality
   - Manages D-Bus interface for individual sensors
   - Handles threshold monitoring and checking
   - Performs device discovery and sensor polling

3. **MCTP Communication (MctpRequester.hpp/cpp)**

   - Provides asynchronous MCTP message handling
   - Implements request/response patterns for device communication
   - Handles socket-based communication with MCTP endpoints

4. **Protocol Implementation (GpuSensor.hpp/cpp)**
   - Defines OCP AMI OEM message types and structures
   - Implements device capability discovery commands
   - Handles temperature reading commands
   - Provides message encoding/decoding functions

## Protocol Details

### Message Types

The service uses OCP AMI protocol (type 0x7E) with the following message
categories:

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

The implementation includes comprehensive error handling with:

- Completion codes (OCP_AMI_SUCCESS, OCP_AMI_ERROR, etc.)
- Reason codes for detailed error information
- Message validation and bounds checking

### Asynchronous Operation

The service uses Boost.Asio for asynchronous operation:

- Non-blocking I/O for MCTP communication
- Timer-based sensor polling
- Event-driven architecture

## License

SPDX-License-Identifier: Apache-2.0

Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

## Contributing

Please follow OpenBMC's contribution guidelines when submitting patches or
improvements.
