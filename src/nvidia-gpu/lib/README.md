# NVIDIA GPU Sensor Library

## Overview

This library provides the core functionality for NVIDIA GPU sensor communication
in OpenBMC using the OCP AMI (Open Compute Project Accelerator Management
Interface) protocol over MCTP (Management Component Transport Protocol). It
implements sensor data retrieval, message encoding/decoding, and transport layer
functionality for GPU temperature monitoring.

## Components

### Core Protocol Implementation

1. **OCP AMI Protocol (`ocp_ami.h/c`)**

   - Base protocol implementation for OCP AMI
   - Message structures and command definitions
   - Protocol-specific encoding/decoding functions
   - Error code definitions and handling

2. **NVIDIA Common (`nvidia_common.h/c`)**

   - Core message types and structures for OCP AMI communication
   - Message header packing/unpacking functionality
   - Common request/response encoding and decoding
   - Error code and reason code handling

3. **NVIDIA Sensors (`nvidia_sensors.h/c`)**
   - Temperature sensor reading functionality
   - Device identification and capability discovery
   - Sensor message structures and commands
   - Encoding/decoding of sensor-specific messages

### Transport Layer

1. **MCTP Transport Base (`transport.h/c`)**

   - Abstract transport layer interface
   - Message routing and handling primitives
   - Transport configuration structures
   - Common transport utilities

2. **AF_MCTP Implementation (`af-mctp.h/c`)**
   - MCTP socket implementation
   - Low-level communication interface
   - Message framing and transmission
   - Socket lifecycle management

## API Reference

### Device Management

```c
// Query device identification
int ocp_ami_oem_nvidia_encode_query_device_identification_req(
    uint8_t instance_id, struct ocp_ami_msg *msg);

// Decode device identification response
int ocp_ami_oem_nvidia_decode_query_device_identification_resp(
    const struct ocp_ami_msg *msg, size_t msg_len, uint8_t *cc,
    uint16_t *reason_code, uint8_t *device_identification,
    uint8_t *device_instance);
```

### Temperature Sensor API

```c
// Get temperature reading
int ocp_ami_oem_nvidia_encode_get_temperature_reading_req(
    uint8_t instance, uint8_t sensor_id, struct ocp_ami_msg *msg);

// Decode temperature reading
int ocp_ami_oem_nvidia_decode_get_temperature_reading_resp(
    const struct ocp_ami_msg *msg, size_t msg_len, uint8_t *cc,
    uint16_t *reason_code, double *temperature_reading);
```

### Transport API

```c
// Initialize MCTP transport
int mctp_transport_af_mctp_init(
    struct mctp_transport_af_mctp **ctx,
    uint8_t msg_type);

// Send/receive messages
ssize_t mctp_transport_af_mctp_send(
    struct mctp_transport_af_mctp *ctx,
    const void *buf, size_t len);
```
