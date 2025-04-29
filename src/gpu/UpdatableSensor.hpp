/*
 * SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION &
 * AFFILIATES. All rights reserved. SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "sensor.hpp"

class GpuSensor : public Sensor
{
  public:
    using Sensor::Sensor;

    virtual void update() = 0;
};
