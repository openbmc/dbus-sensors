/*
 * SPDX-FileCopyrightText: Copyright (c) 2022-2024. All rights
 * reserved. SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

namespace gpio_presence_sensing
{

namespace properties
{
static constexpr const char* propertyName = "Name";
static constexpr const char* propertyPolarity = "Polarity";
static constexpr const char* propertyPresent = "Present";
} // namespace properties

namespace interfaces
{
static constexpr const char* emConfigurationGPIOHWDetectInterface =
    "xyz.openbmc_project.Configuration.GPIODeviceDetect";
static constexpr const char* configurationGPIOHWDetectPresenceGpioInterface =
    "xyz.openbmc_project.Configuration.GPIODeviceDetect.PresenceGpio";
static constexpr const char* configurationGPIODeviceDetectedInterface =
    "xyz.openbmc_project.Configuration.GPIODeviceDetected";
} // namespace interfaces
} // namespace gpio_presence_sensing
