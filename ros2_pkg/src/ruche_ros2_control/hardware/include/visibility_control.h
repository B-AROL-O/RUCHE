// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef ELEGOO_BT2SERIAL__VISIBILITY_CONTROL_H_
#define ELEGOO_BT2SERIAL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ELEGOO_BT2SERIAL_EXPORT __attribute__((dllexport))
#define ELEGOO_BT2SERIAL_IMPORT __attribute__((dllimport))
#else
#define ELEGOO_BT2SERIAL_EXPORT __declspec(dllexport)
#define ELEGOO_BT2SERIAL_IMPORT __declspec(dllimport)
#endif
#ifdef ELEGOO_BT2SERIAL_BUILDING_DLL
#define ELEGOO_BT2SERIAL_PUBLIC ELEGOO_BT2SERIAL_EXPORT
#else
#define ELEGOO_BT2SERIAL_PUBLIC ELEGOO_BT2SERIAL_IMPORT
#endif
#define ELEGOO_BT2SERIAL_PUBLIC_TYPE ELEGOO_BT2SERIAL_PUBLIC
#define ELEGOO_BT2SERIAL_LOCAL
#else
#define ELEGOO_BT2SERIAL_EXPORT __attribute__((visibility("default")))
#define ELEGOO_BT2SERIAL_IMPORT
#if __GNUC__ >= 4
#define ELEGOO_BT2SERIAL_PUBLIC __attribute__((visibility("default")))
#define ELEGOO_BT2SERIAL_LOCAL __attribute__((visibility("hidden")))
#else
#define ELEGOO_BT2SERIAL_PUBLIC
#define ELEGOO_BT2SERIAL_LOCAL
#endif
#define ELEGOO_BT2SERIAL_PUBLIC_TYPE
#endif

#endif  // ELEGOO_BT2SERIAL__VISIBILITY_CONTROL_H_