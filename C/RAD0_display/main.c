// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int32.h>

#include <stdio.h>

#define CUSTOM_ASSERT(ptr) if ((ptr) == NULL) {return -1;}

static float altitude;
static uint32_t engine_power;
static unsigned it;
static char Alert[200];

/**
 * @brief Update screen
 *
 */
void UpdateDisplay()
{
  printf("\r[received messages: %7u]    [Altitude: %8.2f]    [Engine power: %8u]    [Status: %10s]",
    it++, altitude, engine_power, Alert);
}


/**
 * @brief new alert callback
 *
 * @param msgin
 */
void on_alert_message(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  strcpy(Alert, msg->data.data); // NOLINT

  UpdateDisplay();
}


/**
 * @brief new altitude callback
 *
 * @param msgin
 */
void on_altitude_message(const void * msgin)
{
  std_msgs__msg__Float64 * msg = (std_msgs__msg__Float64 *)msgin;

  altitude = msg->data;
  UpdateDisplay();
}

/**
 * @brief
 *
 * @param msgin
 */
void on_power_message(const void * msgin)
{
  std_msgs__msg__UInt32 * msg = (std_msgs__msg__UInt32 *)msgin;

  engine_power = msg->data;
  UpdateDisplay();
}


/**
 * @brief Main
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;

  rclc_node_t * node = NULL;
  rclc_subscription_t * alert_subscription = NULL;
  rclc_subscription_t * altitude_subscription = NULL;
  rclc_subscription_t * power_subscription = NULL;

  rclc_ret_t ret;

  ret = rclc_init(0, NULL);
  if (ret != RCL_RET_OK) {
    return -1;
  }

  node = rclc_create_node("rad0_display_c", "");
  CUSTOM_ASSERT(node);
  alert_subscription =
    rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg,
      String), "std_msgs_msg_String", on_alert_message, 1,
      false);
  CUSTOM_ASSERT(alert_subscription);
  altitude_subscription =
    rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg,
      Float64), "std_msgs_msg_Float64", on_altitude_message, 1,
      false);
  CUSTOM_ASSERT(altitude_subscription);
  power_subscription =
    rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg,
      UInt32), "std_msgs_msg_UInt32", on_power_message, 1,
      false);
  CUSTOM_ASSERT(power_subscription);

  rclc_spin_node(node);

  if (alert_subscription) {ret = rclc_destroy_subscription(alert_subscription);}
  if (altitude_subscription) {ret = rclc_destroy_subscription(altitude_subscription);}
  if (node) {ret = rclc_destroy_node(node);}

  printf("Display node closed.");
}
