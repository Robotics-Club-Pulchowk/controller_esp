#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "errno.h"
#include "driver/gpio.h"

#include "usb/hid_host.h"

/**
 * @brief APP event group
 *
 * Application logic can be different. There is a one among other ways to distingiush the
 * event by application event group.
 * In this example we have two event groups:
 * APP_EVENT            - General event, which is APP_QUIT_PIN press event (Generally, it is IO0).
 * APP_EVENT_HID_HOST   - HID Host Driver event, such as device connection/disconnection or input report.
 */
typedef enum
{
  APP_EVENT = 0,
  APP_EVENT_HID_HOST
} app_event_group_t;

/**
 * @brief APP event queue
 *
 * This event is used for delivering the HID Host event from callback to a task.
 */
typedef struct
{
  app_event_group_t event_group;
  /* HID Host - Device related info */
  struct
  {
    hid_host_device_handle_t handle;
    hid_host_driver_event_t event;
    void *arg;
  } hid_host_device;
} app_event_queue_t;

/**
 * @brief PS4 data type
 * 
 * This datatype is used in the DS4 controller
 */
typedef struct
{
  uint8_t id;
  int8_t lx,
      ly,
      rx,
      ry;
  union
  {
    struct
    {
      uint8_t dpad : 4;
      uint8_t square : 1;
      uint8_t cross : 1;
      uint8_t circle : 1;
      uint8_t triangle : 1;

      uint8_t l1 : 1;
      uint8_t r1 : 1;
      uint8_t l2 : 1;
      uint8_t r2 : 1;
      uint8_t share : 1;
      uint8_t options : 1;
      uint8_t l3 : 1;
      uint8_t r3 : 1;

      uint8_t ps : 1;
      uint8_t touchpad : 1;
      uint8_t reportCounter : 6;
    } __attribute__((packed));
    uint32_t val : 24;
  } __attribute__((packed)) buttons;
  uint8_t lt,
      rt;
} ps4_data_t;
