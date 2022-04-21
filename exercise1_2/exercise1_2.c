/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define RANGE (1024 / 2)

static void compute_odometry(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);
  double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
  double dr = r * WHEEL_RADIUS;         // distance covered by right wheel in meter
  double da = (dr - dl) / AXLE_LENGTH;  // delta orientation
  printf("estimated distance covered by left wheel: %g m.\n", dl);
  printf("estimated distance covered by right wheel: %g m.\n", dr);
  printf("estimated change of orientation: %g rad.\n", da);
}

int main(int argc, char *argv[]) {
  /* define variables */
  WbDeviceTag distance_sensor[8], light_sensor[8], left_motor, right_motor, left_position_sensor, right_position_sensor;
  int i, j;
  double speed[2];
  double distance_sensors_value[8], light_sensors_value[8];

  int time_step;
  int camera_time_step;
  /* initialize Webots */
  wb_robot_init();

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    printf("e-puck2 robot\n");
    time_step = 64;
    camera_time_step = 64;
  } else {  // original e-puck
    printf("e-puck robot\n");
    time_step = 256;
    camera_time_step = 1024;
  }

  /* get and enable the camera and accelerometer */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, camera_time_step);
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* get a handler to the position sensors and enable them. */
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, time_step);
  wb_position_sensor_enable(right_position_sensor, time_step);

  for (i = 0; i < 8; i++) {
    char device_name[4];

    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], time_step);
  }

  for (i = 0; i < 8; i++) {
    char device_name[4];
    
    /* get light sensors */
    sprintf(device_name, "ls%d", i);
    light_sensor[i] = wb_robot_get_device(device_name);
    wb_light_sensor_enable(light_sensor[i], time_step);
  }

  /* main begin */
  speed[0] = 3.14;
  speed[1] = 2.60;
  
  while (wb_robot_step(time_step) != -1) {
    /* get sensors values */
    for (i = 0; i < 8; i++) {
      distance_sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
      light_sensors_value[i] = wb_light_sensor_get_value(light_sensor[i]);
    }
    const double *a = wb_accelerometer_get_values(accelerometer);
    printf("accelerometer values = %0.2f %0.2f %0.2f\n", a[0], a[1], a[2]);

    /* compute odometry and speed values*/
    
    compute_odometry(left_position_sensor, right_position_sensor);
    if (distance_sensors_value[0] > 100 ||
        distance_sensors_value[7] > 100) {
      speed[0] = -3.14;
      speed[1] = -2.60;
    }
    else if (distance_sensors_value[3] > 100 ||
             distance_sensors_value[4] > 100) {
      speed[0] = 3.14;
      speed[1] = 2.60;
    }

    /* set speed values */
    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }
  
  /* main end */

  wb_robot_cleanup();

  return 0;
}
