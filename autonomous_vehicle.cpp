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

/*
 * Description:   Autonoumous vehicle controller example
 */

#include <webots/Camera.hpp>
#include <webots/Device.hpp>
#include <webots/Display.hpp>
#include <webots/GPS.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Lidar.hpp>
#include <webots/vehicle/Car.hpp>

#include <cmath>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

#include "webots_ros_publisher.hpp"

using namespace webots_ros_bridge;
using namespace webots;

ros::Publisher velodyne_pub;
ros::Publisher clock_pub;

// to be used as array indices
enum
{
  X,
  Y,
  Z
};

#define TIME_STEP 50
#define UNKNOWN 99999.99

// Line following PID
#define KP 0.25
#define KI 0.006
#define KD 2

bool PID_need_reset = false;

// Size of the yellow line angle filter
#define FILTER_SIZE 3

// enabe various 'features'
bool enable_collision_avoidance = false;
bool enable_display = false;
bool has_gps = false;
bool has_camera = false;
bool enable_velodyne = false;

Car *car;

// camera
Camera *camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;

// SICK laser
Lidar *sick;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;

//Velodyne
Lidar *velodyne;
double velodyne_max_range = -1;
int velodyne_cloud_size = -1;

// speedometer
Display *display;
int display_width = 0;
int display_height = 0;
ImageRef *speedometer_image;

// GPS
GPS *gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

Keyboard *keyboard;

// misc variables
double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool autodrive = true;

void print_help()
{
  std::cout << "You can drive this car!\n"
            << "Select the 3D window and then use the cursor keys to:\n"
            << "[LEFT]/[RIGHT] - steer\n"
            << "[UP]/[DOWN] - accelerate/slow down\n";
}

void set_autodrive(bool onoff)
{
  if (autodrive == onoff)
    return;
  autodrive = onoff;
  switch (autodrive)
  {
  case false:
    std::cout << "switching to manual drive...\n";
    std::cout << "hit [A] to return to auto-drive.\n";
    break;
  case true:
    if (has_camera)
      std::cout << "switching to auto-drive...\n";
    else
      std::cout << "impossible to switch auto-drive on without camera...\n";
    break;
  }
}

// set target speed
void set_speed(double kmh)
{
  // max speed
  if (kmh > 250.0)
    kmh = 250.0;

  speed = kmh;

  std::cout << "setting speed to " << std::fixed << std::setprecision(0) << kmh << " km/h\n";

  car->setCruisingSpeed(kmh);
}

// positive: turn right, negative: turn left
void set_steering_angle(double wheel_angle)
{
  // limit the difference with previous steering_angle
  if (wheel_angle - steering_angle > 0.1)
    wheel_angle = steering_angle + 0.1;
  if (wheel_angle - steering_angle < -0.1)
    wheel_angle = steering_angle - 0.1;
  steering_angle = wheel_angle;
  // limit range of the steering angle
  if (wheel_angle > 0.5)
    wheel_angle = 0.5;
  else if (wheel_angle < -0.5)
    wheel_angle = -0.5;
  car->setSteeringAngle(wheel_angle);
}

void change_manual_steer_angle(int inc)
{
  set_autodrive(false);

  double new_manual_steering = manual_steering + inc;
  if (new_manual_steering <= 25.0 && new_manual_steering >= -25.0)
  {
    manual_steering = new_manual_steering;
    set_steering_angle(manual_steering * 0.02);
  }

  if (manual_steering == 0)
    std::cout << "going straight\n";
  else
    std::cout << "turning %.2f rad (%s)\n", steering_angle, steering_angle < 0 ? "left" : "right";
}

void check_keyboard()
{
  int key = keyboard->getKey();

  switch (key)
  {
  case Keyboard::UP:
    set_speed(speed + 5.0);
    break;
  case Keyboard::DOWN:
    set_speed(speed - 5.0);
    break;
  case Keyboard::RIGHT:
    change_manual_steer_angle(+1);
    break;
  case Keyboard::LEFT:
    change_manual_steer_angle(-1);
    break;
  case 'A':
    set_autodrive(true);
    break;
  }
}

// compute rgb difference
int color_diff(const unsigned char a[3], const unsigned char b[3])
{
  int i, diff = 0;
  for (i = 0; i < 3; i++)
  {
    int d = a[i] - b[i];
    diff += d > 0 ? d : -d;
  }
  return diff;
}

// returns approximate angle of yellow road line
// or UNKNOWN if no pixel of yellow line visible
double process_camera_image(const unsigned char *image)
{
  int num_pixels = camera_height * camera_width; // number of pixels in the image
  const unsigned char REF[3] = {95, 187, 203};   // road yellow (BGR format)
  int sumx = 0;                                  // summed x position of pixels
  int pixel_count = 0;                           // yellow pixels count

  const unsigned char *pixel = image;
  int x;
  for (x = 0; x < num_pixels; x++, pixel += 4)
  {
    if (color_diff(pixel, REF) < 30)
    {
      sumx += x % camera_width;
      pixel_count++; // count yellow pixels
    }
  }

  // if no pixels was detected...
  if (pixel_count == 0)
    return UNKNOWN;

  return ((double)sumx / pixel_count / camera_width - 0.5) * camera_fov;
}

// filter angle of the yellow line (simple average)
double filter_angle(double new_value)
{
  static bool first_call = true;
  static double old_value[FILTER_SIZE];
  int i;

  if (first_call || new_value == UNKNOWN)
  { // reset all the old values to 0.0
    first_call = false;
    for (i = 0; i < FILTER_SIZE; ++i)
      old_value[i] = 0.0;
  }
  else
  { // shift old values
    for (i = 0; i < FILTER_SIZE - 1; ++i)
      old_value[i] = old_value[i + 1];
  }

  if (new_value == UNKNOWN)
    return UNKNOWN;
  else
  {
    old_value[FILTER_SIZE - 1] = new_value;
    double sum = 0.0;
    for (i = 0; i < FILTER_SIZE; ++i)
      sum += old_value[i];
    return (double)sum / FILTER_SIZE;
  }
}

// returns approximate angle of obstacle
// or UNKNOWN if no obstacle was detected
double process_sick_data(const float *sick_data, double *obstacle_dist)
{
  const int HALF_AREA = 20; // check 20 degrees wide middle area
  int sumx = 0;
  int collision_count = 0;
  int x;
  *obstacle_dist = 0.0;
  for (x = sick_width / 2 - HALF_AREA; x < sick_width / 2 + HALF_AREA; x++)
  {
    float range = sick_data[x];
    if (range < 20.0)
    {
      sumx += x;
      collision_count++;
      *obstacle_dist += range;
    }
  }

  // if no obstacle was detected...
  if (collision_count == 0)
    return UNKNOWN;

  *obstacle_dist = *obstacle_dist / collision_count;
  return ((double)sumx / collision_count / sick_width - 0.5) * sick_fov;
}

void update_display()
{
  const double NEEDLE_LENGTH = 50.0;

  // display background
  display->imagePaste(speedometer_image, 0, 0);

  // draw speedometer needle
  double current_speed = car->getCurrentSpeed();
  if (std::isnan(current_speed))
    current_speed = 0.0;
  double alpha = current_speed / 260.0 * 3.72 - 0.27;
  int x = -NEEDLE_LENGTH * cos(alpha);
  int y = -NEEDLE_LENGTH * sin(alpha);
  display->drawLine(100, 95, 100 + x, 95 + y);

  // draw text
  char txt[64];
  std::stringstream ss;
  ss << "GPS coords: " << std::setprecision(1) << std::fixed << gps_coords[X] << " " << gps_coords[Z];
  display->drawText(ss.str(), 10, 130);
  ss.str("");
  ss << "GPS speed: " << gps_speed;
  display->drawText(ss.str(), 10, 140);
}

void compute_gps_speed()
{
  const double *coords = gps->getValues();
  const double speed_ms = gps->getSpeed();
  // store into global variables
  gps_speed = speed_ms * 3.6; // convert from m/s to km/h
  std::copy(coords, coords + 3, gps_coords);
}

double applyPID(double yellow_line_angle)
{
  static double oldValue = 0.0;
  static double integral = 0.0;

  if (PID_need_reset)
  {
    oldValue = yellow_line_angle;
    integral = 0.0;
    PID_need_reset = false;
  }

  // anti-windup mechanism
  if (std::signbit(yellow_line_angle) != std::signbit(oldValue))
    integral = 0.0;

  double diff = yellow_line_angle - oldValue;

  // limit integral
  if (integral < 30 && integral > -30)
    integral += yellow_line_angle;

  oldValue = yellow_line_angle;
  return KP * yellow_line_angle + KI * integral + KD * diff;
}

void velodyne_process()
{
  const LidarPoint* cloud = velodyne->getPointCloud();

  WebotsRosPubisher::getInstance().publishPointCloud(cloud, velodyne_cloud_size, velodyne_max_range - 2);

}

int main(int argc, char **argv)
{

  ros::init(argc,argv,"controller");

  WebotsRosPubisher& ros_pub =  WebotsRosPubisher::getInstance();

  ros_pub.enableSimulationMode();

  Car car_instance;
  car = &car_instance;
  

  sick = car->getLidar("Sick LMS 291");
  velodyne = car->getLidar("Velodyne VLP-16");
  display = car->getDisplay("display");
  gps = car->getGPS("gps");
  camera = car->getCamera("camera");
  keyboard = car->getKeyboard();

  if (NULL != sick)
    enable_collision_avoidance = true;
  if(NULL != velodyne)
    enable_velodyne = true;
  if (NULL != display)
    enable_display = true;
  if (NULL != gps)
    has_gps = true;
  if (NULL != camera)
    has_camera = true;

  // camera device
  if (has_camera)
  {
    camera->enable(TIME_STEP);
    camera_width = camera->getWidth();
    camera_height = camera->getHeight();
    camera_fov = camera->getFov();
  }

  // SICK sensor
  if (enable_collision_avoidance)
  {
    sick->enable(TIME_STEP);
    sick_width = sick->getHorizontalResolution();
    sick_range = sick->getMaxRange();
    sick_fov = sick->getFov();
  }

  if(enable_velodyne)
  {
    velodyne->enable(TIME_STEP);
    velodyne->enablePointCloud();
    velodyne_max_range = velodyne->getMaxRange();
    velodyne_cloud_size = velodyne->getNumberOfPoints();
  }

  // initialize gps
  if (has_gps)
    gps->enable(TIME_STEP);

  // initialize display (speedometer)
  if (enable_display)
    speedometer_image = display->imageLoad("speedometer.png");

  // start engine
  if (has_camera)
    set_speed(20.0); // km/h
  car->setHazardFlashers(true);
  car->setDippedBeams(true);
  car->setAntifogLights(true);
  car->setWiperMode(Car::WiperMode::SLOW);

  print_help();

  // allow to switch to manual control
  keyboard->enable(TIME_STEP);

  // main loop
  while (car->step() != -1)
  {
    ros_pub.publishClock(car->getTime() * 1e6);
    ros_pub.publishSteeringAngle(car->getSteeringAngle());
    ros_pub.publishWheelEncoder(car->getWheelEncoder(Car::WheelIndex::WHEEL_FRONT_LEFT),
                                car->getWheelEncoder(Car::WheelIndex::WHEEL_FRONT_RIGHT),
                                car->getWheelEncoder(Car::WheelIndex::WHEEL_REAR_LEFT),
                                car->getWheelEncoder(Car::WheelIndex::WHEEL_REAR_RIGHT));
                               

    // get user input
    check_keyboard();
    static int i = 0;

    // updates sensors only every TIME_STEP milliseconds
    if (i % (int)(TIME_STEP / car->getBasicTimeStep()) == 0)
    {
      // read sensors

      const unsigned char *camera_image = NULL;
      const float *sick_data = NULL;
      if (has_camera)
        camera_image = camera->getImage();
      if (enable_collision_avoidance)
        sick_data = sick->getRangeImage();

      if (autodrive && has_camera)
      {
        double yellow_line_angle = filter_angle(process_camera_image(camera_image));
        double obstacle_dist;
        double obstacle_angle;
        if (enable_collision_avoidance)
          obstacle_angle = process_sick_data(sick_data, &obstacle_dist);

        // avoid obstacles and follow yellow line
        if (enable_collision_avoidance && obstacle_angle != UNKNOWN)
        {
          // an obstacle has been detected
          car->setBrakeIntensity(0.0);
          // compute the steering angle required to avoid the obstacle
          double obstacle_steering = steering_angle;
          if (obstacle_angle > 0.0 && obstacle_angle < 0.4)
            obstacle_steering = steering_angle + (obstacle_angle - 0.25) / obstacle_dist;
          else if (obstacle_angle > -0.4)
            obstacle_steering = steering_angle + (obstacle_angle + 0.25) / obstacle_dist;
          double steer = steering_angle;
          // if we see the line we determine the best steering angle to both avoid obstacle and follow the line
          if (yellow_line_angle != UNKNOWN)
          {
            const double line_following_steering = applyPID(yellow_line_angle);
            if (obstacle_steering > 0 && line_following_steering > 0)
              steer = obstacle_steering > line_following_steering ? obstacle_steering : line_following_steering;
            else if (obstacle_steering < 0 && line_following_steering < 0)
              steer = obstacle_steering < line_following_steering ? obstacle_steering : line_following_steering;
          }
          else
            PID_need_reset = true;
          // apply the computed required angle
          set_steering_angle(steer);
        }
        else if (yellow_line_angle != UNKNOWN)
        {
          // no obstacle has been detected, simply follow the line
          car->setBrakeIntensity(0.0);
          set_steering_angle(applyPID(yellow_line_angle));
        }
        else
        {
          // no obstacle has been detected but we lost the line => we brake and hope to find the line again
          car->setBrakeIntensity(0.4);
          PID_need_reset = true;
        }
      }

      // update stuff
      if (has_gps)
        compute_gps_speed();
      if (enable_display)
        update_display();
      if(enable_velodyne)
        velodyne_process();
    }

    ++i;
  }

  return 0; // ignored
}
