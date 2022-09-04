/*
 * Copyright 1996-2021 Cyberbotics Ltd.
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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/display.h>

////////// Vehicle Params //////////
#define MOOSE_WHEEL_DIAMETER 0.635  // wheel diameter in meters [m] 

////////// For Moose path following //////////
#define TIME_STEP 16
#define MAXIMUM_NUMBER_OF_COORDINATES 8000  // Max size of the history.
#define DISTANCE_TOLERANCE 6.5  // (default = 1.5)
// #define MAX_SPEED 7.0   // in radians (default = 7.0): Note that 26 radians with 25" diameter wheels is ~30km/h which is vehicle top speed
#define TURN_COEFFICIENT 7.0   // (default = 4.0) --> 6.5

// Placeholder globals to update key metrics – will use to print to console
char TERRAIN [50] = "Open_Area";    // string of terrain type
double MAX_SPEED = 7.0;             // rad/s
double KMPH = 0;                    // Km/h
double DIST = 0;                    // m
int WAYPOINTS = 0;                  // number of waypoints in route

enum XYZAComponents { X = 0, Y, Z, ALPHA };
enum Sides { LEFT, RIGHT };

// vector contains du, dv coordinates for waypoint navigation
typedef struct _Vector {
  double u;
  double v;
} Vector;

// custom struct to hold RGB values for downward facing camera on moose. Note: it can
// store corresponding velocities for colours (which are used for terrain classes)
typedef struct _TerrainRGB {
  char trn[50];
  int r;
  int g;
  int b;
  double vel;
} TerrainRGB;

// structure to contain all datatypes from vehicle config files
typedef struct _Config {
  Vector* wpts;
  TerrainRGB* trns;
} Config;

// camera globals
int image_width = 2;       // pixel value image width of cameraDown on moose vehicle
int image_height = 2;      // pixel value image height of cameraDown on moose vehicle
int terrain_classes = 0;    // number of terrain classes to check through
const unsigned char *image; //store image pointer here

static WbDeviceTag cameraFront, cameraDown;
static WbDeviceTag motors[8];
static WbDeviceTag gps;
static WbDeviceTag compass;

static int current_target_index = 0;
static bool autopilot = true;
static bool old_autopilot = true;
static int old_key = -1;
bool first = true;
float displacement = 0.0;

// get path to Webots config files
static char* get_config_path( ){
  static char cwd[400];
  char * pch;
  
  // get current working directory
  getcwd(cwd, 400);
  
  // get directory of top level repo directory
  pch=strrchr(cwd, '/');
  cwd[pch-24-cwd] = '\0';
  return cwd;
}

static double modulus_double(double a, double m) {
  const int div = (int)(a / m);
  double r = a - div * m;
  if (r < 0.0)
    r += m;
  return r;
}

// set left and right motor speed [rad/s]
static void robot_set_speed(double left, double right) {
  int i;
  for (i = 0; i < 4; i++) {
    wb_motor_set_velocity(motors[i + 0], left);
    wb_motor_set_velocity(motors[i + 4], right);
  }
}

// timer to check elapsed time since start of run
static double elapsed_time( ){
  static double start_time;
  static int i;
  if ( i == 0 ){ 
    start_time= wb_robot_get_time() - 1.0;  // -1 to wait for vehicle to start moving!
    i++;
  }
  else { return (wb_robot_get_time() - start_time); }
  return 0.0;
}

// get keyboard inputs, swap between manual and autopilot modes
static void check_keyboard() {
  double speeds[2] = {0.0, 0.0};

  int key = wb_keyboard_get_key();
  if (key >= 0) {
    switch (key) {
      case WB_KEYBOARD_UP:
        speeds[LEFT] = MAX_SPEED;
        speeds[RIGHT] = MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_DOWN:
        speeds[LEFT] = -MAX_SPEED;
        speeds[RIGHT] = -MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_RIGHT:
        speeds[LEFT] = MAX_SPEED;
        speeds[RIGHT] = -MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_LEFT:
        speeds[LEFT] = -MAX_SPEED;
        speeds[RIGHT] = MAX_SPEED;
        autopilot = false;
        break;
      case 'P':
        if (key != old_key) {  // perform this action just once
          const double *position_3d = wb_gps_get_values(gps);
          printf("position: {%f, %f, %f}\n", position_3d[X], position_3d[Y], position_3d[Z]);
        }
        break;
      case 'O':
        if (key != old_key) {  // perform this action just once
          printf("Elapsed Time: %.2f s  |  Distance Travelled: %.2f m  |  Terrain: %s  |  Speed: %.2f Km/h\n", 
                elapsed_time(), DIST, TERRAIN, KMPH);
        }
        break;
      case 'A':
        if (key != old_key)  // perform this action just once
          autopilot = !autopilot;
        break;
    }
  }
  if (autopilot != old_autopilot) {
    old_autopilot = autopilot;
    if (autopilot)
      printf("auto control\n");
    else
      printf("manual control\n");
  }

  robot_set_speed(speeds[LEFT], speeds[RIGHT]);
  old_key = key;
}

// ||v||
static double norm(const Vector *v) {
  return sqrt(v->u * v->u + v->v * v->v);
}

// v = v/||v||
static void normalize(Vector *v) {
  double n = norm(v);
  v->u /= n;
  v->v /= n;
}

// v = v1-v2
static void minus(Vector *v, const Vector *const v1, const Vector *const v2) {
  v->u = v1->u - v2->u;
  v->v = v1->v - v2->v;
}

// autopilot: pass trough the predefined target positions
static void run_autopilot( int *target_points_size, Vector *new_targets ) {
  // prepare the speed array
  double speeds[2] = {0.0, 0.0};

  // read gps position and compass values
  const double *position_3d = wb_gps_get_values(gps);
  const double *north_3d = wb_compass_get_values(compass);

  // compute the 2D position of the robot and its orientation
  const Vector position = {position_3d[X], position_3d[Y]};

  // compute the direction and the distance to the target
  Vector direction;
  minus(&direction, &(new_targets[current_target_index]), &position);
  const double distance = norm(&direction);
  normalize(&direction);

  // compute the error angle
  const double robot_angle = atan2(north_3d[0], north_3d[1]);
  const double target_angle = atan2(direction.v, direction.u);
  double beta = modulus_double(target_angle - robot_angle, 2.0 * M_PI) - M_PI;

  // move singularity
  if (beta > 0)
    beta = M_PI - beta;
  else
    beta = -beta - M_PI;

  // a target position has been reached
  if (distance < DISTANCE_TOLERANCE) {
    char index_char[3] = "th";
    if (current_target_index == 0)
      sprintf(index_char, "st");
    else if (current_target_index == 1)
      sprintf(index_char, "nd");
    else if (current_target_index == 2)
      sprintf(index_char, "rd");
    printf("%d%s target reached\n", current_target_index + 1, index_char);
    current_target_index++;
    current_target_index %= *target_points_size;

    // post results of test if the final waypoint is reached
    if (current_target_index == WAYPOINTS){
      printf("TEST COMPLETE!\n---> Elapsed Time: %.2f s  |  Distance Travelled: %.2f m  |  Average Speed: %.2f Km/h\n", 
            elapsed_time(), DIST, ( (DIST / elapsed_time()) * 60. * 60. ) / 1000. );
    }
  }
  // move the robot to the next target
  else {
    //////////////SPEED LEFT/////////////////
    // speeds[LEFT] = MAX_SPEED - M_PI + TURN_COEFFICIENT * beta;
    speeds[LEFT] = MAX_SPEED + TURN_COEFFICIENT * beta;
    if (speeds[LEFT] > 26. ){ speeds[LEFT] = 26.; } // max speed threshold
    else if (speeds[LEFT] < -26. ){ speeds[LEFT] = -26.; } // min speed threshold
    
    //////////////SPEED RIGHT/////////////////
    // speeds[RIGHT] = MAX_SPEED - M_PI - TURN_COEFFICIENT * beta;
    speeds[RIGHT] = MAX_SPEED - TURN_COEFFICIENT * beta;
    if (speeds[RIGHT] > 26. ){ speeds[RIGHT] = 26.; } // max speed threshold
    else if (speeds[RIGHT] < -26. ){ speeds[RIGHT] = -26.; } // max speed threshold
  }
  // set the motor speeds
  robot_set_speed(speeds[LEFT], speeds[RIGHT]);
}

// Open the vehicle configuration file and extract the translation and rotation values
static Config* vehicle_config ( int *target_points_size ) {
  // setup Waypoint vector and Terrain struct to add values to
  Vector test_targets[*target_points_size];
  TerrainRGB terrain_types[*target_points_size];

  // get parent path, then add relative path extension to config file
  char* parentpath = get_config_path();
  char* fullpath = strcat( parentpath, "maps/WEBOTS_vehicle_config.txt");

  // create vector to append waypoints to
  char * line = NULL;
  size_t len = 0;
  ssize_t read;
  FILE * fp = fopen(fullpath, "r");
  if (fp == NULL)
      exit(EXIT_FAILURE);

  // loop through all the lines in text file
  int row = 0, col = 0;
  while ( (read = getline(&line, &len, fp)) != -1 ){
    // loop through all the words in line
    char *ptr = strtok(line, " ");
    col = 0;
    while ( ptr != NULL ){
      // row for getting the length of waypoints
      if ( row == 4  && col == 1){
        WAYPOINTS = atoi(ptr);  // assign the number of waypoints to variable
        memset( test_targets, 0, atoi(ptr)*sizeof(int) ); 
      }
      // row for getting the waypoints
      if ( row == 5 && col == 1){
        double u, v;
        int count = 0, step = 0;
        char * token = strtok(ptr, "{},");
        while( token != NULL ) {
          // if even, set value to u
          if (count == 0){ u = atof(token); }
          // if odd then set value to v, then push to targets
          else if (count == 1) {
            v = atof(token);
            Vector val = {u, v};
            test_targets[step] = val;
            step++;
          }
          // reset counter i.e. ignore waypoint heights
          else { count = -1; }
          token = strtok(NULL, "{},");
          count++;
        }
      }
      // set memory size of terrain type classes
      if ( row == 6  && col == 1){ memset( terrain_types, 0, atoi(ptr)*sizeof(int) ); 
                                   terrain_classes = atoi(ptr);}  // save size of terrain type classes array
      // row for getting the terrain class colours
      if ( row == 7 && col == 1){
        int el = 0;
        int i = 0;
        TerrainRGB temp;
        char * token = strtok(ptr, "{},");
        while( token != NULL ) {
          // printf("%s\n", token);
          if (i == 0){ strcpy(temp.trn, token); }
          else if (i == 1){ temp.r   = atof(token); }
          else if (i == 2){ temp.g   = atof(token); }
          else if (i == 3){ temp.b   = atof(token); }
          else if (i == 4){ temp.vel = atof(token);
                            i = -1;                     // reset counter       
                            terrain_types[el] = temp;   // add temp TerrainRGB struct to terrain_types super-struct
                            el++;} 
          token = strtok(NULL, "{},");
          i++;
        }
      }
      ptr = strtok(NULL, " ");
      col++;
    }
    row++;
  }
  fclose(fp);

  // setup config struct which is returned by function
  static Config vehicle_config;
  vehicle_config.wpts = test_targets;
  vehicle_config.trns = terrain_types;
  return &vehicle_config;
}

// Return average RGB values of input image as TerrainRGB structure format
static TerrainRGB get_avg_rgb( const unsigned char *image ){
  TerrainRGB trnrgb;    // initialise RGB struct
  trnrgb.r = 0; trnrgb.g = 0; trnrgb.b = 0; // assign zero value to add to
  for (int x = 0; x < image_width; x++){
    for (int y = 0; y < image_height; y++) {
      trnrgb.r += wb_camera_image_get_red(image, image_width, x, y);
      trnrgb.g += wb_camera_image_get_green(image, image_width, x, y);
      trnrgb.b += wb_camera_image_get_blue(image, image_width, x, y);
    }
  }
  // calculate average values
  int adj = 0;
  int px = image_width * image_height;
  trnrgb.r = (adj + trnrgb.r)/px;
  trnrgb.g = (adj + trnrgb.g)/px;
  trnrgb.b = (adj + trnrgb.b)/px;
  return trnrgb;
}

// Using the downward facing camera, check what terrain is below, set speed to corresponding value
static void get_vehicle_velocity( const unsigned char *image, TerrainRGB *terrain_types ){
  // get average image colour in RGB
  TerrainRGB values = get_avg_rgb( image );

  // loop through all classes to see which one is most similar to current avg colour
  int diff;
  int best = 255 * 3; // max difference it could be
  double velocity;
  char temp[50] = "";
  for (int i = 0; i < terrain_classes; i++) {
    // calculate difference between current avg colour and terrain colour
    diff =  abs(values.r - terrain_types[i].r) +
            abs(values.g - terrain_types[i].g) +
            abs(values.b - terrain_types[i].b);
    if (diff < best) {
      strcpy(temp, terrain_types[i].trn);
      velocity = terrain_types[i].vel;
      best = diff;
    }
  }
  // assign new terrain class to global
  strcpy(TERRAIN, temp);

  // km/h to m/s then m/s to rad/s using wheel diameter
  MAX_SPEED = fabs( (2.0 * velocity * 1000.0) / (MOOSE_WHEEL_DIAMETER * pow( 60.0 , 2.0 )) );
}

// Sum the total distance that the vehicle has travelled over the experiment duration
static void distance_travelled( double *points ){
  // get new points
  const double *position_3d = wb_gps_get_values(gps);
  float timestamp = elapsed_time();

  if (first == false){
    // add displacement to total sum
    displacement = sqrt(pow( points[X] - position_3d[X], 2.0 ) + 
                        pow( points[Y] - position_3d[Y], 2.0 ) + 
                        pow( points[Z] - position_3d[Z], 2.0 ) );

    // v = dD / dT i.e. calculate elapsed time from prev time step, then put into km/h from m/s
    KMPH = (displacement * 3.6) / (timestamp - points[Z+1]);
    
    // update total distance travelled
    DIST += displacement;
  }
  else { first = false; }

  // set old points to equal new points
  points[X] = position_3d[X];
  points[Y] = position_3d[Y];
  points[Z] = position_3d[Z];
  points[Z+1] = timestamp; // include timestamp
}


int main(int argc, char *argv[]) {
  // initialize webots communication
  wb_robot_init();

  // setup webots vehicle params for terrain tests
  int target_points_size = MAXIMUM_NUMBER_OF_COORDINATES;
  Config* vehicle_params = vehicle_config( &target_points_size );

  // initialize cameras
  cameraFront = wb_robot_get_device("cameraFront");
  cameraDown = wb_robot_get_device("cameraDown");
  wb_camera_enable(cameraFront, 2 * TIME_STEP);
  wb_camera_enable(cameraDown, 2 * TIME_STEP);

  // print user instructions
  printf("You can drive this robot:\n");
  printf("Select the 3D window and use cursor keys:\n");
  printf("Press 'A' to return to the autopilot mode\n");
  printf("Press 'P' to get the robot position\n");
  printf("\n");

  wb_robot_step(1000);

  const char *names[8] = {"left motor 1",  "left motor 2",  "left motor 3",  "left motor 4",
                          "right motor 1", "right motor 2", "right motor 3", "right motor 4"};

  // get motor tags
  int i;
  for (i = 0; i < 8; i++) {
    motors[i] = wb_robot_get_device(names[i]);
    wb_motor_set_position(motors[i], INFINITY);
  }

  // get gps tag and enable
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  // get compass tag and enable
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);

  // enable keyboard
  wb_keyboard_enable(TIME_STEP);

  // start forward motion
  robot_set_speed(MAX_SPEED, MAX_SPEED);

  // variables for calculating total distance travelled
  double all_positions_3d[4]; // will store values as [oldX, oldY, oldZ, timestamp]

  // main loop
  elapsed_time();   // start timer now
  while (wb_robot_step(TIME_STEP) != -1) {
    // refresh the camera views
    wb_camera_get_image(cameraFront);   
    image = wb_camera_get_image(cameraDown);  // store output to image pointer for processing

    // from input image, get vehicle velocity (determined by colour in frame corresponding 
    // to a specific predefined terrain class)
    get_vehicle_velocity( image, vehicle_params->trns );

    // sum distance travelled in step to total distance travelled
    if (wb_robot_step(TIME_STEP * 10) != -1){ // wait longer time before summing
      distance_travelled( all_positions_3d );
    }

    check_keyboard();
    if (autopilot)
      run_autopilot( &target_points_size, vehicle_params->wpts );
  }
  wb_robot_cleanup();
  return 0;
}