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
 * 
 * Modifications by Oliver Heilmann
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PATH "/Users/Oliver/Documents/CODING/Python_Prgms/WeBots_ElevationMap/maps/elevationmap_vehicle_config.txt" // path to vehicle configuration file
#define MAXIMUM_NUMBER_OF_COORDINATES 2000  // Size of the history.
#define REFRESH_FACTOR 20                  // Refresh the trail every REFRESH_FACTOR * WorldInfo.basicTimeStep.

// Create the trail shape with the correct number of coordinates.
static void create_route_shape() {
  // If TRAIL exists in the world then silently remove it.
  WbNodeRef existing_route = wb_supervisor_node_get_from_def("MAP_ROUTE");
  if (existing_route)
    wb_supervisor_node_remove(existing_route);

  int i;
  char route_string[0x10000] = "\0";  // Initialize a big string which will contain the TRAIL node.

  // Create the TRAIL Shape.
  strcat(route_string, "DEF MAP_ROUTE Shape {\n");
  strcat(route_string, "  appearance Appearance {\n");
  strcat(route_string, "    material Material {\n");
  strcat(route_string, "      diffuseColor 1 0 0\n");
  strcat(route_string, "      emissiveColor 1 0 0\n");
  strcat(route_string, "    }\n");
  strcat(route_string, "  }\n");
  strcat(route_string, "  geometry DEF ROUTE_LINE_SET IndexedLineSet {\n");
  strcat(route_string, "    coord Coordinate {\n");
  strcat(route_string, "      point [\n");
  for (i = 0; i < MAXIMUM_NUMBER_OF_COORDINATES; ++i)
    strcat(route_string, "      0 0 0\n");
  strcat(route_string, "      ]\n");
  strcat(route_string, "    }\n");
  strcat(route_string, "    coordIndex [\n");
  for (i = 0; i < MAXIMUM_NUMBER_OF_COORDINATES; ++i)
    strcat(route_string, "      0 0 -1\n");
  strcat(route_string, "    ]\n");
  strcat(route_string, "  }\n");
  strcat(route_string, "}\n");

  // Import ROUTE and append it as the world root nodes.
  WbFieldRef root_children_field = wb_supervisor_node_get_field(wb_supervisor_node_get_root(), "children");
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, route_string);
}

// Open the vehicle configuration file and extract the translation and rotation values
static void vehicle_config ( double *new_translation ) {
  char * line = NULL;
  size_t len = 0;
  ssize_t read;
  FILE * fp = fopen(PATH, "r");
  if (fp == NULL)
      exit(EXIT_FAILURE);

  // loop through all the lines in text file
  int row = 0, col = 0;
  while ( (read = getline(&line, &len, fp)) != -1 ){
    // loop through all the words in line
    char *ptr = strtok(line, " ");
    col = 0;
    while ( ptr != NULL ){
      if ( row == 2 ){
        if (col == 1){ new_translation[0] =  atof(ptr); }
        if (col == 2){ new_translation[1] =  atof(ptr); }
        if (col == 3){ new_translation[2] =  atof(ptr); }
      }
      ptr = strtok(NULL, " ");
      col++;
    }
    row++;
  }
  fclose(fp);
}


int main(int argc, char **argv) {
  wb_robot_init();

  // Set the refresh rate of this controller, and so, set the refresh rate of the line set.
  int time_step = (int)wb_robot_get_basic_time_step();  // i.e. `WorldInfo.basicTimeStep`
  time_step *= REFRESH_FACTOR;

  // ////////////////////////////////////////////////////////////////
  // Open text file with vehicle configs, update the translation and rotation fields
  double new_translation[3] = {0, 0, 0};
  vehicle_config( new_translation );

  // // Get route supervisor node and then move to desired coords at startup
  WbNodeRef route_node = wb_supervisor_node_get_from_def("SUPER_ROUTE");
  WbFieldRef translation_field = wb_supervisor_node_get_field(route_node, "translation");
  wb_supervisor_field_set_sf_vec3f(translation_field, new_translation);
  ////////////////////////////////////////////////////////////////

  // // Get the target object node, i.e. the TARGET Transform in the E-puck turretSlot field.
  // WbNodeRef target_node = wb_supervisor_node_get_from_def("TARGET");

  // Create the ROUTE Shape which will contain the red line set.
  create_route_shape();

  // // Get interesting references to the TRAIL subnodes.
  WbNodeRef trail_line_set_node = wb_supervisor_node_get_from_def("ROUTE_LINE_SET");
  WbNodeRef coordinates_node = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(trail_line_set_node, "coord"));
  WbFieldRef point_field = wb_supervisor_node_get_field(coordinates_node, "point");
  WbFieldRef coord_index_field = wb_supervisor_node_get_field(trail_line_set_node, "coordIndex");

  // int index = 0;           // This points to the current position to be drawn.
  // bool first_step = true;  // Only equals to true during the first step.

  // // Main loop.
  // while (wb_robot_step(time_step) != -1) {  
  //   // Get the current target translation.
  //   const double *target_translation = wb_supervisor_node_get_position(target_node);

  //   // Add the new target translation in the line set.
  //   wb_supervisor_field_set_mf_vec3f(point_field, index, target_translation);
 
  //   // Update the line set indices.
  //   if (index > 0) {
  //     // Link successive indices.
  //     wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (index - 1), index - 1);
  //     wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (index - 1) + 1, index);
  //   } else if (index == 0 && first_step == false) {
  //     // Link the first and the last indices.
  //     wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1), 0);
  //     wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1) + 1,
  //                                      MAXIMUM_NUMBER_OF_COORDINATES - 1);
  //   }
  //   // Unset the next indices.
  //   wb_supervisor_field_set_mf_int32(coord_index_field, 3 * index, index);
  //   wb_supervisor_field_set_mf_int32(coord_index_field, 3 * index + 1, index);

  //   // Update global variables.
  //   first_step = false;
  //   index++;
  //   index = index % MAXIMUM_NUMBER_OF_COORDINATES;
  // };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
