/*
 * By Oliver Heilmann
 */

#include <webots/robot.h>
#include <webots/supervisor.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PATH "/Users/Oliver/Documents/CODING/Python_Prgms/WeBots_ElevationMap/maps/elevationmap_vehicle_config.txt" // path to vehicle configuration file
#define MAXIMUM_NUMBER_OF_COORDINATES 2000  // Max size of the history.

typedef struct _Vector {
  double x;
  double y;
  double z;
} Vector;

// Create the trail shape with the correct number of coordinates.
static void create_route_shape( int *max_num_coords ) {
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
  for (i = 0; i < *max_num_coords; ++i)
    strcat(route_string, "      0 0 0\n");
  strcat(route_string, "      ]\n");
  strcat(route_string, "    }\n");
  strcat(route_string, "    coordIndex [\n");
  for (i = 0; i < *max_num_coords; ++i)
    strcat(route_string, "      0 0 -1\n");
  strcat(route_string, "    ]\n");
  strcat(route_string, "  }\n");
  strcat(route_string, "}\n");

  // Import ROUTE and append it as the world root nodes.
  WbFieldRef root_children_field = wb_supervisor_node_get_field(wb_supervisor_node_get_root(), "children");
  wb_supervisor_field_import_mf_node_from_string(root_children_field, -1, route_string);
}

// Open the vehicle configuration file and extract the translation and rotation values
Vector* vehicle_config ( double *new_translation, int *tps, Vector coords[] ) {
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

      // row for getting vehicle start coordinates
      if ( row == 2 ){
        if (col == 1){ new_translation[0] =  atof(ptr); }
        if (col == 2){ new_translation[1] =  atof(ptr); }
        if (col == 3){ new_translation[2] =  atof(ptr); }
      }

      // row for getting the length of waypoints
      if ( row == 4  && col == 1){
        *tps =  atoi(ptr);
        memset( coords, 0, *tps*sizeof(int) );
      }

      // row for getting waypoints x,y,z and putting in vector pointer array
      if ( row == 5 && col == 1){
        double x, y, z;
        int c = 0, s = 0;
        char * token = strtok(ptr, "{},");
        while( token != NULL ) {
          if (c == 0){ x = atof(token); }
          else if (c == 1){ y = atof(token); }
          else if (c == 2){ 
            z = atof(token);
            Vector val = {x, y, z};
            coords[s] = val;
            c = -1;
            s++;
          }
          token = strtok(NULL, "{},");
          c++;
        }
      }
      ptr = strtok(NULL, " ");
      col++;
    }
    row++;
  }
  fclose(fp);
  return coords;
}


int main(int argc, char **argv) {
  wb_robot_init();

  // Open text file with vehicle configs, update the translation and rotation fields
  double new_translation[3] = {0, 0, 0};
  int target_points_size = MAXIMUM_NUMBER_OF_COORDINATES;
  Vector coordinates[target_points_size];
  Vector *new_coords = vehicle_config( new_translation, &target_points_size, coordinates );

  // // Get route supervisor node and then move to desired coords at startup
  WbNodeRef route_node = wb_supervisor_node_get_from_def("SUPER_ROUTE");
  WbFieldRef translation_field = wb_supervisor_node_get_field(route_node, "translation");
  wb_supervisor_field_set_sf_vec3f(translation_field, new_translation);

  // Create the MAP_ROUTE Shape which will contain the red line set.
  create_route_shape( &target_points_size );

  // Get interesting references to the TRAIL subnodes.
  WbNodeRef route_line_set_node = wb_supervisor_node_get_from_def("ROUTE_LINE_SET");
  WbNodeRef coordinates_node = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(route_line_set_node, "coord"));
  WbFieldRef point_field = wb_supervisor_node_get_field(coordinates_node, "point");
  WbFieldRef coord_index_field = wb_supervisor_node_get_field(route_line_set_node, "coordIndex");

  // Loop through waypoint coordinates and draw lines between them
  for (int i = 0; i < target_points_size; i++) {

    // extract Vector struct params and reformat to Webots format
    double target_translation[3] =  {  new_coords[i].x, new_coords[i].y, new_coords[i].z };

    // Add the new target translation in the line set.
    wb_supervisor_field_set_mf_vec3f(point_field, i, target_translation);

    // Update the line set indices.
    if (i > 0) {
      // Link successive indices.
      wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (i - 1), i - 1);
      wb_supervisor_field_set_mf_int32(coord_index_field, 3 * (i - 1) + 1, i);
    }
  }
  // // closing stuff
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
