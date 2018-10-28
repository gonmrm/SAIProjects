#ifndef __FMS_H__
#define __FMS_H__       // This way, fms.h will not be included more than once on program


#include <stdio.h>      // printf, fopen, fgets, fclose
#include <stdlib.h>     // atof
#include <string.h>     // strcpy, strtok
#include <math.h>       // asin, sin, cos, sqrt, pow

#define MAX_FILENAME_SIZE 20
#define MAX_LINE_SIZE 128
#define MAX_WAYPOINTS 100 
#define FIELDS 4
#define PI 3.14159265
#define RADIUS 6371000
#define INTERVAL_DISTANCE 5000
#define ALPHA 0.003

typedef struct waypoint{
    float latitude;
    float longitude;
    float altitude;
    float theta;
    float speed;
    float heading;
    float time;
    float v_north;
    float v_east;    
} WAYPOINT;


float degreesToRadians(float deg_angle);
float radiansToDegrees(float rad_angle);
int invalidWaypointValues(WAYPOINT *data, int number_waypoints);
void getWaypointValues(char *line, WAYPOINT *waypoint);
int readDataFromFile(char *file_name, WAYPOINT *data);
float twoPointsDistance(WAYPOINT first_point, WAYPOINT second_point);
int pointsBetweenWaypoints(WAYPOINT first_point, WAYPOINT second_point, WAYPOINT *middle_points);
float calculatePathDistance(WAYPOINT *data, int number_waypoints, int option);
float calculatePositions(WAYPOINT *data, int number_waypoints, int option);
float calculateHeading(WAYPOINT first_point, WAYPOINT second_point);
float compute_waypoint_time(WAYPOINT first_point, WAYPOINT second_point,float two_points_distance);


#endif      // endif for definition of __FMS_H__ 