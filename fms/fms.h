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
#define RADIUS 637100

typedef struct waypoint{
    float latitude;
    float longitude;
    float altitude;
    float theta;
    float speed;
    float heading;
    float time;
} WAYPOINT;


float degreesToRadians(float deg_angle);
float radiansToDegrees(float rad_angle);
void getWaypointValues(char *line, WAYPOINT *waypoint);
int readDataFromFile(char *file_name, WAYPOINT *data);
float twoPointsDistance(WAYPOINT first_point, WAYPOINT second_point);
int pointsBetweenWaypoints(WAYPOINT first_point, WAYPOINT second_point, WAYPOINT *middle_points);
float calculatePathDistance(WAYPOINT *data, int number_waypoints);


#endif      // endif for definition of __FMS_H__ 