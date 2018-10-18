#include "fms.h"


float degreesToRadians(float deg_angle){
    /* It will get an angle in degrees and return it in radians */
    float rad_angle;

    rad_angle = (deg_angle / 180.0) * PI;    // formula to convert degrees to radians

    return rad_angle;
}



void getWaypointValues(char *line, WAYPOINT *waypoint){
    /* It will parse line of file into values of the waypoint */
    // Name, latitude, longitude, altitude
    // :::: waypoint variable will store values
    int character = 0;
    char separator[] = ";";        // separator between waypoint values on file
    char *parts;
    char values[FIELDS][MAX_LINE_SIZE];
    float latitude, longitude, altitude, speed;

    printf("%s\n", line);

    parts = strtok (line, separator);       // function breaks string into series of tokens, returns pointer to first token
    
    while (parts != NULL){
        strcpy(values[character], parts);   // copies string value to store on another place in memory
        parts = strtok (NULL, separator);
        character++;
    }

    latitude    = degreesToRadians(atof(values[1]));    // atof() converts string to float
    longitude   = degreesToRadians(atof(values[2]));
    altitude    = atof(values[3]);
    speed       = atof(values[4]);

    waypoint->latitude   = latitude;
    waypoint->longitude  = longitude;
    waypoint->altitude   = altitude;
    waypoint->speed      = speed;


}


int readDataFromFile(char *file_name, WAYPOINT *data){
    /* It will read data from given file and convert it into a matrix, that will be returned */
    // data variable will store the waypoints information
    // :::: Returns the number of waypoints in file

    FILE *file = fopen(file_name, "r");

    if(file == NULL){
        // For some reason it did not open file well
        perror(file_name);
        return 0;
    }
    else{
        // Successfully opened!
        int number_waypoints = 0;       // first waypoint
        char line[MAX_LINE_SIZE];

        while(fgets(line, sizeof line, file) != NULL){
            // reads line
            getWaypointValues(line, &data[number_waypoints]);
            number_waypoints++;
        }

        fclose(file);       // close file

        return number_waypoints;
    }
}


float twoPointsDistance(WAYPOINT first_point, WAYPOINT second_point){
    /* It will calculate the global min distance between two waypoints */
    // Altitude of the course is assumed to be the altitude of the first point
    // Latitudes and longitudes are in radians, altitudes are in feet (ft)
    // :::: Returns distance in meters

    float first_latitude, second_latitude, latitude_var;
    float first_longitude, second_longitude, longitude_var;
    float altitude;

    float angle, distance;

    first_latitude = first_point.latitude;
    second_latitude = second_point.latitude;
    latitude_var = second_latitude - first_latitude;

    first_longitude = first_point.longitude;
    second_longitude = second_point.longitude;
    longitude_var = second_longitude - first_longitude;

    altitude = first_point.altitude * 0.3048;     // feet to meters conversion

    // Haversine formula: computes angle between two points on a sphere
    angle = 2 * asin(sqrt(pow(sin(latitude_var / 2.0), 2) + cos(first_latitude)*cos(second_latitude)*pow(sin(longitude_var / 2.0), 2)));
    distance = angle * (RADIUS + altitude);

    return distance;
}


void pointsBetweenWaypoints(WAYPOINT first_point, WAYPOINT second_point, WAYPOINT *middle_points){
    /* It will calculate middle point values between two waypoints */
    // Points are in the great circle path between the two waypoints

    float two_points_distance, altitude;
    int middle_intervals, interval;

    two_points_distance = twoPointsDistance(first_point, second_point);
    altitude = first_point.altitude * 0.3048;     // feet to meters conversion

    middle_intervals = two_points_distance / 20000;     // divide in points separated by 20km

    if(middle_intervals <= 1){
        // no points in the middle
        
    }
    else{
        for(interval = 0; interval < (middle_intervals-1); interval++){
            // TO DO :: CALCULAR NAO SEI QUANTOS PONTOS ENTRE OS WAYPOINTS, CALCULANDO:
            // LATITUDE, LONGITUDE, 
            // SPEED (igual ao primeiro first_point.speed),
        }
    }

}



float calculatePathDistance(WAYPOINT *data, int number_waypoints){
    /* It will calculate the global min distance from a set of waypoints in a matrix */
    // :::: Returns final_distance in Nautical Miles
    
    int point;
    float total_distance = 0;
    float two_points_distance;
    WAYPOINT middle_points[number_waypoints][2000];

    for(point = 1; point < number_waypoints; point++){
        // cycle sums distances between consecutive points
        two_points_distance = twoPointsDistance(data[point-1], data[point]);
        pointsBetweenWaypoints(data[point-1], data[point], middle_points[point-1]);
        total_distance = total_distance + two_points_distance;
        printf("1->%d: %f meters\n", point+1, total_distance);  // total distance so far
    }

    // TO DO ::
    // Nesta altura o middle_points já estará cheio de pontos,
    // portanto tem de haver uma função que passe por todos esses pontos
    // e calcule true heading em cada um, tempo em que está a passar por lá
    // (depois mais para a frente também o theta...)

    // Passing total distance from m to NM
    total_distance = total_distance / 1852.0;

    return total_distance;
    
}

