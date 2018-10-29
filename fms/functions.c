#include "fms.h"


float degreesToRadians(float deg_angle){
    /* It will get an angle in degrees and return it in radians */
    float rad_angle;

    rad_angle = (deg_angle / 180.0) * PI;    // formula to convert degrees to radians

    return rad_angle;
}

float radiansToDegrees(float rad_angle){
    /* It will get an angle in rad and return it in degrees*/
    float deg_angle;

    deg_angle = (rad_angle / PI) * 180;    // formula to convert radians to degrees

    return deg_angle;
}


int invalidWaypointValues(WAYPOINT *data, int number_waypoints){
    /* It will see if there are any invalid waypoint values */
    // :::: returns 1 if true, 0 if false
    int waypoint_index;

    for (waypoint_index = 0; waypoint_index < number_waypoints; waypoint_index++){
        if (data[waypoint_index].latitude > (PI/2) || data[waypoint_index].latitude < (-PI/2) )
            return 1;
        if (data[waypoint_index].longitude > (PI) || data[waypoint_index].longitude < (-PI) )
            return 1;
        if (data[waypoint_index].altitude <= 0)
            return 1;
        if (data[waypoint_index].speed <= 0)
            return 1;
    }

    return 0;
}





void getWaypointValues(char *line, WAYPOINT *waypoint){
    /* It will parse line of file into values of the waypoint */
    // Name, latitude, longitude, altitude
    // :::: waypoint variable will store values
    int character       = 0;
    char separator[]    = ":;";        // separator between waypoint values on file
    char *parts;
    char values[FIELDS][MAX_LINE_SIZE];

    parts = strtok (line, separator);       // function breaks string into series of tokens, returns pointer to first token
    
    while (parts != NULL){
        strcpy(values[character], parts);   // copies string value to store on another place in memory
        parts = strtok (NULL, separator);
        character++;
    }

    waypoint->latitude    = degreesToRadians(atof(values[1]));    // atof() converts string to float
    waypoint->longitude   = degreesToRadians(atof(values[2]));
    waypoint->altitude    = atof(values[3]);
    waypoint->speed       = atof(values[4]);

    printf("%f;%f\n", waypoint->latitude, waypoint->longitude);
    
}


int readDataFromFile(char *file_name, WAYPOINT *data){
    /* It will read data from given file and convert it into a matrix, that will be returned */
    // data variable will store the waypoints information
    // :::: Returns the number of waypoints in file
    // :::: 0 in case of invalid waypoint values

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

        if (invalidWaypointValues(data, number_waypoints) == 1) // there are some invalid values inside waypoints
            return 0;
        else
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

    first_latitude      = first_point.latitude;
    second_latitude     = second_point.latitude;
    latitude_var        = second_latitude - first_latitude;

    first_longitude     = first_point.longitude;
    second_longitude    = second_point.longitude;
    longitude_var       = second_longitude - first_longitude;

    altitude            = first_point.altitude * 0.3048;     // feet to meters conversion

    // Haversine formula: computes central angle between two points on a sphere (angle with relation to sphere center)
    angle       = 2 * asin(sqrt(pow(sin(latitude_var / 2.0), 2) + cos(first_latitude)*cos(second_latitude)*pow(sin(longitude_var / 2.0), 2)));
    distance    = angle * (RADIUS + altitude);

    return distance;
}


int pointsBetweenWaypoints(WAYPOINT first_point, WAYPOINT second_point, WAYPOINT *middle_points) {
    /* It will calculate middle point values between two waypoints */
    // Points are in the great circle path between the two waypoints
    // :::: returns intervals between the 2 waypoints

    float two_points_distance, initial_altitude, var_altitude;
    int middle_intervals, interval;
    float alfa0,alfa01,alfa1,alfa2;
    float sigma,sigma01,sigma02,sigma12;
    float lambda0,lambda01;
    float next_time, next_altitude;

    two_points_distance = twoPointsDistance(first_point, second_point);
    initial_altitude    = first_point.altitude * 0.3048;     // feet to meters conversion
    var_altitude        = second_point.altitude - first_point.altitude;

    middle_intervals = two_points_distance / INTERVAL_DISTANCE;     // divide in points separated by INTERVAL_DISTANCE meters
    middle_points[0] = first_point; // middle_points begin with first_point


    // Constants used for calculation
    alfa1 = atan2( (cos(second_point.latitude)*sin(second_point.longitude-first_point.longitude)), ( cos(first_point.latitude)*sin(second_point.latitude) - sin(first_point.latitude)*cos(second_point.latitude)*cos(second_point.longitude-first_point.longitude)) );
    alfa2 = atan2( (cos(first_point.latitude)*sin(second_point.longitude-first_point.longitude)), ( -cos(second_point.latitude)*sin(first_point.latitude) + sin(second_point.latitude)*cos(first_point.latitude)*cos(second_point.longitude-first_point.longitude)) );
    alfa0 = atan2( (sin(alfa1)*cos(first_point.latitude) ), ( sqrt( pow((cos(alfa1)),2) + pow((sin(alfa1)),2) * pow((sin(first_point.latitude)),2) ) ) );
    if (first_point.latitude==0 && alfa1== 0.5*PI)
        sigma01 = 0;
    else
       sigma01  = atan2( (tan(first_point.latitude)) , (cos(alfa1)) );
    lambda01    = atan2( (sin(alfa0)*sin(sigma01)) , (cos(sigma01)) );
    lambda0     = first_point.longitude - lambda01;


    for(interval = 0; interval <= middle_intervals; interval++){

        sigma = sigma01 + ((interval)*INTERVAL_DISTANCE)/((initial_altitude + RADIUS) * 1.00);  // 1.00 for remaining float

        middle_points[interval].speed       = first_point.speed;
        middle_points[interval].time        = (interval * INTERVAL_DISTANCE / (first_point.speed * 1000 / 3600.00))+first_point.time;

        if (interval != 0){
            // first point does not need to calculate these values
            middle_points[interval].latitude    = atan2 ( (cos(alfa0)*sin(sigma)) , ( sqrt( pow((cos(sigma)),2) + pow((sin(alfa0)),2) * pow((sin(sigma)),2) ) ) );
            middle_points[interval].longitude   = atan2 ( sin(alfa0)*sin(sigma) , (cos(sigma)) ) + lambda0; 
            middle_points[interval].altitude    = first_point.altitude + var_altitude - var_altitude * exp(-ALPHA * middle_points[interval].time);
            middle_points[interval - 1].heading = calculateHeading(middle_points[interval - 1], middle_points[interval]);
        }
        next_time       = ((interval + 1) * INTERVAL_DISTANCE / (first_point.speed * 1000 / 3600.00))+first_point.time;
        next_altitude   = first_point.altitude + var_altitude - var_altitude * exp(-ALPHA * next_time);

        middle_points[interval].theta   = atan2( (next_altitude - middle_points[interval].altitude), INTERVAL_DISTANCE );
    }

    middle_points[interval - 1].heading = middle_points[interval - 2].heading;  // last middle_point is assumed to have same heading as the one before

    return middle_intervals;
}

float calculateHeading(WAYPOINT first_point, WAYPOINT second_point){
    /* It will calculate heading of first_point based on a straight line towards second_point */
    // :::: returns heading in radians

    float first_latitude, second_latitude;
    float var_longitude;
    float first_heading;
    float x, y;

    first_latitude  = first_point.latitude;
    second_latitude = second_point.latitude;
    var_longitude   = second_point.longitude - first_point.longitude;

    y = sin(var_longitude) * cos(second_latitude);
    x = cos(first_latitude) * sin(second_latitude) - (sin(first_latitude) * cos(second_latitude) * cos(var_longitude));

    first_heading   = atan2(y, x);

    return first_heading;
}


float calculatePathDistance(WAYPOINT *data, int number_waypoints, int option){
    /* It will calculate the global min distance from a set of waypoints in a matrix */
    // :::: Returns final_distance in Nautical Miles
    
    int point, middle_intervals[number_waypoints-1];
    float total_distance = 0;
    float two_points_distance;

    data[0].time = 0; // initialize time
    
    for(point=0; point <number_waypoints-1;point++){

        two_points_distance = twoPointsDistance(data[point], data[point+1]);
        if (option == 0)
            printf("Distance between Waypoint W%d and W%d is %.6f meters \n", point+1, point+2, two_points_distance);

        total_distance      = total_distance + two_points_distance;
        if (option == 0)
            printf("Distance so far: W1->W%d: %.6f meters\n", point+2, total_distance);  // total distance so far

        data[point+1].time  = compute_waypoint_time(data[point],data[point+1],two_points_distance);
    }

    // Passing total distance from m to NM
    total_distance = total_distance / 1852.0;

    return total_distance;
}

float calculatePositions(WAYPOINT *data, int number_waypoints, int option){

    int point, index, middle_intervals[number_waypoints-1];
    WAYPOINT middle_points[number_waypoints][2000];
    WAYPOINT pontos_com_erro[number_waypoints][2000];
    WAYPOINT pontos_com_controlador[number_waypoints][2000];
    float two_points_distance;
    float old_speed;
    float latitude_degree_distance;
    float var_dist_north, var_dist_east;
    float var_time;
    float ui_prev=0;
    float ui=0;
    float up=0;
    float error_prev=0;
    float pos_error;

    
    for(point=0; point < number_waypoints-1;point++){

        two_points_distance     = twoPointsDistance(data[point], data[point+1]);
        data[point+1].time      = compute_waypoint_time(data[point],data[point+1],two_points_distance);
        middle_intervals[point] = pointsBetweenWaypoints(data[point], data[point+1], middle_points[point]);
        
    }
    
    if (option == 1){
        for(point = 0; point < number_waypoints-1; point++){
            for(index = 0; index <= middle_intervals[point]; index++){
                printf("Time: %.2f, Theta: %.5f, Vtas: %.2f, Heading: %3.4f\n", middle_points[point][index].time, middle_points[point][index].theta,middle_points[point][index].speed, radiansToDegrees(middle_points[point][index].heading));
                //printf("%.4f;%.4f;%.4f\n", middle_points[point][index].time, middle_points[point][index].latitude, middle_points[point][index].longitude);
            }
        }
    }

    if(option == 2){

        for(point = 0; point < number_waypoints-1; point++){

            old_speed = data[point].speed;

            for(index = 0; index <= middle_intervals[point]; index++){  
                    pontos_com_erro[point][index]           = middle_points[point][index];
                    pontos_com_erro[point][index].speed     = old_speed * ( 1.0 + 0.01*sin( (2*PI*(middle_points[point][index].time) ) / (20*60) ) );;
                    pontos_com_erro[point][index].v_north   = pontos_com_erro[point][index].speed * cos (pontos_com_erro[point][index].theta)*cos(pontos_com_erro[point][index].heading);
                    pontos_com_erro[point][index].v_east    = pontos_com_erro[point][index].speed * cos (pontos_com_erro[point][index].theta)*sin(pontos_com_erro[point][index].heading);

            }
        }

        for(point = 0; point < number_waypoints-1; point++){

            for(index = 0; index <= middle_intervals[point]-1; index++){ // o index é middle_intervals[point]-1? assim falta um ponto
                
                if(index == 0 && point!=0){

                    var_time        =  pontos_com_erro[point][index].time - pontos_com_erro[point-1][middle_intervals[point-1]].time;
                    var_dist_north  = (pontos_com_erro[point][index].v_north * 1000 / 3600)*var_time; 
                    var_dist_east   = (pontos_com_erro[point][index].v_east * 1000 / 3600)*var_time;

                    latitude_degree_distance = 2 * PI * (RADIUS + pontos_com_erro[point][index].altitude) / 360.0;

                    pontos_com_erro[point][index].latitude=pontos_com_erro[point-1][middle_intervals[point-1]].latitude + degreesToRadians(var_dist_north/latitude_degree_distance);;
                    pontos_com_erro[point][index].longitude=pontos_com_erro[point-1][middle_intervals[point-1]].longitude + degreesToRadians(var_dist_east/( cos(pontos_com_erro[point][index].latitude) * latitude_degree_distance ));;
                }

                var_time        =  pontos_com_erro[point][index+1].time - pontos_com_erro[point][index].time;
                var_dist_north  = (pontos_com_erro[point][index].v_north * 1000 / 3600)*var_time; 
                var_dist_east   = (pontos_com_erro[point][index].v_east * 1000 / 3600)*var_time;

                latitude_degree_distance = 2 * PI * (RADIUS + pontos_com_erro[point][index].altitude) / 360.0;

                pontos_com_erro[point][index+1].latitude    = pontos_com_erro[point][index].latitude + degreesToRadians(var_dist_north/latitude_degree_distance);
                pontos_com_erro[point][index+1].longitude   = pontos_com_erro[point][index].longitude + degreesToRadians(var_dist_east/( cos(pontos_com_erro[point][index].latitude) * latitude_degree_distance ));

                pos_error = twoPointsDistance(middle_points[point][index], pontos_com_erro[point][index]);
                printf("Time: %f, Error: %f\n", pontos_com_erro[point][index].time, pos_error);
                //printf("%.4f;%.4f;%.4f\n", pontos_com_erro[point][index].time, pontos_com_erro[point][index].latitude, pontos_com_erro[point][index].longitude);
                //printf("Time: %.2f, Theta: %.2f, Vtas: %.2f, Heading: %3.4f, error: %f\n", pontos_com_erro[point][index].time, pontos_com_erro[point][index].theta, pontos_com_erro[point][index].speed, pontos_com_erro[point][index].heading, pos_error);

            }
        }
    }
    if(option == 3 || option == 4){

        float error;
        float corrected_speed;
        int index;

        for(point = 0; point < number_waypoints-1; point++){

            old_speed = data[point].speed;

            for(index = 0; index <= middle_intervals[point]; index++){  

                pontos_com_controlador[point][index]=middle_points[point][index];
            }

            for(index = 0; index <= middle_intervals[point]-1; index++){  

                if (point != 0){
                    if(index == 0){
                        up = K_P * (error);
                        ui = ui_prev + (K_P*K_I*15/2) * (error + error_prev);
                        corrected_speed = pontos_com_controlador[point-1][middle_intervals[point-1]].speed -(ui+up) ;
                        ui_prev = ui;
                        error_prev = error; 
                    }
                    else{
                        up = K_P * (error);
                        ui = ui_prev + (K_P*K_I*15/2) * (error + error_prev);
                        corrected_speed = pontos_com_controlador[point][index - 1].speed -(ui+up) ;
                        ui_prev = ui;
                        error_prev = error;
                    }
                }
                else{
                    corrected_speed = old_speed;
                }

                pontos_com_controlador[point][index].speed   = corrected_speed * ( 1.0 + 0.01*sin( (2*PI*(middle_points[point][index].time) ) / (20*60) ) );

                if(option==4){
                    pontos_com_controlador[point][index].heading = calculateHeading(pontos_com_controlador[point][index], middle_points[point][index+1]);    
                }

                pontos_com_controlador[point][index].v_north = pontos_com_controlador[point][index].speed * cos (pontos_com_controlador[point][index].theta)*cos(pontos_com_controlador[point][index].heading);
                pontos_com_controlador[point][index].v_east  = pontos_com_controlador[point][index].speed * cos (pontos_com_controlador[point][index].theta)*sin(pontos_com_controlador[point][index].heading);

                error = pontos_com_controlador[point][index].speed - old_speed;

                var_time = pontos_com_controlador[point][index+1].time - pontos_com_controlador[point][index].time;

                var_dist_north = (pontos_com_controlador[point][index].v_north * 1000 / 3600)*var_time; 
                var_dist_east  = (pontos_com_controlador[point][index].v_east * 1000 / 3600)*var_time;

                latitude_degree_distance = 2 * PI * (RADIUS + pontos_com_controlador[point][index].altitude) / 360.0;

                if(option==3){
                    if(index == 0 && point!=0){
                        
                        var_time        =  pontos_com_controlador[point][index].time - pontos_com_controlador[point-1][middle_intervals[point-1]].time;
                        var_dist_north  = (pontos_com_controlador[point][index].v_north * 1000 / 3600)*var_time; 
                        var_dist_east   = (pontos_com_controlador[point][index].v_east * 1000 / 3600)*var_time;

                        latitude_degree_distance = 2 * PI * (RADIUS + pontos_com_controlador[point][index].altitude) / 360.0;
    
                        pontos_com_controlador[point][index].latitude=pontos_com_controlador[point-1][middle_intervals[point-1]].latitude + degreesToRadians(var_dist_north/latitude_degree_distance);;
                        pontos_com_controlador[point][index].longitude=pontos_com_controlador[point-1][middle_intervals[point-1]].longitude + degreesToRadians(var_dist_east/( cos(pontos_com_erro[point][index].latitude) * latitude_degree_distance ));;
                   }
                }

                pontos_com_controlador[point][index+1].latitude  = pontos_com_controlador[point][index].latitude+degreesToRadians(var_dist_north/latitude_degree_distance);
                pontos_com_controlador[point][index+1].longitude = pontos_com_controlador[point][index].longitude+degreesToRadians(var_dist_east/( cos(pontos_com_controlador[point][index].latitude) * latitude_degree_distance ));

                pos_error = twoPointsDistance(middle_points[point][index], pontos_com_controlador[point][index]);
                //printf("%.4f;%.4f;%.4f\n", pontos_com_controlador[point][index].time, pontos_com_controlador[point][index].latitude, pontos_com_controlador[point][index].longitude);
                printf("Time: %.2f, Error: %f\n", pontos_com_controlador[point][index].time, pos_error);
                //printf("Time: %.2f, Theta: %.2f, Vtas: %.2f, Heading: %3.1f, error: %f\n", pontos_com_controlador[point][index].time, pontos_com_controlador[point][index].theta, pontos_com_controlador[point][index].speed, pontos_com_controlador[point][index].heading, pos_error);

            }
            pontos_com_controlador[point][index].heading = pontos_com_controlador[point][index - 1].heading;
        }
    }

    return 0.0;
    
}



float compute_waypoint_time(WAYPOINT first_point, WAYPOINT second_point,float two_points_distance){
    /* It will calculate the time instant when the second waypoint is being passed */
    // :::: returns time in seconds

    double time;
    
    time = (two_points_distance / first_point.speed) * (3600/1000.0);
    time = first_point.time + time;

    return time;
}
