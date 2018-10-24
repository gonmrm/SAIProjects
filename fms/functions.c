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





void getWaypointValues(char *line, WAYPOINT *waypoint){
    /* It will parse line of file into values of the waypoint */
    // Name, latitude, longitude, altitude
    // :::: waypoint variable will store values
    int character = 0;
    char separator[] = ":;";        // separator between waypoint values on file
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

    // Haversine formula: computes central angle between two points on a sphere (angle with relation to sphere center)
    angle = 2 * asin(sqrt(pow(sin(latitude_var / 2.0), 2) + cos(first_latitude)*cos(second_latitude)*pow(sin(longitude_var / 2.0), 2)));
    distance = angle * (RADIUS + altitude);

    return distance;
}


int pointsBetweenWaypoints(WAYPOINT first_point, WAYPOINT second_point, WAYPOINT *middle_points) {
    /* It will calculate middle point values between two waypoints */
    // Points are in the great circle path between the two waypoints

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

    middle_points[0] = first_point; //Colocar no primeiro ponto desta linha da matriz o waypoint que lhe dá inicio

    alfa1 = atan2( (cos(second_point.latitude)*sin(second_point.longitude-first_point.longitude)), ( cos(first_point.latitude)*sin(second_point.latitude) - sin(first_point.latitude)*cos(second_point.latitude)*cos(second_point.longitude-first_point.longitude)) );
    alfa2 = atan2( (cos(first_point.latitude)*sin(second_point.longitude-first_point.longitude)), ( -cos(second_point.latitude)*sin(first_point.latitude) + sin(second_point.latitude)*cos(first_point.latitude)*cos(second_point.longitude-first_point.longitude)) );
    alfa0 = atan2( (sin(alfa1)*cos(first_point.latitude) ), ( sqrt( pow((cos(alfa1)),2) + pow((sin(alfa1)),2) * pow((sin(first_point.latitude)),2) ) ) );

    if (first_point.latitude==0 && alfa1== 0.5*PI)
        sigma01 = 0;
    else
       sigma01 = atan2( (tan(first_point.latitude)) , (cos(alfa1)) );

    lambda01    = atan2( (sin(alfa0)*sin(sigma01)) , (cos(sigma01)) );
    lambda0     = first_point.longitude - lambda01;

    for(interval = 0; interval <= middle_intervals; interval++){
            
        //ACRESCENTAR aquela condição de se está acima de 0 ou abaixo de 0 a todos os atan2)
        //sigma12=atan2( ( sqrt( pow(( cos(first_point.latitude)*sin(second_point.latitude) - sin(first_point.latitude)*cos(second_point.latitude)*cos(second_point.longitude-first_point.longitude) ),2) + pow(( cos(second_point.latitude)*sin(second_point.longitude-first_point.longitude) ),2) ) ), ( sin(first_point.latitude)*sin(second_point.latitude) + cos(first_point.latitude)*cos(second_point.latitude)*cos(second_point.longitude-first_point.longitude) ) );
        //sigma02=sigma01+sigma12;

        sigma = sigma01 + ((interval)*INTERVAL_DISTANCE)/((initial_altitude + RADIUS) * 1.00);  // 1.00 for remaining float

        middle_points[interval].speed       = first_point.speed;
        middle_points[interval].time        = (interval * INTERVAL_DISTANCE / (first_point.speed * 1000 / 3600.00))+first_point.time;

        if (interval != 0){
            // first point does not need to calculate these values
            middle_points[interval].latitude    = atan2 ( (cos(alfa0)*sin(sigma)) , ( sqrt( pow((cos(sigma)),2) + pow((sin(alfa0)),2) * pow((sin(sigma)),2) ) ) );
            middle_points[interval].longitude   = atan2 ( sin(alfa0)*sin(sigma) , (cos(sigma)) ) + lambda0; 
            middle_points[interval].altitude    = first_point.altitude + var_altitude - var_altitude * exp(-ALPHA * middle_points[interval].time);
        }
        next_time       = ((interval + 1) * INTERVAL_DISTANCE / (first_point.speed * 1000 / 3600.00))+first_point.time;
        next_altitude   = first_point.altitude + var_altitude - var_altitude * exp(-ALPHA * next_time);

        middle_points[interval].theta   = atan2( (next_altitude - middle_points[interval].altitude), INTERVAL_DISTANCE );
        middle_points[interval].heading = atan2 ( tan(alfa0), cos(sigma) );
    }

    // DEBUGGING
    ///*
    //for(interval = 0; interval <= middle_intervals; interval++){
    //    printf("LATITUDE : %.4f LONGITUDE : %.4f TIME : %.4f ALTITUDE : %.4f THETA : %.4f HEADING : %.4f\n",radiansToDegrees(middle_points[interval].latitude), radiansToDegrees(middle_points[interval].longitude), middle_points[interval].time, middle_points[interval].altitude, radiansToDegrees(middle_points[interval].theta), radiansToDegrees(middle_points[interval].heading));
    //}
    //*/
    ////////////

    return middle_intervals; //ultima posição com dados relevantes no vetor middle_points;
}
float calculateHeading(WAYPOINT first_point, WAYPOINT second_point){

    WAYPOINT final_point;
    float initial_altitude, var_altitude;
    float alfa0,alfa1,alfa2;
    float sigma,sigma01,sigma02,sigma12;
    float lambda0,lambda01;
    float next_time, next_altitude;

    initial_altitude    = first_point.altitude * 0.3048;     // feet to meters conversion
    var_altitude        = second_point.altitude - first_point.altitude;

    alfa1 = atan2( (cos(second_point.latitude)*sin(second_point.longitude-first_point.longitude)), ( cos(first_point.latitude)*sin(second_point.latitude) - sin(first_point.latitude)*cos(second_point.latitude)*cos(second_point.longitude-first_point.longitude)) );
    alfa2 = atan2( (cos(first_point.latitude)*sin(second_point.longitude-first_point.longitude)), ( -cos(second_point.latitude)*sin(first_point.latitude) + sin(second_point.latitude)*cos(first_point.latitude)*cos(second_point.longitude-first_point.longitude)) );
    alfa0 = atan2( (sin(alfa1)*cos(first_point.latitude) ), ( sqrt( pow((cos(alfa1)),2) + pow((sin(alfa1)),2) * pow((sin(first_point.latitude)),2) ) ) );

    if (first_point.latitude==0 && alfa1== 0.5*PI)
        sigma01 = 0;
    else
       sigma01 = atan2( (tan(first_point.latitude)) , (cos(alfa1)) );            
        //ACRESCENTAR aquela condição de se está acima de 0 ou abaixo de 0 a todos os atan2)
        //sigma12=atan2( ( sqrt( pow(( cos(first_point.latitude)*sin(second_point.latitude) - sin(first_point.latitude)*cos(second_point.latitude)*cos(second_point.longitude-first_point.longitude) ),2) + pow(( cos(second_point.latitude)*sin(second_point.longitude-first_point.longitude) ),2) ) ), ( sin(first_point.latitude)*sin(second_point.latitude) + cos(first_point.latitude)*cos(second_point.latitude)*cos(second_point.longitude-first_point.longitude) ) );
        //sigma02=sigma01+sigma12;

        // Rever esta função-INTERVAL_DISTANCE
    sigma = sigma01 + (INTERVAL_DISTANCE)/((initial_altitude + RADIUS) * 1.00);  // 1.00 for remaining float
    final_point.heading = atan2 ( tan(alfa0), cos(sigma) );

    return final_point.heading;
}


float calculatePathDistance(WAYPOINT *data, int number_waypoints){
    /* It will calculate the global min distance from a set of waypoints in a matrix */
    // :::: Returns final_distance in Nautical Miles
    
    int point;
    float total_distance = 0;
    float two_points_distance;
    
    
    
    for(point=0; point <number_waypoints-1;point++){

        two_points_distance = twoPointsDistance(data[point], data[point+1]);
        printf("Distance between Waypoint W%d and W%d is %.6f meters \n", point+1, point+2, two_points_distance);
        total_distance = total_distance + two_points_distance;
        printf("Distance so far: W1->W%d: %.6f meters\n", point+2, total_distance);  // total distance so far
        data[point+1].time=compute_waypoint_time(data[point],data[point+1],two_points_distance);
   }


    // Passing total distance from m to NM
    printf("Total distance in nautical miles: %.6f\n",total_distance / 1852.0);
    return total_distance;
}

float calculatePositions(WAYPOINT *data, int number_waypoints, int auto_pilot){

    int point, middle_intervals[number_waypoints-1];
    WAYPOINT middle_points[number_waypoints][2000];
    WAYPOINT pontos_com_erro[number_waypoints][2000];
    float two_points_distance;
    // PONTO 2

    if(auto_pilot==0){
    
        for(point=0; point <number_waypoints;point++){

            two_points_distance = twoPointsDistance(data[point], data[point+1]);
            data[point+1].time=compute_waypoint_time(data[point],data[point+1],two_points_distance);
            middle_intervals[point] = pointsBetweenWaypoints(data[point], data[point+1], middle_points[point]);
       }

        for(point = 0; point < number_waypoints; point++){

            float speed_antiga=data[point].speed;
            //printf("SPEED ANTIGA = %.6f\n",speed_antiga);

            for(int i=0;i<=middle_intervals[point];i++){  
                    pontos_com_erro[point][i]=middle_points[point][i];
                    float new_speed= speed_antiga * ( 1.0 + 0.01*sin( (2*PI*(middle_points[point][i].time) ) / (20*60) ) );
                    //printf("SPEED NOVA = %.6f\n",new_speed);
                    pontos_com_erro[point][i].speed=new_speed;
                    pontos_com_erro[point][i].v_north= pontos_com_erro[point][i].speed * cos (pontos_com_erro[point][i].theta)*cos(pontos_com_erro[point][i].heading);
                    pontos_com_erro[point][i].v_east= pontos_com_erro[point][i].speed * cos (pontos_com_erro[point][i].theta)*sin(pontos_com_erro[point][i].heading);

            }
        }

        for(point = 0; point < number_waypoints; point++){

            for(int i=0;i<=middle_intervals[point]-1;i++){

                float var_tempo= pontos_com_erro[point][i+1].time-pontos_com_erro[point][i].time;

                //printf("var_tempo = %.6f\n",var_tempo);

                float var_dist_north= (pontos_com_erro[point][i].v_north * 1000 / 3600)*var_tempo; 
                float var_dist_east= (pontos_com_erro[point][i].v_east * 1000 / 3600)*var_tempo;

                //printf("var_dist north = %.6f\n",var_dist_north);
                //printf("var_dist east = %.6f\n",var_dist_east);

                float var_lat= var_dist_north/111139;
                float var_long= var_dist_east/111139;

                //printf("var_lat = %.6f\n",radiansToDegrees(var_lat));

                pontos_com_erro[point][i+1].latitude=pontos_com_erro[point][i].latitude+degreesToRadians(var_lat);
                pontos_com_erro[point][i+1].longitude=pontos_com_erro[point][i].longitude+degreesToRadians(var_long);


            }


        }


        for(point = 0; point < number_waypoints; point++){

            for(int i=0;i<=middle_intervals[point]-1;i++){

                printf("SEM ERRO : LATITUDE : %.4f LONGITUDE : %.4f TIME : %.4f ALTITUDE : %.4f THETA : %.4f HEADING : %.4f\n",radiansToDegrees(middle_points[point][i].latitude), radiansToDegrees(middle_points[point][i].longitude), middle_points[point][i].time, middle_points[point][i].altitude, radiansToDegrees(middle_points[point][i].theta), radiansToDegrees(middle_points[point][i].heading));
                printf("COM ERRO : LATITUDE : %.4f LONGITUDE : %.4f TIME : %.4f ALTITUDE : %.4f THETA : %.4f HEADING : %.4f\n",radiansToDegrees(pontos_com_erro[point][i].latitude), radiansToDegrees(pontos_com_erro[point][i].longitude), pontos_com_erro[point][i].time, pontos_com_erro[point][i].altitude, radiansToDegrees(pontos_com_erro[point][i].theta), radiansToDegrees(pontos_com_erro[point][i].heading));



            }


        }

    }
    else if(auto_pilot==1){
        // PONTO 3 e 4
        // V(t+intervalo_tempo) = V(tempo) - 0.1 * (V(tempo) - Vref)

        WAYPOINT pontos_com_controlador[number_waypoints][2000];

        float erro;
        float speed_corrigida;

        for(point = 0; point < number_waypoints; point++){

            float speed_antiga=data[point].speed;
            //printf("SPEED ANTIGA = %.6f\n",speed_antiga);

            erro = 0;

            for(int i=0;i<=middle_intervals[point];i++){  
                    pontos_com_controlador[point][i]=middle_points[point][i];

                    if (i != 0){
                        speed_corrigida = pontos_com_controlador[point][i - 1].speed - 2 * erro;
                    }
                    else{
                        speed_corrigida = speed_antiga;
                    }

                    float new_speed= speed_corrigida * ( 1.0 + 0.01*sin( (2*PI*(middle_points[point][i].time) ) / (20*60) ) );
                    //printf("SPEED NOVA = %.6f\n",new_speed);
                    pontos_com_controlador[point][i].speed = new_speed;

                    // TO DO ::::::::
                    /*
                    Isolar função que calcula heading, outra que calcula theta, etc..., que recebem dois middle points e fazem o cálculo.
                    Neste caso vamos utilizar o calculo entre o ponto_corrigido atual e o middle_point [atual+1]

                    Já temos heading e theta definidos no sitio. Agora calculamos v_north e v_east para calcular depois lat e lon do proximo ponto (utilizando já o novo heading), isto está em baixo em comentário.

                    Trocar cena dos 111139

                    */

                    pontos_com_controlador[point][i].v_north= pontos_com_controlador[point][i].speed * cos (pontos_com_controlador[point][i].theta)*cos(pontos_com_controlador[point][i].heading);
                    pontos_com_controlador[point][i].v_east= pontos_com_controlador[point][i].speed * cos (pontos_com_controlador[point][i].theta)*sin(pontos_com_controlador[point][i].heading);

                    erro = pontos_com_controlador[point][i].speed - speed_antiga;
                    printf("%f - %f\n", pontos_com_controlador[point][i].speed, speed_antiga);

            }
        }
    }
    
    /*
    for(point = 0; point < number_waypoints; point++){
        
        for(int i=0;i<=middle_intervals[point]-1;i++){
            
            float var_tempo= pontos_com_erro[point][i+1].time-pontos_com_erro[point][i].time;

            //printf("var_tempo = %.6f\n",var_tempo);

            float var_dist_north= (pontos_com_erro[point][i].v_north * 1000 / 3600)*var_tempo; 
            float var_dist_east= (pontos_com_erro[point][i].v_east * 1000 / 3600)*var_tempo;

            //printf("var_dist north = %.6f\n",var_dist_north);
            //printf("var_dist east = %.6f\n",var_dist_east);

            float var_lat= var_dist_north/111139;  // var_lat/(RADIUS+altitude)
            float var_long= var_dist_east/111139;  // var_lon/((RADIUS+altitude)*cos(latitude) ou uma média da latitude final com a inicial

            //printf("var_lat = %.6f\n",radiansToDegrees(var_lat));

            pontos_com_erro[point][i+1].latitude=pontos_com_erro[point][i].latitude+degreesToRadians(var_lat);
            pontos_com_erro[point][i+1].longitude=pontos_com_erro[point][i].longitude+degreesToRadians(var_long);


        }

    
    }*/





    
}



float compute_waypoint_time(WAYPOINT first_point, WAYPOINT second_point,float two_points_distance){


double tempo=two_points_distance / first_point.speed;
tempo=tempo*1000/3600;
tempo=first_point.time+tempo;

return tempo;
}
