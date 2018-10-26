/**** FMS PROJECT - GROUP 1 ****/
// Hugo Marques         : 80929
// Gonçalo Magalhães    : 81817
// Rui Correia          : 90823

// Compilation: gcc functions.c fms.c -lm -o fms
// Calling: ./fms filename.txt



#include "fms.h"


int main(int argc, char *argv[])
{
    /* VARIABLES DEFINITION */
    char file_name[MAX_FILENAME_SIZE];
    WAYPOINT data[MAX_WAYPOINTS];
    float path_distance;
    int number_waypoints;


    
    if(argc == 2){   // name of executable and name of file
        strcpy(file_name, argv[1]);
    } else {
        printf("Number of arguments not correct: should be 2 - the program call and the waypoints file.\n");
        return 0;
    }

    number_waypoints = readDataFromFile(file_name, data);  // returns 0 in case of no waypoints

    if(number_waypoints == 0){
        printf("There was some problem: either a problem with the file or no waypoints were found.");
        return 0;
    }

    // DEBUGGING
    /*
    for(int i = 0; i < number_waypoints; i++){
        printf("%d:\n", i+1);
        printf("%f , %f\n", data[i].latitude, data[i].longitude);
    }
    */
    ////////////

    data[0].time=0;

    path_distance = calculatePathDistance(data, number_waypoints);    // returns final distance in nmi
    calculatePositions(data,number_waypoints,1);

    printf("GMD: %f nmi\n", path_distance);




    
    
    return 0;

}
