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
    int number_waypoints, option;


    
    if(argc == 2){   // name of executable and name of file
        strcpy(file_name, argv[1]);
    } else {
        printf("Number of arguments not correct: should be 2 - the program call and the waypoints file.\n");
        return 0;
    }

    number_waypoints = readDataFromFile(file_name, data);  // returns 0 in case of no waypoints or invalid values

    if(number_waypoints == 0){
        printf("There was some problem: either a problem with the file or no waypoints were found.");
        return 0;
    }

    printf("\nWhich option would you want?\n0 - GMD\n1 - No error functions;\n2 - Position error due to speed measurement\n");
    printf("3 - Position error with auto-pilot\n4 - Position error with auto-pilot, heading recalculation and dead-reackoning update\n: ");
    scanf("%d", &option);

    if(option < 0 || option > 4){  // option can only be 0, 1, 2, 3, 4
        printf("Invalid option.\n");
        return 0;
    }

    printf("\n\n\n");

    if (option == 0){
        path_distance = calculatePathDistance(data, number_waypoints, option);    // returns final distance in nmi
        printf("GMD: %f nmi\n", path_distance);
    } else
        calculatePositions(data, number_waypoints, option);


    return 0;

}
