#include <IMU/LSM6DO.h>

bool LSM6DO::init(){
    reset();

    if (!get_id()){
        printf("LSM6DO not found\n");
        return false;
    }
    else{
        printf("LSM6DO found\n");
    }

    update_range();
    return true;
};