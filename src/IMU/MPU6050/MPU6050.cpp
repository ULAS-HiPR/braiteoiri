#include <IMU/MPU6050.h>

bool MPU6050::init(){
    reset();

    if (!get_id()){
        printf("MPU6050 not found\n");
        return false;
    }
    else{
        printf("MPU6050 found\n");
    }

    update_range();
    set_range(MPU6050_RANGE_2_G);
    return true;
};

bool MPU6050::update(imu_data* out){
    int16_t accel[3];
    read_raw_accel(accel);

    float accel_scale = 1.0;
    switch (accel_range) {
        case MPU6050_RANGE_2_G:  accel_scale = 16384.0; break;
        case MPU6050_RANGE_4_G:  accel_scale = 8192.0; break;
        case MPU6050_RANGE_8_G:  accel_scale = 4096.0; break;
        case MPU6050_RANGE_16_G: accel_scale = 2048.0; break;
    }

    out->accel.x = ((float)accel[0])/ accel_scale;
    out->accel.y = ((float)accel[1])/ accel_scale;
    out->accel.z = ((float)accel[2])/ accel_scale;
    return true;
};

void MPU6050::reset(){
    uint8_t reg = 0x6B; 
    uint8_t buf = 0x80;
    uint8_t data[2] = {reg, buf};
    i2c_handler.write(MPU6050_ADDRESS, data, 1);
    i2c_handler.delay_ms(100);
    buf = 0x00; 
    data [1] = buf;
    i2c_handler.write(MPU6050_ADDRESS, data, 1);
}

void MPU6050::read_raw_accel(int16_t accel[3]){
    uint8_t buffer[6];
    i2c_handler.read(MPU6050_ADDRESS, MPU6050_ACCEL_OUT, buffer, 6);

    for (int i = 0; i < 3; i++) {
        accel[i] = (int16_t)((buffer[i * 2] << 8) | buffer[i * 2 + 1]);
    }
}

bool MPU6050::get_id(){
    uint8_t buffer[1];
    i2c_handler.read(MPU6050_ADDRESS,MPU6050_ID_LOC, buffer, 1);
    printf("MPU6050 ID: %02x\n", buffer[0]);
    if (buffer[0] == MPU6050_ID){
        return true;
    }
    return false;
}

void MPU6050::set_range(mpu6050_accel_range_t accel_range_in){
    uint8_t data[2] = {MPU6050_ACCEL_CONFIG, (accel_range_in << 3)};
    i2c_handler.write(MPU6050_ADDRESS, data, 1);
    printf("MPU6050 set range: %d\n", accel_range_in);
    update_range();
}

void MPU6050::update_range(){
    uint8_t buf[1];
    i2c_handler.read(MPU6050_ADDRESS, MPU6050_ACCEL_OUT, buf, 1);
    accel_range = (mpu6050_accel_range_t)((buf[0] >> 3) & 0x03);
    printf("MPU6050 range: %d\n", accel_range);
}
