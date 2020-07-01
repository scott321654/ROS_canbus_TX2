#ifndef _IMU2_H_
#define _IMU2_H_

#include "I2Cdev.h"
#include "imu_config.h"

#include <Wire.h>
#include "geometry_msgs/Vector3.h"

double offsetax = 0, offsetay = 0, offsetaz = 0;

bool initIMU()
{
    Wire.begin();
    bool ret;
    
    accelerometer.initialize();
    ret = accelerometer.testConnection();
    if(!ret)
        return false;

    gyroscope.initialize();
    ret = gyroscope.testConnection();
    if(!ret)
        return false;
  
    magnetometer.initialize();
    /*ret = magnetometer.testConnection();
    if(!ret)
        return false;*/

    return true;
}

void readAccelerometer_Gyroscope(geometry_msgs::Vector3* accel,geometry_msgs::Vector3* gyro)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    
    accelerometer.getAcceleration_getRotation(&ax, &ay, &az, &gx, &gy, &gz);
    //ACCEL_SCALE_SQUARE_G_TO_ACCEL = 0.00059875
    (*accel).x = ax * (double) ACCEL_SCALE_SQUARE_G_TO_ACCEL;
    (*accel).y = ay * (double) ACCEL_SCALE_SQUARE_G_TO_ACCEL;
    (*accel).z = az * (double) ACCEL_SCALE_SQUARE_G_TO_ACCEL;
    //(*accel).x = ax *  0.00059875;
    //(*accel).y = ay *  0.00059875;
    //(*accel).z = az *  0.00059875;
    //pointer intruction the same above
    //GYRO_SCALE_SQUARE_DEG_TO_RAD = 0.00013316
    //gyro->x = gx * (double) GYRO_SCALE_SQUARE_DEG_TO_RAD;
    //gyro->y = gy * (double) GYRO_SCALE_SQUARE_DEG_TO_RAD;
    //gyro->z = gz * (double) GYRO_SCALE_SQUARE_DEG_TO_RAD;
    gyro->x = gx * 0.00013316;
    gyro->y = gy * 0.00013316;
    gyro->z = gz * 0.00013316;
}

geometry_msgs::Vector3 readAccelerometer()
{
    geometry_msgs::Vector3 accel;
    int16_t ax, ay, az;
    
    accelerometer.getAcceleration(&ax, &ay, &az);

    //accel.x = (ax) * (double) ACCEL_SCALE * G_TO_ACCEL;
    //accel.y = (ay) * (double) ACCEL_SCALE * G_TO_ACCEL;
    //accel.z = (az) * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.x = (ax) * (double) ACCEL_SCALE_SQUARE_G_TO_ACCEL;
    accel.y = (ay) * (double) ACCEL_SCALE_SQUARE_G_TO_ACCEL;
    accel.z = (az) * (double) ACCEL_SCALE_SQUARE_G_TO_ACCEL;

    return accel;
}

geometry_msgs::Vector3 readGyroscope()
{
    geometry_msgs::Vector3 gyro;
    int16_t gx, gy, gz;

    gyroscope.getRotation(&gx, &gy, &gz);

    //gyro.x = gx * (double) GYRO_SCALE * DEG_TO_RAD;
    //gyro.y = gy * (double) GYRO_SCALE * DEG_TO_RAD;
    //gyro.z = gz * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.x = gx * (double) GYRO_SCALE_SQUARE_DEG_TO_RAD;
    gyro.y = gy * (double) GYRO_SCALE_SQUARE_DEG_TO_RAD;
    gyro.z = gz * (double) GYRO_SCALE_SQUARE_DEG_TO_RAD;

    return gyro;
}

geometry_msgs::Vector3 readMagnetometer()
{
    geometry_msgs::Vector3 mag;
    int16_t mx, my, mz;

    magnetometer.getHeading(&mx, &my, &mz);

    //mag.x = mx * (double) MAG_SCALE * UTESLA_TO_TESLA;
    //mag.y = my * (double) MAG_SCALE * UTESLA_TO_TESLA;
    //mag.z = mz * (double) MAG_SCALE * UTESLA_TO_TESLA;

    mag.x = mx * (double) MAG_SCALE_SQUARE_UTESLA_TO_TESLA;
    mag.y = my * (double) MAG_SCALE_SQUARE_UTESLA_TO_TESLA;
    mag.z = mz * (double) MAG_SCALE_SQUARE_UTESLA_TO_TESLA;

    return mag;
}

void offsetimu()
{
    int tt = 0;
    int16_t ax, ay, az;
    while(tt < 50)
    {
        accelerometer.getAcceleration(&ax, &ay, &az);
        offsetax = offsetax + ax * (double) ACCEL_SCALE * G_TO_ACCEL;
        offsetay = offsetay + ay * (double) ACCEL_SCALE * G_TO_ACCEL;
        offsetaz = offsetaz + (az * (double) ACCEL_SCALE * G_TO_ACCEL) + (G_TO_ACCEL);
        delayMicroseconds(100000);
        tt++;
    }
    offsetax = offsetax/50;
    offsetay = offsetay/50;
    offsetaz = offsetaz/50;
}

#endif