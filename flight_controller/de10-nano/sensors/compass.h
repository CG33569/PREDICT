//
// Created by rahul on 01/04/20.
//

#ifndef PATMOS_COMPASS_H
#define PATMOS_COMPASS_H
#include "../basic_lib/PREDICTBasic.h"


void read_compass() {
    // ==================== BAROMETER - PRESSURE CONVERSION ================
    i2c_reg8_write8(0x77, 0x48, (int)NULL); // Send ADC command for pressure
    // =====================================================================
    __int16_t compass_y_tmp = i2c_reg8_read16b(compass_address,0x03);                 //Add the low and high byte to the compass_y variable.
    compass_y_tmp *= -1;                                              //Invert the direction of the axis.
    __int16_t compass_z_tmp = i2c_reg8_read16b(compass_address,0x05);                 //Add the low and high byte to the compass_z variable.;
    __int16_t compass_x_tmp = i2c_reg8_read16b(compass_address,0x07);                 //Add the low and high byte to the compass_x variable.;
    compass_x_tmp *= -1;                                              //Invert the direction of the axis.

    pthread_mutex_lock(&mutex);
    compass_x = compass_x_tmp;
    compass_y = compass_y_tmp;
    compass_z = compass_z_tmp;
    pthread_mutex_unlock(&mutex);

//    printf("compasss: %d, %d, %d ", compass_x,compass_y,compass_z);
    //Before the compass can give accurate measurements it needs to be calibrated. At startup the compass_offset and compass_scale
    //variables are calculated. The following part will adjust the raw compas values so they can be used for the calculation of the heading.
    if (compass_calibration_on == 0) {                            //When the compass is not beeing calibrated.
        compass_y += compass_offset_y;                              //Add the y-offset to the raw value.
        compass_y *= compass_scale_y;                               //Scale the y-value so it matches the other axis.
        compass_z += compass_offset_z;                              //Add the z-offset to the raw value.
        compass_z *= compass_scale_z;                               //Scale the z-value so it matches the other axis.
        compass_x += compass_offset_x;                              //Add the x-offset to the raw value.
    }

    //The compass values change when the roll and pitch angle of the quadcopter changes. That's the reason that the x and y values need to calculated for a virtual horizontal position.
    //The 0.0174533 value is phi/180 as the functions are in radians in stead of degrees.
    fixedpt phiO180 = fixedpt_rconst(0.0174533);
    fixedpt cosRollP = fixedpt_cos(fixedpt_mul(fixedpt_rconst(angle_roll), phiO180));
    fixedpt cosPitchM = fixedpt_cos(fixedpt_mul(fixedpt_rconst(angle_pitch), -phiO180));
    fixedpt sinRollP = fixedpt_sin(fixedpt_mul(fixedpt_rconst(angle_roll), phiO180));
    fixedpt sinPitchM = fixedpt_sin(fixedpt_mul(fixedpt_rconst(angle_pitch), -phiO180));

    fixedpt temp_a = fixedpt_mul(fixedpt_rconst(compass_x), cosPitchM);
    fixedpt temp_b = fixedpt_mul(fixedpt_rconst(compass_y) ,fixedpt_mul(sinRollP, sinPitchM));
    fixedpt temp_c = fixedpt_mul( fixedpt_rconst(compass_z)  , fixedpt_mul( cosRollP,sinPitchM ));
    fixedpt compass_x_horizontal_fx = fixedpt_sub(fixedpt_add(temp_a,temp_b), temp_c );
    fixedpt compass_y_horizontal_fx = fixedpt_add(fixedpt_mul(fixedpt_rconst(compass_y) , cosRollP), fixedpt_mul(fixedpt_rconst(compass_z), sinRollP)) ;


    compass_x_horizontal = fixedpt_2float(compass_x_horizontal_fx);
    compass_y_horizontal = fixedpt_2float(compass_y_horizontal_fx);
    //Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
    //Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
    
    if (compass_y_horizontal < 0)actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
    else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);
    pthread_mutex_lock(&mutex);
    actual_compass_heading += declination;                                 //Add the declination to the magnetic compass heading to get the geographic north.
    if (actual_compass_heading < 0) actual_compass_heading += 360;         //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
    else if (actual_compass_heading >= 360) actual_compass_heading -= 360; //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
    pthread_mutex_unlock(&mutex);
}

//At startup the registers of the compass need to be set. After that the calibration offset and scale values are calculated.
void setup_compass() {
    if(PRINT_COMMANDS)printf("compass setup\n");
    i2c_reg8_write8(compass_address,0x00,0x78);                                            //Set the Configuration Regiser A bits as 01111000 to set sample rate (average of 8 at 75Hz).
    i2c_reg8_write8(compass_address,0x01,0x20);                                            //Set the Configuration Regiser B bits as 00100000 to set the gain at +/-1.3Ga.
    i2c_reg8_write8(compass_address,0x02,0x00);                                            //Set the Mode Regiser bits as 00000000 to set Continues-Measurement Mode.

//Calculate the calibration offset and scale values
    pthread_mutex_lock(&mutex);
    compass_scale_y = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[3] - compass_cal_values[2]);
    compass_scale_z = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);

    compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
    compass_offset_y = (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]) * compass_scale_y;
    compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;
    pthread_mutex_unlock(&mutex);
}


//The following subrouting calculates the smallest difference between two heading values.
float course_deviation(float course_b, float course_c) {
    course_a = course_b - course_c;
    if (course_a < -180 || course_a > 180) {
        if (course_c > 180)base_course_mirrored = course_c - 180;
        else base_course_mirrored = course_c + 180;
        if (course_b > 180)actual_course_mirrored = course_b - 180;
        else actual_course_mirrored = course_b + 180;
        course_a = actual_course_mirrored - base_course_mirrored;
    }
    return course_a;
}


#endif //PATMOS_COMPASS_H
