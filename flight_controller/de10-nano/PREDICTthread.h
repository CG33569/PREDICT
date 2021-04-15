#ifndef PREDICT_Thread_H
#define PREDICT_Thread_H

// **************************
/////   I2C THREAD   /////
// ***********************
// i2c thread core 1
void i2c_thread()
{
    ////////////////////////////
    /////   SETUP PHASE   /////
    //////////////////////////
    gyro_setup();
    // setup_compass();
    // read_compass();                                               //Read and calculate the compass data.
    // pthread_mutex_lock(&mutex);
    // angle_yaw = actual_compass_heading;                           //Set the initial compass heading.
    // pthread_mutex_unlock(&mutex);
    // barometer_setup();
    if(GYRO_CALLIB)callibrate_gyro();
    if(COMP_CALLIB)callibrate_compass();
    if(LEVEL_CALLIB)callibrate_level();
            //Read the MPU-6050
    __uint32_t timer = get_cpu_usecs();

    ///////////////////////////
    /////   MAIN PHASE   /////
    /////////////////////////
    while(!program_off){
        // Update the variables
        pthread_mutex_lock(&mutex);
        pthread_mutex_unlock(&mutex);
        //Read the MPU-6050, barmeter and compass
        // read_compass();
        gyro_signalen();
        // barometer_pressure();
        while (get_cpu_usecs() - timer < dt*1000000); 
        // if(PRINT_COMMANDS)
        // {
        //     pthread_mutex_lock(&printmutex);
        //     printf("\t\t\ti2c: %llu\n",get_cpu_usecs() - timer );
        //     pthread_mutex_unlock(&printmutex);
        // }

        timer = get_cpu_usecs();
    }

}

//receiver thread core 2
void receiver_thread()
{
    while(!program_off)
    {
        pthread_mutex_lock(&mutex);
        pthread_mutex_unlock(&mutex);
        transmitter_read();
    }
}

//gps and telemetry thread core 3 (not used fue to technical difficulties)
void gps_telemetry_thread()
{
    __int32_t lat_gps_previous, lon_gps_previous;
    __int32_t lat_gps_actual, lon_gps_actual;
    float gps_pitch_adjust_north, gps_roll_adjust_north;
    __uint8_t new_line_found, new_gps_data_available;
    __uint8_t gps_rotating_mem_location;
    __int32_t gps_lat_total_avarage, gps_lon_total_avarage;
    __int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
    __int32_t gps_lat_error, gps_lon_error;
    __int32_t gps_lat_error_previous, gps_lon_error_previous;
    __uint8_t latitude_north, longiude_east ;
    struct gps_tpv GPS;

    if(PRINT_COMMANDS)printf("gps setup\n");
    gps_init_tpv(&GPS);

    while(!program_off){
        read_gps(&GPS);
        lat_gps_actual = abs((double)GPS.latitude/GPS_LAT_LON_FACTOR);
        lon_gps_actual = abs((double)GPS.longitude/GPS_LAT_LON_FACTOR);

        if(PRINT_COMMANDS)printf("GPS = %d, %d, mode = %d\n", lat_gps_actual, lon_gps_actual, GPS.mode);
        //pthread_mutex_unlock(&mutex);
        
        double alt =  (double)GPS.altitude/GPS_VALUE_FACTOR;

        if (GPS.latitude >0)latitude_north = 1;                                               //When flying north of the equator the latitude_north variable will be set to 1.
        else latitude_north = 0;                                                                           //When flying south of the equator the latitude_north variable will be set to 0.

        if (GPS.longitude >0)longiude_east = 1;                                                //When flying east of the prime meridian the longiude_east variable will be set to 1.
        else longiude_east = 0;

        if(GPS.mode == 3)
        {
            pthread_mutex_lock(&mutex);
            number_used_sats =8;                                             //Filter the number of satillites from the GGA line.
            pthread_mutex_unlock(&mutex);
        }
        else
        {
            pthread_mutex_lock(&mutex);
            number_used_sats =3;
            pthread_mutex_unlock(&mutex);
        }



        if (lat_gps_previous == 0 && lon_gps_previous == 0) {                                              //If this is the first time the GPS code is used.
            lat_gps_previous = lat_gps_actual;                                                               //Set the lat_gps_previous variable to the lat_gps_actual variable.
            lon_gps_previous = lon_gps_actual;                                                               //Set the lon_gps_previous variable to the lon_gps_actual variable.
        }
        l_lat_gps = lat_gps_previous;                                                                      //Set the l_lat_gps variable to the previous latitude value.
        l_lon_gps = lon_gps_previous;                                                                      //Set the l_lon_gps variable to the previous longitude value.

        lat_gps_previous = lat_gps_actual;                                                                 //Remember the new latitude value in the lat_gps_previous variable for the next loop.
        lon_gps_previous = lon_gps_actual;                                                                 //Remember the new longitude value in the lat_gps_previous variable for the next loop.

        // printf("lat_gps_actual: %f  lon_gps_actual: %f\n",(double)GPS.latitude,(double)GPS.longitude );
        if (number_used_sats < 8){if(PRINT_COMMANDS && error_print)printf("not enough satellite to lock on");}                                                              //Turn the LED on the STM solid on (LED function is inverted). Check the STM32 schematic.

        if (flight_mode >= 3 && waypoint_set == 0) {                                                          //If the flight mode is 3 (GPS hold) and no waypoints are set.
            pthread_mutex_lock(&mutex);
            waypoint_set = 1;                                                                                   //Indicate that the waypoints are set.
            l_lat_waypoint = l_lat_gps;                                                                         //Remember the current latitude as GPS hold waypoint.
            l_lon_waypoint = l_lon_gps;                                                                         //Remember the current longitude as GPS hold waypoint.
            pthread_mutex_unlock(&mutex);
        }

        if (flight_mode >= 3 && waypoint_set == 1) {                                                          //If the GPS hold mode and the waypoints are stored.
            //GPS stick move adjustments
            if (flight_mode == 3 && takeoff_detected == 1) {
                if (!latitude_north) {
                    l_lat_gps_float_adjust += 0.0015 * (((channel_2 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_1 - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); //South correction
                }
                else {
                    l_lat_gps_float_adjust -= 0.0015 * (((channel_2 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_1 - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); //North correction
                }

                if (!longiude_east) {
                    l_lon_gps_float_adjust -= (0.0015 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_2 - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); //West correction
                }

                else {
                    l_lon_gps_float_adjust += (0.0015 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_2 - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); //East correction
                }
            }

            if (l_lat_gps_float_adjust > 1) {
                l_lat_waypoint ++;
                l_lat_gps_float_adjust --;
            }
            if (l_lat_gps_float_adjust < -1) {
                l_lat_waypoint --;
                l_lat_gps_float_adjust ++;
            }

            if (l_lon_gps_float_adjust > 1) {
                l_lon_waypoint ++;
                l_lon_gps_float_adjust --;
            }
            if (l_lon_gps_float_adjust < -1) {
                l_lon_waypoint --;
                l_lon_gps_float_adjust ++;
            }

            gps_lon_error = l_lon_waypoint - l_lon_gps;                                                         //Calculate the latitude error between waypoint and actual position.
            gps_lat_error = l_lat_gps - l_lat_waypoint;                                                         //Calculate the longitude error between waypoint and actual position.

            gps_lat_total_avarage -=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
            gps_lat_rotating_mem[ gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous;          //Calculate the new change between the actual pressure and the previous measurement.
            gps_lat_total_avarage +=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Add the new value to the long term avarage value.

            gps_lon_total_avarage -=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
            gps_lon_rotating_mem[ gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous;          //Calculate the new change between the actual pressure and the previous measurement.
            gps_lon_total_avarage +=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Add the new value to the long term avarage value.
            gps_rotating_mem_location++;                                                                        //Increase the rotating memory location.
            if ( gps_rotating_mem_location == 35) gps_rotating_mem_location = 0;                                //Start at 0 when the memory location 35 is reached.

            gps_lat_error_previous = gps_lat_error;                                                             //Remember the error for the next loop.
            gps_lon_error_previous = gps_lon_error;                                                             //Remember the error for the next loop.

            //Calculate the GPS pitch and roll correction as if the nose of the multicopter is facing north.
            //The Proportional part = (float)gps_lat_error * gps_p_gain.
            //The Derivative part = (float)gps_lat_total_avarage * gps_d_gain.
            gps_pitch_adjust_north = (float)gps_lat_error * gps_p_gain + (float)gps_lat_total_avarage * gps_d_gain;
            gps_roll_adjust_north = (float)gps_lon_error * gps_p_gain + (float)gps_lon_total_avarage * gps_d_gain;

            if (!latitude_north)gps_pitch_adjust_north *= -1;                                                   //Invert the pitch adjustmet because the quadcopter is flying south of the equator.
            if (!longiude_east)gps_roll_adjust_north *= -1;                                                     //Invert the roll adjustmet because the quadcopter is flying west of the prime meridian.

            //Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
            
            gps_roll_adjust = ((float)gps_roll_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_pitch_adjust_north * cos((angle_yaw - 90) * 0.017453));
            gps_pitch_adjust = ((float)gps_pitch_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_roll_adjust_north * cos((angle_yaw + 90) * 0.017453));

            //Limit the maximum correction to 300. This way we still have full controll with the pitch and roll stick on the transmitter.
            if (gps_roll_adjust > 300) gps_roll_adjust = 300;
            if (gps_roll_adjust < -300) gps_roll_adjust = -300;
            if (gps_pitch_adjust > 300) gps_pitch_adjust = 300;
            if (gps_pitch_adjust < -300) gps_pitch_adjust = -300;
            pthread_mutex_lock(&mutex);
            gps_roll_adjust +=0;
            gps_pitch_adjust +=0;
            pthread_mutex_unlock(&mutex);
        }

        if (flight_mode >= 3 && start > 0) {                                                                  //If flight mode is set to 3 (GPS hold).
            pthread_mutex_lock(&mutex);
            flight_mode = 2;                                                                                    //Set the flight mode to 2.
            pthread_mutex_unlock(&mutex);
        }

        if (flight_mode < 3 && waypoint_set > 0) {                                                              //If the GPS hold mode is disabled and the waypoints are set.
            pthread_mutex_lock(&mutex);
            gps_roll_adjust = 0;                                                                                  //Reset the gps_roll_adjust variable to disable the correction.
            gps_pitch_adjust = 0;                                                                                 //Reset the gps_pitch_adjust variable to disable the correction.
            if (waypoint_set == 1) {                                                                              //If the waypoints are stored
                gps_rotating_mem_location = 0;                                                                      //Set the gps_rotating_mem_location to zero so we can empty the
                waypoint_set = 2;                                                                                   //Set the waypoint_set variable to 2 as an indication that the buffer is not cleared.
            }
            pthread_mutex_unlock(&mutex);
            gps_lon_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
            gps_lat_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
            gps_rotating_mem_location++;                                                                          //Increment the gps_rotating_mem_location variable for the next loop.
            if (gps_rotating_mem_location == 36) {                                                                //If the gps_rotating_mem_location equals 36, all the buffer locations are cleared.
                pthread_mutex_lock(&mutex);
                waypoint_set = 0;                                                                                   //Reset the waypoint_set variable to 0.
                //Reset the variables that are used for the D-controller.
                gps_lat_error_previous = 0;
                gps_lon_error_previous = 0;
                gps_lat_total_avarage = 0;
                gps_lon_total_avarage = 0;
                gps_rotating_mem_location = 0;
                //Reset the waypoints.
                l_lat_waypoint = 0;
                l_lon_waypoint = 0;
                pthread_mutex_unlock(&mutex);
            }
        }
    }

    exchange_telemetry_data();


}

void telemetryPID_thread()
{
    char recUART[512]="";
    const unsigned int msgL = 124;
    
    while(!program_off)
    {
        pthread_mutex_lock(&mutex);
        receive_telemtry(recUART,msgL);
        get_telemetry_pid(recUART,msgL);
        pthread_mutex_unlock(&mutex);
        millis(100);
    }



}



#endif //PREDICT_Thread_H
