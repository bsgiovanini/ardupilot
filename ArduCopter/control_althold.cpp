/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Copter.h"
#include <pthread.h>
#include <iostream>
#include <cstdlib>
#include "collision_verification.h"
#include "MPU9250.h"
#include "LSM9DS1.h"

#define NUM_SONARS     4

struct thread_data{
   Vector3d s_rel_pose;
   Matrix3d s_rot_matrix;
   int address;
};

CollisionVerification cv;

Vector3d position(0,0,0);
Vector3d attitude(0,0,0);

struct thread_data td[NUM_SONARS];

void *sonars_callback(void *threadarg)
{
    struct thread_data *my_data;

    my_data = (struct thread_data *) threadarg;

  //  float range = read from sonar in 10Hz;

    float range = 10.0;

    cv.sonar_callback(range, my_data->s_rel_pose, my_data->s_rot_matrix, position, attitude, 0);

   pthread_exit(NULL);
}

void *pose_callback(void *threadid)
{

   InertialSensor *mpu, *lsm;
   mpu = new MPU9250();
   lsm = new LSM9DS1();

   if (!mpu->probe() || !lsm->probe()) {
	printf("Sensors not enabled\n");
        pthread_exit(NULL);
   }

   mpu->initialize();
   lsm->initialize();

   long tid;
   tid = (long)threadid;
   std::cout << "Pose Thread ID, " << tid << std::endl;

   float axm, axl, aym, ayl, azm, azl;
   float gxm, gxl, gym, gyl, gzm, gzl;
   float mxm, mxl, mym, myl, mzm, mzl;
//-------------------------------------------------------------------------

    while(1) {
        mpu->update();
        lsm->update();
        mpu->read_accelerometer(&axm, &aym, &azm);
        lsm->read_accelerometer(&axl, &ayl, &azl);
        mpu->read_gyroscope(&gxm, &gym, &gzm);
        lsm->read_gyroscope(&gxl, &gyl, &gzl);
        mpu->read_magnetometer(&mxm, &mym, &mzm);
        lsm->read_magnetometer(&mxl, &myl, &mzl);
        printf("Acc: %+7.3f %+7.3f %+7.3f %+7.3f %+7.3f %+7.3f\n ", axm, aym, azm, axl, ayl, azl);
        printf("Gyr: %+8.3f %+8.3f %+8.3f %+8.3f %+8.3f %+8.3f\n ", gxm, gym, gzm, gxl, gyl, gzl);
        printf("Mag: %+7.3f %+7.3f %+7.3f %+7.3f %+7.3f %+7.3f\n", mxm, mym, mzm, mxl, myl, mzl);

       usleep(500000);
    }	

//   while(1) {
//
//   }


   pthread_exit(NULL);
}


/*
 * control_althold.pde - init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Copter::althold_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Alt Hold if the Rotor Runup is not complete
    if (!ignore_checks && !motors.rotor_runup_complete()){
        return false;
    }
#endif

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    // stop takeoff if running
    takeoff_stop();

    //my code - begin

    std::cout << "criando threads....." << std::endl;

    pthread_t thread_sonars[NUM_SONARS], thread_pose;
    int ts, tp = 0, cont;

    std::cout <<"creating thread Pose " << std::endl;
    tp = pthread_create(&thread_pose, NULL, pose_callback, (void *)100);

    if (tp) {
        std::cout << "Error:unable to create pose thread" << std::endl;
    }

    td[0].address = 0x3a;
    td[0].s_rel_pose = Vector3d(0.1, -0.1, 0.12);
    td[0].s_rot_matrix.from_euler(0.0, 0.0, 0.0);
    td[1].address = 0x70;
    td[1].s_rel_pose = Vector3d(0.1, -0.2, 0.12);
    td[1].s_rot_matrix.from_euler(0.0, 0.0, 0.0);
    td[2].address = 0x3c;
    td[2].s_rel_pose = Vector3d(0.1, 0.1, 0.12);
    td[2].s_rot_matrix.from_euler(0.0, 0.0, 0.0);
    td[3].address = 0x37;
    td[3].s_rel_pose = Vector3d(0.1, 0.2, 0.12);
    td[3].s_rot_matrix.from_euler(0.0, 0.0, 0.0);

    for( cont=0; cont < NUM_SONARS; cont++ ){
      std::cout <<" creating thread SONAR " << cont << std::endl;
      ts = pthread_create(&thread_sonars[cont], NULL, sonars_callback, (void *)&td[cont]);
      if (ts){
          std::cout << "Error:unable to create sonar thread," << ts << std::endl;
      }
    }



    //my code -- end

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::althold_run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->control_in);
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    uint32_t now_milli = AP_HAL::millis();

    double timestamp = now_milli/1000;

    attitude = Vector3d(ahrs.roll, ahrs.pitch, ahrs.yaw);

    Vector3d velocity (0.0, 0.0, 0.0);

    cv.nav_callback(timestamp, attitude, velocity , position, target_roll, target_pitch, target_yaw_rate, target_climb_rate);


#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (channel_throttle->control_in > get_takeoff_trigger_throttle()) && motors.rotor_runup_complete());
#else
    bool takeoff_triggered = (ap.land_complete && (channel_throttle->control_in > get_takeoff_trigger_throttle()));
#endif

    // Alt Hold State Machine Determination
    if(!ap.auto_armed) {
        althold_state = AltHold_Disarmed;
    } else if (!motors.get_interlock()){
        althold_state = AltHold_MotorStop;
    } else if (takeoff_state.running || takeoff_triggered){
        althold_state = AltHold_Takeoff;
    } else if (ap.land_complete){
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_Disarmed:

#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        attitude_control.set_yaw_target_to_current_heading();
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // Multicopter do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif  // HELI_FRAME
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in)-throttle_average);
        break;

    case AltHold_MotorStop:

#if FRAME_CONFIG == HELI_FRAME    
        // helicopters are capable of flying even with the motor stopped, therefore we will attempt to keep flying
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // force descent rate and call position controller
        pos_control.set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        pos_control.update_z_controller();
#else   // Multicopter do not stabilize roll/pitch/yaw when motor are stopped
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in)-throttle_average);
#endif  // HELI_FRAME
        break;

    case AltHold_Takeoff:

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
        break;

    case AltHold_Landed:

#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        attitude_control.set_yaw_target_to_current_heading();
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(channel_throttle->control_in),false,g.throttle_filt);
#else   // Multicopter do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(get_throttle_pre_takeoff(channel_throttle->control_in),true,g.throttle_filt);
#endif
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in)-throttle_average);
        break;

    case AltHold_Flying:
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call throttle controller
        if (sonar_enabled && (sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.update_z_controller();
        break;
    }
}
