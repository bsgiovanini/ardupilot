/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Copter.h"
#include <pthread.h>
#include <iostream>
#include <cstdlib>
#include "collision_verification.h"
#include "MPU9250.h"
#include "LSM9DS1.h"
#include <deque>
#include <sys/time.h>
#include "Acceleration.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>


#define G_SI          9.80665
#define NUM_SONARS     3
#define USLEEP_T       1000
#define ZERO_ACC_LIMIT 64
#define MAX_READING 100


struct thread_data{
   Vector3d s_rel_pose;
   Matrix3d s_rot_matrix;
   int address;
};

CollisionVerification cv;

Vector3d position(0,0,0);
Vector3d attitude(0,0,0);
Vector3d velocity(0,0,0);

struct thread_data td[NUM_SONARS];

void *sonars_callback(void *threadarg)
{
    struct thread_data *my_data;

    int fd;

    my_data = (struct thread_data *) threadarg;

    int dID = my_data->address;

    if((fd=wiringPiI2CSetup(dID))<0)
	printf("error opening i2c channel %d \n", dID);


    while (1) {

	int e = wiringPiI2CWrite(fd, 0x51);

	usleep(100000);

	int r = wiringPiI2CReadReg16(fd, 0xe1);

	int val = (r >> 8) & 0xff | (r << 8) & 0x1;

	if (val <= MAX_READING) {

		float range = val/100.0;
    		cv.sonar_callback(range, my_data->s_rel_pose, my_data->s_rot_matrix, position, attitude, 0);

	} else {
		printf("Not sending %f \n", val/100.0);
	}

   }

   pthread_exit(NULL);
}

void *pose_callback(void *threadid)
{

   InertialSensor *mpu, *lsm;

   Project::Acceleration acc(USLEEP_T);
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
   float axm_offset = -0.0137;
   float aym_offset = 0.0127;
   float azm_offset = -0.0630;
   float axl_offset = 0.0046;
   float ayl_offset = 0.0251;
   float azl_offset = -0.0043;

   Vector3d velocity_prev(0.0, 0.0, 0.0);

  // Vector3d accel_prev(0.0, 0.0, 0.0);

   int count_zeros_acc = 0;

//   struct timeval start_sp, end_sp;

   unsigned long long dt_print_sum = 0;

//-------------------------------------------------------------------------

    while(1) {

 //       gettimeofday(&start_sp, NULL);
 //       unsigned long long start_t_us = TIME(start_sp.tv_sec,start_sp.tv_usec);

        mpu->update();
        lsm->update();
        mpu->read_accelerometer(&axm, &aym, &azm);
        lsm->read_accelerometer(&axl, &ayl, &azl);
        axm=axm/G_SI - axm_offset;
        aym=aym/G_SI - aym_offset;
        azm=azm/G_SI - azm_offset;
        axl=axl/G_SI - axl_offset;
        ayl=ayl/G_SI - ayl_offset;
        azl=azl/G_SI - azl_offset;

	acc.updateAcceleration(-aym, -axm, azm, -ayl, -axl, azl, attitude);
       
	unsigned long long time_acc;
	Vector3d accel = acc.getAcceleration(time_acc);

	if (accel.is_zero()) {
		count_zeros_acc++;
        } else {
		count_zeros_acc = 0;
	}


	unsigned long long time_prev;
	Vector3d accel_prev = acc.getPrevAcceleration(time_prev);
	
	if (time_prev == 0llu) time_prev = time_acc;

	unsigned long long dt_usec = time_acc - time_prev;

	double dt = dt_usec * 0.0000001;

	dt_print_sum += dt_usec;

	//printf("dt   %f\n", dt);

	if (count_zeros_acc >= ZERO_ACC_LIMIT) {
		velocity = Vector3d(0.0, 0.0, 0.0);
        } else {
		
        	velocity = velocity + (accel_prev + accel)*0.5*dt;
	}
	
	position += (velocity_prev + velocity)*0.5*dt;

	if (dt_print_sum >= 500000) {

		printf("Acce  %+7.3f %+7.3f %+7.3f \n", accel.x, accel.y, accel.z);

		printf("Vela  %+7.3f %+7.3f %+7.3f \n", velocity.x, velocity.y, velocity.z);

		printf("Posi  %+7.3f %+7.3f %+7.3f \n", position.x, position.y, position.z);

        //printf("attit  %+7.3f %+7.3f %+7.3f \n", attitude.x, attitude.y, attitude.z);

	//printf("g_inv  %+7.3f %+7.3f %+7.3f \n", g_rot.x, g_rot.y, g_rot.z);

       // printf("Acc: %+7.3f %+7.3f %+7.3f %+7.3f %+7.3f %+7.3f\n ", axm, aym, azm, axl, ayl, azl);
      //  printf("Gyr: %+8.3f %+8.3f %+8.3f %+8.3f %+8.3f %+8.3f\n ", gxm, gym, gzm, gxl, gyl, gzl);
      //  printf("Mag: %+7.3f %+7.3f %+7.3f %+7.3f %+7.3f %+7.3f\n", mxm, mym, mzm, mxl, myl, mzl);

 //       printf(" Attitude roll: %f, pitch: %f, yaw: %f \n", attitude.x, attitude.y, attitude.z);
		dt_print_sum = 0;

	}

	velocity_prev = velocity;

//	accel_prev = accel;

        usleep(USLEEP_T);
    }	


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

    double timestamp = now_milli*0.001;

    attitude = Vector3d(ahrs.roll, ahrs.pitch, ahrs.yaw);
  	
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
