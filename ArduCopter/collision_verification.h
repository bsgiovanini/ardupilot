/*
 * collision_verification.h
 *
 *  Created on: Mar 6, 2016
 *      Author: bruno
 */

#define OCTREE_RESOLUTION 0.25 // tree grid resolution (meters)
#define MAX_RANGE 1.00 // sonar max range in meters
#define MAP_MAX_RANGE MAX_RANGE-0.01
#define MAX_DIST 1000 // a big enough initialization distance (meters)
#define V_MAX 1.5 // max velocity considered in m/s
#define TIME_AHEAD 1.5 // amount of time will be watch out to predict the trajectory
#define DELTA_VOL V_MAX*TIME_AHEAD
#define TTC_LIMIT 2.0 // time to colide limit
#define CONTROL_LIMIT 1.0 // duration of the automatic control (in second)
#define OCCUPIED_PROB 0.0 // 0 is 50%
#define TRAJECTORY_DT 0.25 // time interval between future trajectory steps (seconds)
#define QUADROTOR_SPHERE_RADIUS  0.32;

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <AP_Math/AP_Math.h>
#include <math.h>
#include <pthread.h>


class PID {
    public:
     double kp, kd, ki;
     double _last_time, _last_error, _error_sum;

     void configure(double kp_l, double kd_l, double ki_l) {
        kp = kp_l;
        kd = kd_l;
        ki = ki_l;
     }

     void reset() {
        _last_time = 0;
        _last_error = std::numeric_limits<double>::infinity();
        _error_sum = 0;
     }

    double getCommand(double e, double timestamp) {
    // Compute dt in seconds
        double dt = timestamp - _last_time;

        double de = 0;
        if (_last_time != 0) {
            // Compute de (error derivation)
            if (_last_error < std::numeric_limits<double>::infinity()) {

                de = (e - _last_error) / dt;
            }

            // Integrate error
            _error_sum += e * dt;
        }

        // Update our trackers
        _last_time = timestamp;
        _last_error = e;

        // Compute commands
        double command = kp * e + ki * _error_sum + kd * de;

        return command;
    }

     PID(double kp_l, double kd_l, double ki_l) {
        configure(kp_l, kd_l, ki_l);
        reset();
     }

     PID() {
     }

};

class CollisionVerification
{
    public:

        CollisionVerification():
            tree(OCTREE_RESOLUTION),
            control_mode(0),
            contador(0),
            pid_x(0.5, 0, 0.35),
            pid_y(0.5, 0, 0.35),
            pid_z(0.8, 0, 0.35),
            pid_yaw(1.0, 0, 0.30),
            yaw_obj(0){

        		if (pthread_mutex_init(&lock, NULL) != 0)
        	    {
        	        printf("\n mutex init failed\n");

        	    }
        }
        void sonar_callback(float range, Vector3d s_rel_pose, Matrix3d s_rel_rot_pose, Vector3d v_pose, Vector3d attitude, int debug);
        //timestamp in seconds
        void nav_callback(double timestamp, Vector3d attitude, Vector3d velocity, Vector3d position, float &desired_roll_cmd, float &desired_pitch_cmd, float &desired_yaw_cmd, float &desired_climb_cmd);
        void initMap();
    private:
        octomap::OcTree tree;
        int control_mode; // init
        float contador;
        PID pid_x; //(0.5, 0, 0.35); init
        PID pid_y; //(0.5, 0, 0.35); init
        PID pid_z; //(0.8, 0, 0.35); init
        PID pid_yaw; //(1.0, 0, 0.30); init
        Vector3d pos_obj;
        double yaw_obj;
        pthread_mutex_t lock;

        std::vector<Vector3d> predict_trajectory2(Vector3d xdot0, Vector3d x0, float tstart, float tend, float dt_p, Vector3d future_position);

        double within(double v,double vmin,double vmax) {

            if (abs(v) > 0.01) {
                if (v < vmin) {
                    return vmin;
                } else if (v > vmax) {
                    return vmax;
                } else {
                    return v;
                }
            } else {
                return 0.0;
            }
        }

        int in_perimeter(Vector3d pos, Vector3d obs_center, float c_factor);
        int there_will_be_collision(Vector3d pos, Vector3d obs_center);


};





