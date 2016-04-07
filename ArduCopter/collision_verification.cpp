/*
 * collision_verification.cpp

 *
 *  Created on: Mar 6, 2016
 *      Author: bruno
 */

#include "collision_verification.h"


std::vector<Vector3d> CollisionVerification::predict_trajectory2(Vector3d xdot0, Vector3d x0, float tstart, float tend, float dt_p, Vector3d future_position) {


    Vector3d x_p = x0;
    std::vector<Vector3d> trajectory;

    int nIntervals = (tend - tstart)/dt_p;

    for (int nTurn = 1; nTurn <= nIntervals; nTurn++) {


        //cout << xdot << endl;
        x_p = x_p + (xdot0 * dt_p); //posicao linear

        trajectory.push_back(x_p);

    }

    future_position = x_p;

    return trajectory;

    //ROS_INFO("I heard ax: [%f]  ay: [%f] az: [%f]", x_p.x, x_p.y, x_p.z);

}

int CollisionVerification::in_perimeter(Vector3d pos, Vector3d obs_center, float c_factor) {

    float flb_x_c_obstacle = obs_center.x - c_factor;
    float flb_y_c_obstacle = obs_center.y - c_factor;
    float flb_z_c_obstacle = obs_center.z - c_factor;

    //back - left - bottom
    float blb_x_c_obstacle = obs_center.x + c_factor;
    float blb_y_c_obstacle = obs_center.y - c_factor;
    float blb_z_c_obstacle = obs_center.z - c_factor;

    //back - right - bottom
    float brb_x_c_obstacle = obs_center.x + c_factor;
    float brb_y_c_obstacle = obs_center.y + c_factor;
    float brb_z_c_obstacle = obs_center.z - c_factor;

    //front - right - bottom
    float frb_x_c_obstacle = obs_center.x - c_factor;
    float frb_y_c_obstacle = obs_center.y + c_factor;
    float frb_z_c_obstacle = obs_center.z - c_factor;

    //front - left - top
    float flt_x_c_obstacle = obs_center.x - c_factor;
    float flt_y_c_obstacle = obs_center.y - c_factor;
    float flt_z_c_obstacle = obs_center.z + c_factor;

    //back - left - top
    float blt_x_c_obstacle = obs_center.x + c_factor;
    float blt_y_c_obstacle = obs_center.y - c_factor;
    float blt_z_c_obstacle = obs_center.z + c_factor;

    //font - right - top
    float frt_x_c_obstacle = obs_center.x - c_factor;
    float frt_y_c_obstacle = obs_center.y + c_factor;
    float frt_z_c_obstacle = obs_center.z + c_factor;

    //back - right - top
    float brt_x_c_obstacle = obs_center.x + c_factor;
    float brt_y_c_obstacle = obs_center.y + c_factor;
    float brt_z_c_obstacle = obs_center.z + c_factor;

    int verify_x = ((pos.x >= flb_x_c_obstacle) && (pos.x >= frb_x_c_obstacle) && (pos.x >= flt_x_c_obstacle) && (pos.x >= frt_x_c_obstacle) &&
    (pos.x <= blb_x_c_obstacle) && (pos.x <= brb_x_c_obstacle) && (pos.x <= blt_x_c_obstacle) && (pos.x <= brt_x_c_obstacle));

    int verify_y = ((pos.y >= flb_y_c_obstacle) && (pos.y >= blb_y_c_obstacle) && (pos.y >= flt_y_c_obstacle) && (pos.y >= blt_y_c_obstacle) &&
    (pos.y <=  frb_y_c_obstacle ) && (pos.y <= brb_y_c_obstacle) && (pos.y <= frt_y_c_obstacle ) && (pos.y <= brt_y_c_obstacle));

    int verify_z = ((pos.z >= flb_z_c_obstacle) && (pos.z >= frb_z_c_obstacle) && (pos.z >= blb_z_c_obstacle) && (pos.z >= brb_z_c_obstacle) &&
    (pos.z <= flt_z_c_obstacle  ) && (pos.z <= frt_z_c_obstacle ) && (pos.z <= blt_z_c_obstacle) && (pos.z <= brt_z_c_obstacle));

    return verify_x && verify_y && verify_z;


}



int CollisionVerification::there_will_be_collision(Vector3d pos, Vector3d obs_center) {

    float c_factor = (OCTREE_RESOLUTION * sqrt(2)/2) + QUADROTOR_SPHERE_RADIUS;

    return in_perimeter(pos, obs_center, c_factor);

}


void CollisionVerification::sonar_callback(float range, Vector3d s_rel_pose, Matrix3d s_rel_rot_pose, Vector3d v_pose, Vector3d attitude, int debug) {

    Matrix3d R;

    R.from_euler((float)attitude.x, (float)attitude.y, (float)attitude.z);

    Vector3d global_s_pose = v_pose + R * s_rel_pose;

    Vector3d interm = s_rel_pose +  s_rel_rot_pose * Vector3d(range, 0, 0);

    Vector3d global_end_ray = v_pose + R * (interm);

    octomap::point3d startPoint (global_s_pose.x, global_s_pose.y, global_s_pose.z);

    octomap::point3d endPoint (global_end_ray.x, global_end_ray.y, global_end_ray.z);


    if (global_end_ray.z > 0.1) {  //evict the ground

           tree.insertRay (startPoint, endPoint, MAP_MAX_RANGE);
    }

}

void CollisionVerification::nav_callback(double timestamp, Vector3d attitude, Vector3d velocity, Vector3d position, double &desired_roll_cmd, double &desired_pitch_cmd, double &desired_yaw_cmd, double &desired_climb_cmd) {

    Vector3d future_position;

    std::vector<Vector3d> trajectory = predict_trajectory2(velocity, position, timestamp, timestamp + TIME_AHEAD, TRAJECTORY_DT, future_position);

    float short_dist = MAX_DIST;

    if (control_mode) {

    //        cout << "TIMESTAMP " << timestamp << endl;
    //        cout << "CONTADOR " << contador << endl;
    //        cout << "LIMIT " << CONTROL_LIMIT << endl;
            if (timestamp < contador + CONTROL_LIMIT) {

                double u_x = pid_x.getCommand(pos_obj.x - position.x, timestamp);
                double u_y = pid_y.getCommand(pos_obj.y - position.y, timestamp);
                double u_z = pid_z.getCommand(pos_obj.z - position.z, timestamp);
                double u_yaw = pid_yaw.getCommand(yaw_obj - attitude.z, timestamp);

                double cx   = within(cos(attitude.z) * u_x + sin(attitude.z) * u_y, -1, 1);
                double cy   = within(-sin(attitude.z) * u_x + cos(attitude.z) * u_y, -1, 1);
                double cz   = within(u_z, -1, 1);
                double cyaw = within(u_yaw, -1, 1);

               // f_vector_print("objetivo", pos_obj);

               // f_vector_print("posicao", x);

           std::cout << "SENDING AUTOMATIC CONTROLS" << std::endl;

               desired_roll_cmd = cx;
               desired_pitch_cmd = cy;
               desired_yaw_cmd = cyaw;
               desired_climb_cmd = cz;

            } else {

                control_mode = 0;
                std::cout << "CONTROL MODE OFF" << std::endl;
                pid_x.reset();
                pid_y.reset();
                pid_z.reset();
                pid_yaw.reset();
            }


        } else {

            octomap::OcTreeKey bbxMinKey, bbxMaxKey;

            //Vector3d lim1 = x + R * Vector3d(0, -DELTA_VOL/2, -1.0);
            //Vector3d lim2 = x + R * Vector3d(DELTA_VOL, DELTA_VOL/2, 1.0);

            octomap::point3d min_vol = octomap::point3d(position.x-DELTA_VOL/2, position.y -DELTA_VOL/2, position.z-1.0);
            octomap::point3d max_vol = octomap::point3d(position.x+DELTA_VOL/2, position.y +DELTA_VOL/2, position.z+1.0);


            tree.coordToKeyChecked(min_vol, bbxMinKey);
            tree.coordToKeyChecked(max_vol, bbxMaxKey);

            for(octomap::OcTree::leaf_bbx_iterator it = tree.begin_leafs_bbx(bbxMinKey, bbxMaxKey), end_bbx = tree.end_leafs_bbx(); it!= end_bbx; ++it){

                //cout << "passei" << endl;

                octomap::point3d coords = it.getCoordinate();

                Vector3d wrapped_coords = Vector3d(coords(0), coords(1), coords(2));


                if (it->getValue() > OCCUPIED_PROB) {


                    for (std::vector<Vector3d>::iterator it=trajectory.begin(); it!=trajectory.end(); ++it) {

                        Vector3d pos = *it;

                        if (there_will_be_collision(pos, wrapped_coords)) {

                            float dist = (wrapped_coords - position).length();

                            std::cout << "DIST " << dist << std::endl;

                            if (dist < short_dist) {
                                short_dist = dist;
                            }
                            break;
                        }

                    }

                }
                  //manipulate node, e.g.:
                  //cout << "Node center: " << it.getCoordinate() << endl;
                  //cout << "Node size: " << it.getSize() << endl;
                  //cout << "Node value: " << it->getValue() << endl;

                if (short_dist < MAX_DIST) {

                    float ttc = short_dist/velocity.length();

                    if (ttc < TTC_LIMIT) {

                        pos_obj = position;
                        yaw_obj = atan2(sin(attitude.z),cos(attitude.z));
                        control_mode = 1;
                        contador = timestamp;
                        std::cout << "CONTROL MODE ON" << std::endl;
                        pid_x.reset();
                        pid_y.reset();
                        pid_z.reset();
                        pid_yaw.reset();

                    }

                }
            }
            //}

        }


        //gettimeofday(&stop, NULL);


        //previous_omega = omega;

        //gettimeofday(&stop, NULL);


}








