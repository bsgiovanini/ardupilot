#include "Acceleration.h"

void Project::Acceleration::getRawAcc(double ax1, double ay1, double az1, double ax2, double ay2, double az2, Vector3d &acc1, Vector3d &acc2, unsigned long long &time_us) {

    struct timeval time_sample;
    acc1 = Vector3d(ax1, ay1, az1);
    acc2 = Vector3d(ax2, ay2, az2);

    gettimeofday(&time_sample, NULL);
    time_us = TIME(time_sample.tv_sec,time_sample.tv_usec);
}

bool Project::Acceleration::acceptDiffAccs(Vector3d acc1, Vector3d acc2) {

    Vector3d diff(fabs(acc1.x - acc2.x), fabs(acc1.y - acc2.y), fabs(acc1.z - acc2.z));

    return (diff.x <= ACC_DIFF_1_AND_2 && diff.y <= ACC_DIFF_1_AND_2 && diff.z  <= ACC_DIFF_1_AND_2);
}

bool Project::Acceleration::acceptMaxAcc(Vector3d acc) {

    return (fabs(acc.x) <= ACC_MAX && fabs(acc.y) <= ACC_MAX && fabs(acc.z) <= ACC_MAX);
}

Vector3d Project::Acceleration::doInterpolation(sample lastS, sample newS) {


    Vector3d x =  (((newS.data - lastS.data) * deltaT )/ (newS.time_us - lastS.time_us)) + lastS.data;

    return x;

}


void Project::Acceleration::updateAcceleration(double ax1, double ay1, double az1, double ax2, double ay2, double az2, Vector3d attitude) {

    //calibrate accelerometers

    Vector3d acc1;

    Vector3d acc2;
    unsigned long long time;
    getRawAcc(ax1, ay1, az1, ax2, ay2, az2, acc1, acc2, time);


//    printf("acc1  %+7.3f %+7.3f %+7.3f \n", acc1.x, acc1.y, acc1.z);

//    printf("acc2  %+7.3f %+7.3f %+7.3f \n", acc2.x, acc2.y, acc2.z);

    if (!acceptDiffAccs(acc1, acc2)) { //eliminate if the difference between accs is big

        printf("nao aceitou diff\n");
        return;
    }



    Vector3d acc = (acc1 + acc2) * 0.5;

//    printf("Acc1  %+7.3f %+7.3f %+7.3f \n", acc.x, acc.y, acc.z);

    if (!acceptMaxAcc(acc)) {
        printf("nao aceitou acc max\n");
        return;
    }

    handleAcceleration(acc, attitude);
   
//    printf("Acc2  %+7.3f %+7.3f %+7.3f \n", acc.x, acc.y, acc.z);

    if (buffer_acc.empty()) { //first measurement
        sample sp;
        sp.data = acc;
        sp.time_us = time;
        buffer_acc.push_back(sp);
        return;
    }

    sample newSample;
    newSample.data = acc;
    newSample.time_us = time;

    for(deque<sample>::iterator it = buffer_acc.begin(); it != buffer_acc.end(); it++){
        sample s = *it;
        newSample.data += s.data;
    }
    newSample.data = newSample.data/(buffer_acc.size() + 1);

 	
    unsigned long long timeSample = buffer_acc.back().time_us + deltaT;

    if (timeSample <= time) {

        while (timeSample <= time) {
            sample sp;
            sp.data = doInterpolation(buffer_acc.back(), newSample);
            sp.time_us = timeSample;
            buffer_acc.push_back(sp);
            timeSample += deltaT;
        }
    }

    while (buffer_acc.size() > ACC_BUFFER_SIZE) {
        buffer_acc.pop_front();
    }


}

 Vector3d Project::Acceleration::getAcceleration(unsigned long long &timestamp) {

    sample acc = buffer_acc.back();
    timestamp = acc.time_us;
    Vector3d val = acc.data;
    if (fabs(val.x) <= MIN_ACC) val.x = 0.0;
    if (fabs(val.y) <= MIN_ACC) val.y = 0.0;
    if (fabs(val.z) <= MIN_ACC) val.z = 0.0;
    val*= G_SI;	

    return val;
 }

 Vector3d Project::Acceleration::getPrevAcceleration(unsigned long long &timestamp) {
    if (buffer_acc.size() > 1) {

       	Vector3d val = buffer_acc.at(buffer_acc.size()-2).data;
    	if (fabs(val.x) <= MIN_ACC) val.x = 0.0;
   	if (fabs(val.y) <= MIN_ACC) val.y = 0.0;
    	if (fabs(val.z) <= MIN_ACC) val.z = 0.0;
	timestamp = buffer_acc.at(buffer_acc.size()-2).time_us;
	val*= G_SI;
        return val;
    }
    else {
	timestamp = 0;
        return Vector3d(0,0,0);
    }
 }

void Project::Acceleration::handleAcceleration(Vector3d &acc, Vector3d attitude) {

   Matrix3d rotmat;

   rotmat.from_euler(attitude.x, attitude.y, attitude.z);

   //acc = rotmat * acc - Vector3d(0, 0, 1);

   Vector3d g_rot = rotmat.mul_transpose(Vector3d(0, 0, 1.0));

//   printf("grot  %+7.3f %+7.3f %+7.3f \n", g_rot.x, g_rot.y, g_rot.z);

   acc -= g_rot;

}

