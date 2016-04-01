#include "Acceleration.h"

void Project::Acceleration::getRawAcc(double ax1, double ay1, double az1, double ax2, double ay2, double az2, Vector3d &acc1, Vector3d &acc2, unsigned long long &time_us) {

    struct timeval time_sample;
    acc1 = Vector3d(ax1, ay1, az1);
    acc2 = Vector3d(ax2, ay2, az2);

    gettimeofday(&time_sample, NULL);
    time_us = TIME(time_sample.tv_sec,time_sample.tv_usec);
}

bool Project::Acceleration::acceptDiffAccs(Vector3d acc1, Vector3d acc2) {

    Vector3d diff(abs(acc1.x - acc2.x), abs(acc1.y - acc2.y), abs(acc1.z - acc2.z));

    return (diff.x <= ACC_DIFF_1_AND_2 && diff.y <= ACC_DIFF_1_AND_2 && diff.z  <= ACC_DIFF_1_AND_2);
}

bool Project::Acceleration::acceptMaxAcc(Vector3d acc) {

    return (abs(acc.x) <= ACC_MAX && abs(acc.y) <= ACC_MAX && abs(acc.z) <= ACC_MAX);
}

Vector3d Project::Acceleration::doInterpolation(sample lastS, sample newS) {


    Vector3d x =  (((newS.data - lastS.data) * ACC_DELTA_T )/ (newS.time_us - lastS.time_us)) + lastS.data;

    return x;

}


void Project::Acceleration::updateAcceleration(double ax1, double ay1, double az1, double ax2, double ay2, double az2) {

    //calibrate accelerometers

    Vector3d acc1;
    Vector3d acc2;
    unsigned long long time;
    getRawAcc(ax1, ay1, az1, ax2, ay2, az2, acc1, acc2, time);

    if (!acceptDiffAccs(acc1, acc2)) { //eliminate if the difference between accs is big

        printf("nao aceitou diff\n");
        return;
    }

    Vector3d acc = (acc1 + acc2) * 0.5;

    if (!acceptMaxAcc(acc)) {
        printf("nao aceitou acc max\n");
        return;
    }


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

    unsigned long long timeSample = buffer_acc.back().time_us + ACC_DELTA_T;

    if (timeSample <= time) {

        while (timeSample <= time) {
            sample sp;
            sp.data = doInterpolation(buffer_acc.back(), newSample);
            sp.time_us = timeSample;
            buffer_acc.push_back(sp);
            timeSample += ACC_DELTA_T;
        }
    }

    while (buffer_acc.size() > ACC_BUFFER_SIZE) {
        buffer_acc.pop_front();
    }


}

 Vector3d Project::Acceleration::getAcceleration(unsigned long long &timestamp) {

    sample acc = buffer_acc.back();
    timestamp = acc.time_us;
    return acc.data;
 }

 Vector3d Project::Acceleration::getPrevAcceleration(unsigned long long &timestamp) {
    if (buffer_acc.size() > 1) {
	timestamp = buffer_acc.at(buffer_acc.size()-1).time_us;
        return buffer_acc.at(buffer_acc.size()-1).data;
    }
    else {
	timestamp = 0;
        return Vector3d(0,0,0);
    }
 }
