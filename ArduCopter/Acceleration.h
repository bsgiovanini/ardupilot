#define ACC_BUFFER_SIZE 2
#define ACC_DIFF_1_AND_2 0.5
#define ACC_MAX 3.5
//#define ACC_DELTA_T 100000 //in microssec. deve ser <= ao intervalo de amostras da IMU
#define TIME(a,b) ((a*1000000ull) + b)
#define MIN_ACC 0.03
#define G_SI 9.80665

#include <math.h>

#include <sys/time.h>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits>
#include <unistd.h>
#include <vector>
#include <AP_Math/AP_Math.h>
#include <AP_Math/quaternion.h>

#include <deque>

using namespace std;



namespace Project {

    struct sample {
        Vector3d data;
        unsigned long long time_us;
    };

    class Acceleration {

        public:

            deque<sample> buffer_acc;
  
            unsigned long long deltaT;

            void getRawAcc(double ax1, double ay1, double az1, double ax2, double ay2, double az2, Vector3d &acc1, Vector3d &acc2, unsigned long long &time_us);

            bool acceptDiffAccs(Vector3d acc1, Vector3d acc2);

            bool acceptMaxAcc(Vector3d acc);

            Vector3d doInterpolation(Project::sample lastS, Project::sample newS);

            void updateAcceleration(double ax1, double ay1, double az1, double ax2, double ay2, double az2, Vector3d attitude);

            Vector3d getAcceleration(unsigned long long &timestamp);

            Vector3d getPrevAcceleration(unsigned long long &timestamp);

	    void handleAcceleration(Vector3d &acc, Vector3d attitude);

	    Acceleration(float delta) {
		deltaT = delta;
	    }

    };

    class Velocity {

        public:

    };


}
