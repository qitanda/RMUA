#ifndef PARAM_H
#define PARAM_H

#include <ros/ros.h>

class Param{
public:
    struct Gain {
		double Kp0, Kp1, Kp2;
		double Kv0, Kv1, Kv2;
		double KAngR, KAngP, KAngY;
	};

	struct RotorDrag {
		double x, y, z;
		double k_thrust_horz;
	};

    struct ThrustModel {
		bool print_flag;
        bool accurate_thrust_model_flag;
		double K1;
		double K2;
		double K3;
		double hover_percentage;
	};

    Param(){}
    ~Param(){}

    void Init(ros::NodeHandle &nh);

    double mass_;
	double gra_;
    double freq_;
    bool bodyrate_flag_;

    Gain gain_;
	RotorDrag rt_drag_;
	ThrustModel thr_model_;
};

#endif