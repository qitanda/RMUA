#include "param.h"

void Param::Init(ros::NodeHandle &nh)
{
    nh.param("/px4ctrl_node/mass", mass_, 1.0);
    nh.param("/px4ctrl_node/gra", gra_, 1.0);
    nh.param("/px4ctrl_node/freq", freq_, 100.0);
    nh.param("/px4ctrl_node/bodyrate_flag", bodyrate_flag_, false);
    

    nh.param("/px4ctrl_node/gain/Kp0", gain_.Kp0, 1.0);
    nh.param("/px4ctrl_node/gain/Kp1", gain_.Kp1, 1.0);
    nh.param("/px4ctrl_node/gain/Kp2", gain_.Kp2, 1.0);
    nh.param("/px4ctrl_node/gain/Kv0", gain_.Kv0, 1.0);
    nh.param("/px4ctrl_node/gain/Kv1", gain_.Kv1, 1.0);
    nh.param("/px4ctrl_node/gain/Kv2", gain_.Kv2, 1.0);
    nh.param("/px4ctrl_node/gain/KAngR", gain_.KAngR, 1.0);
    nh.param("/px4ctrl_node/gain/KAngP", gain_.KAngP, 1.0);
    nh.param("/px4ctrl_node/gain/KAngY", gain_.KAngY, 1.0);

    nh.param("/px4ctrl_node/rotor_drag/x", rt_drag_.x, 0.0);
    nh.param("/px4ctrl_node/rotor_drag/y", rt_drag_.y, 0.0);
    nh.param("/px4ctrl_node/rotor_drag/z", rt_drag_.z, 0.0);
    nh.param("/px4ctrl_node/rotor_drag/k_thrust_horz", rt_drag_.k_thrust_horz, 0.0);

    nh.param("/px4ctrl_node/thrust_model/print_value", thr_model_.print_flag, false);
    nh.param("/px4ctrl_node/thrust_model/accurate_thrust_model", thr_model_.accurate_thrust_model_flag, false);
    nh.param("/px4ctrl_node/thrust_model/K1", thr_model_.K1, 0.0);
    nh.param("/px4ctrl_node/thrust_model/K2", thr_model_.K2, 0.0);
    nh.param("/px4ctrl_node/thrust_model/K3", thr_model_.K3, 0.0);
    nh.param("/px4ctrl_node/thrust_model/hover_percentage", thr_model_.hover_percentage, 0.0);

    // std::cout << "gain/Kp:" << gain_.Kp0 << " " << gain_.Kp1 << " " << gain_.Kp2 << std::endl;
    // std::cout << "gain/Kv:" << gain_.Kv0 << " " << gain_.Kv1 << " " << gain_.Kv2 << std::endl;
    // std::cout << "gain/KAng:" << gain_.KAngR << " " << gain_.KAngP << " " << gain_.KAngY << std::endl;
}
