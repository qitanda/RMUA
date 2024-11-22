#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Eigen>

class Kalman2D
{
public:
    Kalman2D(){}
    Kalman2D(double dt, double pro_noise, double mea_noise){
        Init(dt, pro_noise, mea_noise);
    }
    ~Kalman2D(){}
    
    virtual void StateUpdate(double dt);
    virtual void PredictUpdate(double dt);
    virtual void MeasureUpdate(double x, double dt);
    
    void Init(double dt, double pro_noise, double mea_noise);
    void Reset(Eigen::Vector2d x);
    Eigen::Vector2d GetState(void);
    Eigen::Vector2d PredictWithVelocity(double dt);
    
    Eigen::Vector2d X_;
    Eigen::Matrix<double, 1, 2> H_;
    Eigen::Matrix2d F_;
    Eigen::Matrix2d Q_;
    Eigen::Matrix2d P_;
    Eigen::Matrix<double, 2, 1> K_;
    double R_;
    double pro_noise_;
};

class KalmanCV : public Kalman2D
{
public:
    KalmanCV(){}
    KalmanCV(double dt, double pro_noise, double mea_noise){
        Init(dt, pro_noise, mea_noise);
    }
    ~KalmanCV(){}

    virtual void StateUpdate(double dt);
};





// TODO: jerk，当前统计模型
class Kalman3D
{
public:
    Kalman3D(){}
    Kalman3D(double dt, double pro_noise, double mea_noise){
        Init(dt, pro_noise, mea_noise);
    }
    ~Kalman3D(){}
    
    virtual void StateUpdate(double dt);
    virtual void PredictUpdate(double dt);
    virtual void MeasureUpdate(double x, double dt);
    
    void Init(double dt, double pro_noise, double mea_noise);
    void Reset(Eigen::Vector3d x);
    Eigen::Vector3d GetState(void);
    Eigen::Vector3d PredictWithVelocity(double dt);
    Eigen::Vector3d PredictWithAcc(double dt);

    Eigen::Vector3d X_;
    Eigen::Matrix<double, 1, 3> H_;
    Eigen::Matrix3d F_;
    Eigen::Matrix3d Q_;
    Eigen::Matrix3d P_;
    Eigen::Matrix<double, 3, 1> K_;
    double R_;
    double pro_noise_;
};

class KalmanCA : public Kalman3D
{
public:
    KalmanCA(){}
    ~KalmanCA(){}

    virtual void StateUpdate(double dt);
};

class KalmanSinger : public Kalman3D
{
public:
    KalmanSinger(){}
    ~KalmanSinger(){}

    virtual void StateUpdate(double dt);
    
    void InitState(double dt, double alpha, double sigma_a2, double R);
    void ResetState(Eigen::Vector3d x);
    void NSPredictUpdate(double dt, bool flag);
    void NSPredictVelUpdate(double dt);
    void NSMeasureUpdate(double x, bool flag);

    double R_offset_;
    Eigen::Matrix3d P_offset_;
    double alpha_;     //机动频率
    double sigma_a2_;  //目标机动加速度方差

private:
    void CalcMatricQ(double dt);
};

class Kalman3D_2edge
{
public:
    Kalman3D_2edge(){}
    Kalman3D_2edge(double dt, double pro_noise, double mea_noise){
        Init(dt, pro_noise, mea_noise);
    }
    ~Kalman3D_2edge(){}
    
    virtual void StateUpdate(double dt);
    virtual void PredictUpdate(double dt);
    void MeasureUpdate(double x, double v, double dt);
    
    void Init(double dt, double pro_noise, double mea_noise);
    void Reset(Eigen::Vector3d x);
    // Eigen::Vector3d GetState(void);
    // Eigen::Vector3d PredictWithVelocity(double dt);
    // Eigen::Vector3d PredictWithAcc(double dt);

    Eigen::Vector3d X_;
    Eigen::Matrix<double, 2, 3> H_;
    Eigen::Matrix3d F_;
    Eigen::Matrix3d Q_;
    Eigen::Matrix3d P_;
    Eigen::Matrix<double, 3, 2> K_;
    Eigen::Matrix2d R_;
    double pro_noise_;
};



#endif