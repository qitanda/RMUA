#include "kalman_filter.h"

/*--------------------- Kalman2D ---------------------*/
void Kalman2D::Init(double dt, double pro_noise, double mea_noise)
{
    X_ << 0, 0;
    H_ << 1, 0;
    
    pro_noise_ = pro_noise * pro_noise;
    StateUpdate(dt);

    R_ = mea_noise * mea_noise;
    P_ << R_, 0,
          0,  R_;
}

void Kalman2D::Reset(Eigen::Vector2d x)
{
    X_ = x;
    P_ << R_, 0,
          0,  R_;
}

Eigen::Vector2d Kalman2D::GetState(void)
{
    return X_;
}

Eigen::Vector2d Kalman2D::PredictWithVelocity(double dt)
{
    Eigen::Matrix2d transform;
    transform << 1, dt, 
                 0,  1;
    return transform*X_;
}

void Kalman2D::StateUpdate(double dt)
{
    F_ << 1, dt,
          0,  1;
    
    Q_ << pro_noise_, 0,
          0,          pro_noise_;
}

void Kalman2D::PredictUpdate(double dt)
{
    StateUpdate(dt);
    X_ = F_ * X_;
    P_ = F_*P_*(F_.transpose()) + Q_;    
}

void Kalman2D::MeasureUpdate(double x, double dt)
{
    PredictUpdate(dt);
    double S = H_*P_*(H_.transpose()) + R_;
    K_ = P_*H_.transpose()*(1/S);
    X_ = X_ + K_ * (x-X_(0));
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    P_ = (I-K_*H_) * P_;
}
/*-----------------------------------------------------*/



/*--------------------- KalmanCV ---------------------*/
void KalmanCV::StateUpdate(double dt)
{
    F_ << 1, dt,
          0,  1;
    
    Q_ << std::pow(dt,3)/3.0, dt*dt/2.0,
          dt*dt/2.0,          dt;
    Q_ = Q_ * pro_noise_;
}
/*-----------------------------------------------------*/



/*--------------------- Kalman3D ---------------------*/
void Kalman3D::Init(double dt, double pro_noise, double mea_noise)
{
    X_ << 0, 0, 0;
    H_ << 1, 0, 0;
    
    pro_noise_ = pro_noise * pro_noise;
    StateUpdate(dt);

    R_ = mea_noise * mea_noise;
    P_ << R_, 0,  0,
          0,  R_, 0,
          0,  0,  R_;
}

void Kalman3D::Reset(Eigen::Vector3d x)
{
    X_ = x;
    P_ << R_, 0,  0,
          0,  R_, 0,
          0,  0,  R_;
}

Eigen::Vector3d Kalman3D::GetState(void)
{
    return X_;
}

Eigen::Vector3d Kalman3D::PredictWithVelocity(double dt)
{
    Eigen::Matrix3d transform;
    transform << 1, dt, 0,
                 0,  1, 0,
                 0,  0, 1;
    return transform*X_;
}

Eigen::Vector3d Kalman3D::PredictWithAcc(double dt)
{
    Eigen::Matrix3d transform;
    transform << 1, dt, dt*dt/2,
                 0,  1, dt,
                 0,  0, 1;
    return transform*X_;
}

void Kalman3D::StateUpdate(double dt)
{
    F_ << 1, dt, dt*dt/2,
          0,  1,      dt,
          0,  0,       1;    
    
    Q_ << 1, 0, 0,
          0, 1, 0,
          0, 0, 1;
    Q_ = Q_ * pro_noise_;
}

void Kalman3D::PredictUpdate(double dt)
{
    StateUpdate(dt);
    X_ = F_ * X_;
    P_ = F_*P_*(F_.transpose()) + Q_;
}

void Kalman3D::MeasureUpdate(double x, double dt)
{
    PredictUpdate(dt);
    double S = H_*P_*(H_.transpose()) + R_;
    K_ = P_*H_.transpose()*(1/S);
    X_ = X_ + K_ * (x-X_(0));
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    P_ = (I-K_*H_) * P_;
}
/*-----------------------------------------------------*/



/*--------------------- KalmanCA ---------------------*/
void KalmanCA::StateUpdate(double dt)
{
    F_ << 1, dt, dt*dt/2,
          0,  1,      dt,
          0,  0,       1;    
    
    double q11 = std::pow(dt, 6)/36;
    double q12 = std::pow(dt, 5)/12;
    double q13 = std::pow(dt, 4)/6;
    double q22 = std::pow(dt, 4)/4;
    double q23 = std::pow(dt, 3)/2;
    double q33 = std::pow(dt, 2);

    Q_ << q11, q12, q13,
          q12, q22, q23,
          q13, q23, q33;
    Q_ = Q_ * pro_noise_;
}
/*-----------------------------------------------------*/


/*--------------------- KalmanSinger ---------------------*/
void KalmanSinger::InitState(double dt, double alpha, double sigma_a2, double R)
{
    alpha_ = alpha;
    sigma_a2_ = sigma_a2;
    H_ << 1, 0, 0;
    R_offset_ = R;
    P_offset_ << R,    0, 0,
                 0, R/dt, 0,
                 0,    0, R/dt/dt;
    
    Init(dt, 0, std::sqrt(R));
}

void KalmanSinger::CalcMatricQ(double dt)
{
    double q11 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dt) + 2*alpha_*dt + 2/3*std::pow(alpha_, 3)*std::pow(dt, 3) - 2*std::pow(alpha_, 2)*std::pow(dt, 2) - 4*alpha_*dt*exp(-alpha_*dt)) / (2*std::pow(alpha_, 5));
    double q12 = 2*alpha_*sigma_a2_*(1 + std::exp(-2*alpha_*dt) - 2*std::exp(-alpha_*dt) + 2*alpha_*dt*exp(-alpha_*dt) - 2*alpha_*dt + std::pow(alpha_, 2)*std::pow(dt, 2)) / (2*std::pow(alpha_, 4));
    double q13 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dt) - 2*alpha_*dt*std::exp(-alpha_*dt)) / (2*std::pow(alpha_, 3));
    double q22 = 2*alpha_*sigma_a2_*(4*std::exp(-alpha_*dt) - 3 - std::exp(-2*alpha_*dt) + 2*alpha_*dt) / (2*std::pow(alpha_, 3));
    double q23 = 2*alpha_*sigma_a2_*(1 + std::exp(-2*alpha_*dt) - 2*std::exp(-alpha_*dt)) / (2*std::pow(alpha_, 2));
    double q33 = 2*alpha_*sigma_a2_*(1 - std::exp(-2*alpha_*dt)) / (2*alpha_);

    Q_ << q11, q12, q13,
          q12, q22, q23,
          q13, q23, q33;
}

void KalmanSinger::ResetState(Eigen::Vector3d x)
{
    X_ = x;
    R_ = R_offset_;
    P_ = P_offset_;
}

void KalmanSinger::StateUpdate(double dt)
{
    F_ << 1, dt, (alpha_*dt + std::exp(-alpha_*dt) - 1) / (std::pow(alpha_, 2)),
          0,  1,                         (1 - std::exp(-alpha_*dt))/alpha_,
          0,  0,                                      std::exp(-alpha_*dt);

    CalcMatricQ(dt);
}

void KalmanSinger::NSPredictUpdate(double dt, bool flag)
{
    if(!flag){
        X_.y() = 0;
        X_.z() = 0;
    }
    StateUpdate(dt);
    X_ = F_ * X_;
    P_ = F_*P_*(F_.transpose()) + Q_;
}

void KalmanSinger::NSPredictVelUpdate(double dt)
{
    F_ << 1, dt, 0,
          0,  1, 0,
          0,  0, 1;

    CalcMatricQ(dt);
    X_ = F_ * X_;
    P_ = F_*P_*(F_.transpose()) + Q_;
}

void KalmanSinger::NSMeasureUpdate(double x, bool flag)
{
    Eigen::Vector3d X_hat = X_;
    P_ = (H_.transpose() * (1 / R_) * H_ + P_.inverse()).inverse();
    K_ = P_*H_.transpose() * (1 / R_);
    X_ = X_ + K_*(x - H_*X_);

    if(flag) X_.z() = X_hat.z();
}
/*-----------------------------------------------------*/

/*--------------------- Kalman3D_2edge ---------------------*/
void Kalman3D_2edge::Init(double dt, double pro_noise, double mea_noise)
{
    X_ << 0, 0, 0;
    H_ << 1, 0, 0;
    
    pro_noise_ = pro_noise * pro_noise;
    StateUpdate(dt);

    double r = mea_noise * mea_noise;
    R_ = r * Eigen::Matrix2d::Identity();
    P_ << r, 0,  0,
          0,  r, 0,
          0,  0,  r;
}

void Kalman3D_2edge::Reset(Eigen::Vector3d x)
{
    X_ = x;
    
}

void Kalman3D_2edge::StateUpdate(double dt)
{
    F_ << 1, dt, dt*dt/2,
          0,  1,      dt,
          0,  0,       1;    
    
    double q11 = std::pow(dt, 6)/36;
    double q12 = std::pow(dt, 5)/12;
    double q13 = std::pow(dt, 4)/6;
    double q22 = std::pow(dt, 4)/4;
    double q23 = std::pow(dt, 3)/2;
    double q33 = std::pow(dt, 2);

    Q_ << q11, q12, q13,
          q12, q22, q23,
          q13, q23, q33;
    Q_ = Q_ * pro_noise_;
}

void Kalman3D_2edge::PredictUpdate(double dt)
{
    StateUpdate(dt);
    X_ = F_ * X_;
    P_ = F_*P_*(F_.transpose()) + Q_;
}

void Kalman3D_2edge::MeasureUpdate(double x, double v, double dt)
{
    PredictUpdate(dt);
    Eigen::Matrix2d S;
    S = H_*P_*(H_.transpose()) + R_;
    K_ = P_*H_.transpose()*S.inverse();
    Eigen::Matrix<double, 2, 1> E;
    E << x-X_(0), v-X_(1);
    X_ = X_ + K_ * E;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    P_ = (I-K_*H_) * P_;
}

/*-----------------------------------------------------*/


