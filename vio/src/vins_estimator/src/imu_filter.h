#pragma once

#include <Eigen/Core>
#include <deque>

// #define Sample_IMU 20
// float gyro_x[Sample_IMU] = {0}, gyro_y[Sample_IMU] = {0}, gyro_z[Sample_IMU] = {0};
// float gyro_x_last = 0.0f, gyro_y_last = 0.0f, gyro_z_last = 0.0f;
// float acc_x[Sample_IMU] = {0}, acc_y[Sample_IMU] = {0}, acc_z[Sample_IMU] = {0};
// float acc_x_last = 0.0f, acc_y_last = 0.0f, acc_z_last = 0.0f;
// double gain = 0.0;

class ImuFilter {
 public:
  ImuFilter() = default;

  void process(Eigen::Vector3d& acc, Eigen::Vector3d& gyro);

 private:
  Eigen::Vector3d last_acc_, last_gyro_;
  bool init_ = false;
};

// void ICM42688_read(IMU_t *imu, uint8_t reset, uint8_t temp_init)
// {
//   icm42688_read_bytes(ICM42688_TEMP_DATA1, imu->imu_data, 14);
//   imu->temp = (int16_t)(imu->imu_data[0] << 8 | imu->imu_data[1]) / 132.48 + 25;
//   int16_t accx, accy, accz, gyrox, gyroy, gyroz;
//   accx  = (imu->imu_data[2]  << 8) | imu->imu_data[3];
//   accy  = (imu->imu_data[4]  << 8) | imu->imu_data[5];
//   accz  = (imu->imu_data[6]  << 8) | imu->imu_data[7];
//   gyrox = (imu->imu_data[8]  << 8) | imu->imu_data[9];
//   gyroy = (imu->imu_data[10] << 8) | imu->imu_data[11];
//   gyroz = (imu->imu_data[12] << 8) | imu->imu_data[13];
//   if (temp_init) {
//   imu->other.gyroBiasFound = processGyroBias(reset, imu, gyrox, gyroy, gyroz);
//   if(imu->other.gyroBiasFound) processAccScale(accx, accy, accz, &imu->other); /* accScale */
//   }
//   float gyro_sensitivity = 16.384f; // gyro sensitivity = 32768 / 2000
//   float acc_sensitivity  = 8192;    // acc  sensitivity = 32768 / 4
//   float gyro_x_cur = ((float)(gyrox - imu->gyroBias.x) / gyro_sensitivity) * (1-gain) + gain * gyro_x_last;
//   float gyro_y_cur = ((float)(gyroy - imu->gyroBias.y) / gyro_sensitivity) * (1-gain) + gain * gyro_y_last;
//   float gyro_z_cur = ((float)(gyroz - imu->gyroBias.z) / gyro_sensitivity) * (1-gain) + gain * gyro_z_last;
//   float acc_x_cur = ((float)accx / acc_sensitivity) * (1-gain) + gain * acc_x_last;
//   float acc_y_cur = ((float)accy / acc_sensitivity) * (1-gain) + gain * acc_y_last;
//   float acc_z_cur = ((float)accz / acc_sensitivity) * (1-gain) + gain * acc_z_last;

//   static int index = 0;
//   index++;
//   if (index >= Sample_IMU) {
//   index = 0;
//   }
//   gyro_x[index] = gyro_x_cur;
//   gyro_y[index] = gyro_y_cur;
//   gyro_z[index] = gyro_z_cur;
//   acc_x[index] = acc_x_cur;
//   acc_y[index] = acc_y_cur;
//   acc_z[index] = acc_z_cur;
//   double gyro_x_sum = 0.0f, gyro_y_sum = 0.0f, gyro_z_sum = 0.0f, acc_x_sum = 0.0f, acc_y_sum = 0.0f, acc_z_sum = 0.0f;
//   for (int i = 0; i < Sample_IMU; i++) {
//   gyro_x_sum += gyro_x[i];
//   gyro_y_sum += gyro_y[i];
//   gyro_z_sum += gyro_z[i];
//   acc_x_sum += acc_x[i];
//   acc_y_sum += acc_y[i];
//   acc_z_sum += acc_z[i];
//   }
//   imu->gyro.x = gyro_x_sum / (float)Sample_IMU;
//   imu->gyro.y = gyro_y_sum / (float)Sample_IMU;
//   imu->gyro.z = gyro_z_sum / (float)Sample_IMU;
//   imu->acc.x = acc_x_sum / (float)Sample_IMU;
//   imu->acc.y = acc_y_sum / (float)Sample_IMU;
//   imu->acc.z = acc_z_sum / (float)Sample_IMU;
//   // imu->gyro.x = (float)(gyrox - imu->gyroBias.x) / gyro_sensitivity;
//   // imu->gyro.y = (float)(gyroy - imu->gyroBias.y) / gyro_sensitivity;
//   // imu->gyro.z = (float)(gyroz - imu->gyroBias.z) / gyro_sensitivity;
//   // imu->acc.x = (float)(accx) / acc_sensitivity;
//   // imu->acc.y = (float)(accy) / acc_sensitivity;
//   // imu->acc.z = (float)(accz) / acc_sensitivity;
//   gyro_x_last = gyro_x_cur;
//   gyro_y_last = gyro_y_cur;
//   gyro_z_last = gyro_z_cur;
//   acc_x_last = acc_x_cur;
//   acc_y_last = acc_y_cur;
//   acc_z_last = acc_z_cur;
// }