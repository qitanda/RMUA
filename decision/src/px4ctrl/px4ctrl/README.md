# Critical HIT自主无人机：px4控制器


## 1. PX4飞控基本配置
### (1) 阅读官方文档
* [PX4用户文档](http://docs.px4.io/main/zh/)

PX4飞控**不是**开箱即用的，需要比较复杂的前期调试，推荐花半天时间把官方用户文档 `docs.px4.io` 中的 "Introduction" 至 "Flight Log Analysis"的部分粗略读一下.

### (2) 固件烧写和基础设置
1. 用`QGC`烧写最新固件或老版本稳定固件
2. 机架选择并重启.  
   250mm轴距的穿越机: Generic 250 Racer  
   330mm轴距的: DJI F330  
   450mm轴距的: Generic Quadcopter 或 DJI F450  
   其他轴距的看着来
3. 校准传感器、遥控器，其中5通道设置为飞行模式切换,7通道设置为`Emergency Stop`(推荐);
4. 修改参数  
   CBRK_IO_SAFETY = 22027  
   CBRK_USB_CHK = 197848  
   MAV_1_CONFIG = TELEM 2  
5. 如果使用支持DShot的电调（**推荐**），修改参数  
   SYS_USE_IO = 0  
   DSHOT_CONFIG = DShot** (**是购买的电调所支持的最大值)  
   测试电机转向，如果转反了，**无需重焊**，在`QGC`的 `MAVLink Console` 页面用以下指令把转向反过来即可
   ```
   dshot reverse -m 1
   dshot save -m 1
   ```
   这里的"1"是电机编号，具体接线和操作参考 https://docs.px4.io/master/en/peripherals/dshot.html  
6. 如果使用的是PWM电调，记得做油门校准，百度搜索校准方法;
7. 飞控自带的ekf2（**首选**）需要磁力计和气压计做状态估计，这两者在小飞机上受干扰大，导致状态估计和控制不稳定、精度不高，且经常需要飞控重启，但好处是可以使用定高定点控制.如希望高的控制精度和稳定性，且不想频繁重启飞控，推荐使用飞控自带的互补滤波替换ekf2，设置参数  
   SYS_MC_EST_GROUP = Q attitude estimator(no position)  
   SYS_HAS_BARO = 0  
   SYS_HAS_MAG = 0  
   随后重启即可.这一设置会导致无法启动飞控的定高定点功能，只能使用姿态控制 Stabilized 模式或者 角速度控制 Acro 模式，且两者都是油门直通的，即油门推杆直通飞控的油门输出，需要较多练习才能掌握.

### (3) 用QGC调PID参数
1. （**必须**）在Tuning页面下，调整 Hover Throttle，使得 Stabilize 模式下油门杆量为中时，飞机没有高度方向上的明显加减速.如果飞定高模式或定点模式，这一值调整的不对会导致定高时随机往上冲，比较危险. 在穿越机动力套件上，该值通常在20%附近，在较大轴距、较大重量的飞机上，该值通常在默认值50%附近.
2. （**可选但推荐**）在Tuning页面下点击"Advance"可以打开PID调参页面，按需调整PID参数，具体参考
   https://docs.px4.io/master/en/config_mc/pid_tuning_guide_multicopter.html
   https://docs.px4.io/master/en/config_mc/racer_setup.html
3. 好的控制器参数的表现：角度模式下，控制起来非常跟手，很灵敏。飞机配重到装完所有设备时候的重量，快速来回打满方向杆，飞机不会翻。
   注意:调参需要一定的飞机手控能力，因为需要先后飞角速度(Arco)模式和角度(Stabilized)模式，但如果需要高精度控制，PID调参是不可避免的.**参数没有调好，会导致快速改变姿态的时候飞机翻掉**，此时则必须进行PID调参了. PX4文档调参中的 THR_MDL_FAC参数 可改可不改， 这一参数会在后面的油门模型辨识中被辨识出来.
## 2. mavros安装与配置
mavros是PX4飞控与ROS通信的功能包，协议为mavlink，以串口形式与小电脑连接
### 控制器topic和service
用 `rqt_graph` 检查topic接收情况，确保控制器连接。

`/mavros/state`  
`/mavros/imu/data`  
`/mavros/rc/in`  
`/mavros/setpoint_raw/attitude`  
`/你的odometry`

并且检查 `/mavros/imu/data`和`/你的odometry` 的频率是否在200Hz以上，`/mavros/rc/in` 在10Hz左右.
## 3. SO3控制器
* 参考论文1：[Mellinger文章](https://ieeexplore.ieee.org/abstract/document/5980409)
* 参考论文2：[Faessler文章](https://ieeexplore.ieee.org/abstract/document/8118153)

控制器基本设计思路：位置+速度+加速度**前馈**+**重力**估计补偿控制（即PD+前馈+补偿）

**计算公式**：u = kp * e_pos + kd * e_vel + acc + gravity

## 4. 油门推力估计
为更好的悬停及提高轨迹跟踪响应，还估计了油门推力曲线

模型1(线性简易推力模型)：u = thr_acc * acc

u是油门信号值（0~1），thr_acc为斜率，acc为推力加速度

模型2(非线性精确推力模型)：F = K1 * Voltage^K2 * (K3*u^2 + (1-K3)*u)

有三个参数 K1，K2，K3， F是推力（N），Voltage是电压（V），u是油门信号值（0~1）. 这是高飞老师实验室参考PX4飞控的所用的油门模型和实际测试数据整出来的，缺少理论依据，但估计的比较准确.注意该模型需要mavros提供电压输入，对应topic为/mavros/battery，频率应为100Hz.