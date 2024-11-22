cd ../../IntelligentUAVChampionshipSimulator
./run_simulator.sh & sleep 3;
cd ../Desktop/vio
source devel/setup.zsh & sleep 1;
roslaunch vins_estimator uav2022.launch & sleep 1;
cd ../../zj_ws/drone_ws
source devel/setup.zsh & sleep 1;
rosrun critical_nn_deploy stereo_depth_node & sleep 1;

wait;
