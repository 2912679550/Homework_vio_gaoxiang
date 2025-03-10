//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t):Rwc(R),qwc(R),twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;    // 这帧图像观测到的特征坐标
};
int main()
{
    // 生成初始仿真数据
    int poseNums = 10;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for(int n = 0; n < poseNums; ++n ) {
        double theta = n * 2 * M_PI / ( poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R,t));
    }

    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);
    // 这个特征从第三帧相机开始被观测，i=3
    int start_frame_id = 0;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        camera_pose[i].uv = Eigen::Vector2d(x/z,y/z);
    }
    
    //  TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est;           // 结果保存到这个变量
    P_est.setZero();
    // ! /* your code begin */
    // 构造三角化超定方程的D矩阵
    Eigen::MatrixXd D(2*(end_frame_id-start_frame_id),4);
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        // 构造投影矩阵
        Eigen::Matrix<double, 3, 4> P;
        P.block(0, 0 ,3 ,3 ) = Rcw;
        P.block(0, 3 ,3 ,1 ) = -Rcw * camera_pose[i].twc;
        // 使用像素坐标填充D矩阵
        D.block(2*(i-start_frame_id), 0, 1, 4) = camera_pose[i].uv.x() * P.row(2) - P.row(0);
        D.block(2*(i-start_frame_id)+1, 0, 1, 4) = camera_pose[i].uv.y() * P.row(2) - P.row(1);
    }
    std::cout<< "D: \n" << D << std::endl;
    // 求解超定方程
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D.transpose() * D, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    P_est = U.block<3, 1>(0, 3) / U(3, 3);//取齐次坐标的前三维，并同时除以第四维坐标
    // ! /* your code end */
    
    std::cout <<"ground truth: \n"<< Pw.transpose() <<std::endl;
    std::cout <<"your result: \n"<< P_est.transpose() <<std::endl;
    // TODO:: 请如课程讲解中提到的判断三角化结果好坏的方式，绘制奇异值比值变化曲线
    // ! /* your code begin */
    // 获取SVD分解后的奇异值
    Eigen::VectorXd singular_values = svd.singularValues();
    // 计算奇异值比值（sigma4 / sigma3 越接近0越好）
    double ratio = singular_values(3) / singular_values(2);
    std::cout << "sigma4 / sigma3: " << ratio << std::endl;
    // ! /* your code end */
    return 0;
}
