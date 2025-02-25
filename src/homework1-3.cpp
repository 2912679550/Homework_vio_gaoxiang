// 参考：https://blog.csdn.net/cuifeng1993/article/details/107212322
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;

int main(int argc , char **argv){
    // todo 初始化小变量w
    Eigen::Vector3d w(0.01, 0.02, 0.03); // 小量角速度
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 4 , Eigen::Vector3d(0,0,1)).toRotationMatrix(); // 表示一个绕(0,0,1)，也就是z周旋转45度的旋转矩阵
    Eigen::Quaterniond q(R); // 旋转矩阵转四元数

    cout<<"初始旋转矩阵： "<<endl<<R<<endl;
    cout<<"初始四元数： "<<endl<<q.coeffs().transpose()<<endl;

    // todo 使用第一种方法完成旋转矩阵的更新
    double theta = w.norm(); // w的模长即为旋转的角度
    Eigen::Vector3d w_unit = w.normalized(); // 归一化得到单位旋转轴
    // 构造旋转轴的反对称矩阵
    Eigen::Matrix3d w_unit_hat;
    w_unit_hat<<0 , -w_unit(2) , w_unit(1),
                w_unit(2) , 0 , -w_unit(0),
                -w_unit(1) , w_unit(0) , 0;
    // 使用罗德里格斯公式更新旋转矩阵
    Eigen::Matrix3d R_w = cos(theta) * Eigen::Matrix3d::Identity() + (1 - cos(theta)) * w_unit * w_unit.transpose() + sin(theta) * w_unit_hat;
    Eigen::Matrix3d R_updated = R * R_w;
    cout<<"更新后的旋转矩阵： "<<endl<<R_updated<<endl;

    // todo 使用第二种方式基于四元数完成对旋转矩阵的更新
    Eigen::Quaterniond q_w(1, w(0) / 2, w(1) / 2, w(2) / 2); // 旋转向量转四元数
    Eigen::Quaterniond q_updated = q * q_w; // 四元数相乘
    q_updated.normalize(); // 归一化，单位四元数才可以表示三维旋转
    cout<<"更新后的四元数： "<<endl<<q_updated.coeffs().transpose()<<endl;

    // todo 校核，计算两种方式得到的结果之差
    Eigen::Matrix3d R_diff = R_updated - q_updated.toRotationMatrix();
    cout<<"两种方式得到的旋转矩阵之差： "<<endl<<R_diff<<endl;

    return 0;
}
