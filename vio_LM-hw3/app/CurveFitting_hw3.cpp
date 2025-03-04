#include <iostream>
#include <random>
#include "backend/problem.h"
#include <mgl2/fltk.h>
#include <vector>

using namespace myslam::backend;
using namespace std;

// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex: public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingVertex(): Vertex(3) {}  // abc: 三个参数， Vertex 是 3 维的
    virtual std::string TypeInfo() const { return "abc"; }
}; 

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge: public Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x, double y ): Edge(1,1, std::vector<std::string>{"abc"}) {
        x_ = x;
        y_ = y;
    }
    // 计算曲线模型误差
    virtual void ComputeResidual() override
    {
        Vec3 abc = verticies_[0]->Parameters();  // 估计的参数
        residual_(0) =  abc(0)*x_*x_ + abc(1)*x_ + abc(2)  - y_;  // 构建残差
    }

    // 计算残差对变量的雅克比
    virtual void ComputeJacobians() override
    {
        Vec3 abc = verticies_[0]->Parameters();

        Eigen::Matrix<double, 1, 3> jaco_abc;  // 误差为1维，状态量 3 个，所以是 1x3 的雅克比矩阵
        jaco_abc << x_ * x_ , x_  , 1 ;
        jacobians_[0] = jaco_abc;
    }
    /// 返回边的类型信息
    virtual std::string TypeInfo() const override { return "CurveFittingEdge"; }
public:
    double x_,y_;  // x 值， y 值为 _measurement
};

int main()
{
    double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N = 1000;                          // 数据点
    double w_sigma= 1.;                 // 噪声Sigma值

    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.,w_sigma);

    // 构建 problem
    Problem problem(Problem::ProblemType::GENERIC_PROBLEM);
    shared_ptr< CurveFittingVertex > vertex(new CurveFittingVertex());

    // 设定待估计参数 a, b, c初始值
    vertex->SetParameters(Eigen::Vector3d (0.,0.,0.));
    // 将待估计的参数加入最小二乘问题
    problem.AddVertex(vertex);

    // todo 生成 N 次观测数据
    for (int i = 0; i < N; ++i) {

        double x = i/100.;
        double n = noise(generator);
        // 观测 y
        double y = a*x*x + b*x + c  + n;
//        double y = std::exp( a*x*x + b*x + c );

        // 每个观测对应的残差函数
        shared_ptr< CurveFittingEdge > edge(new CurveFittingEdge(x,y));
        std::vector<std::shared_ptr<Vertex>> edge_vertex;
        edge_vertex.push_back(vertex);
        edge->SetVertex(edge_vertex);

        // 把这个残差添加到最小二乘问题
        problem.AddEdge(edge);
    }

    std::cout<<"\nTest CurveFitting start..."<<std::endl;
    // TODO 使用 LM 求解
    problem.Solve(60);

    std::cout << "-------After optimization, we got these parameters :" << std::endl;
    std::cout << vertex->Parameters().transpose() << std::endl;
    std::cout << "-------ground truth: " << std::endl;
    std::cout << "1.0,  2.0,  1.0" << std::endl;

    // todo 画图
    std::vector<double> x_data;
    mglFLTK gr;
    // x轴为lambda的标号，y轴为lambda的值
    for (int i = 0; i < problem.lambdas.size(); ++i) {
        x_data.push_back(i);
    }
    mglData x_mgl(x_data.size(), &x_data[0]);
    mglData y_fit_mgl(problem.lambdas.size(), &problem.lambdas[0]);

    // 调整x轴刻度范围为1到10，每隔1显示一个刻度
    gr.SetRanges(0, problem.lambdas.size(), 0, 20); // 必须在Plot与axis之前设置
    gr.Axis();
    gr.Plot(x_mgl, y_fit_mgl, "r");
    gr.SetTicks('x', 1, 0);
    gr.Label('x', "x", 0);
    gr.Label('y', "lambda", 0);
    // gr.Title("Curve Fitting");
    gr.Legend(2);
    gr.Run();
    // std
    return 0;
}


