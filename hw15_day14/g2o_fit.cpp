#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_factory.h>


// 自定义顶点
class VertexParameters : public g2o::BaseVertex<4, Eigen::Vector4d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override {
        _estimate << 1.0, 1.0, 0.0, 0.0;
    }

    void oplusImpl(const double* update) override {
        _estimate += Eigen::Map<const Eigen::Vector4d>(update);
    }

    bool read(std::istream& /*is*/) override { return false; }

    bool write(std::ostream& /*os*/) const override { return false; }
};

// 自定义边
class EdgeResidual : public g2o::BaseUnaryEdge<1, double, VertexParameters> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeResidual(double t, double y)
        : t_(t), y_(y) {}

    void computeError() override {
        // TODO: 完成这个函数（提示：需要对 _error[0] 进行赋值）
        auto v = dynamic_cast<const VertexParameters*>(_vertices[0]);
        const Eigen::Vector4d esti = v->estimate();
        _error[0] = y_ - (esti[0] * std::sin(esti[1] * t_ + esti[2]) + esti[3]);
    }

    void linearizeOplus() override {
        // TODO: 完成这个函数（提示：需要对 _jacobianOplusXi 进行赋值）
        auto v = dynamic_cast<const VertexParameters*>(_vertices[0]);
        const Eigen::Vector4d esti = v->estimate();
        _jacobianOplusXi[0] = -std::sin(esti[1] * t_ + esti[2]);
        _jacobianOplusXi[1] = -esti[0] * t_ * std::cos(esti[1] * t_ + esti[2]);
        _jacobianOplusXi[2] = -esti[0] * std::cos(esti[1] * t_ + esti[2]);
        _jacobianOplusXi[3] = -1;
    }

    bool read(std::istream& /*is*/) override { return false; }

    bool write(std::ostream& /*os*/) const override { return false; }

    double t_;
    double y_;
};

int main() {
    // 从文件中读取数据
    std::vector<double> t_values;
    std::vector<double> y_values;

    std::ifstream file("../data.txt");  // 注意修改为自己的文件路径
    if (!file) {
        std::cerr << "Error opening file." << std::endl;
        return 1;
    }

    // 读取数据到程序中
    double t, y;
    while (file >> t >> y) {
        t_values.push_back(t);
        y_values.push_back(y);
    }

    // 构建g2o优化器
    g2o::SparseOptimizer optimizer;
    auto linearSolver =
            std::make_unique<g2o::LinearSolverDense<g2o::BlockSolver< g2o::BlockSolverTraits<4, 1> >::PoseMatrixType> >();
    auto blockSolver =
            std::make_unique<g2o::BlockSolver< g2o::BlockSolverTraits<4, 1> > >(std::move(linearSolver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    optimizer.setAlgorithm(algorithm);

    // 添加顶点
    VertexParameters* parameters = new VertexParameters();
    /***    开始（添加顶点）    ***/
    parameters->setToOriginImpl();
    parameters->setId(0);
    optimizer.addVertex(parameters);
    /***    结束（添加顶点）    ***/ 


    // 添加边
    for (size_t i = 0; i < t_values.size(); ++i) {
        /***    开始（添加边）    ***/ 
        auto edge = new EdgeResidual(t_values[i], y_values[i]);
        edge->setId(i);
        edge->setVertex(0, parameters);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        optimizer.addEdge(edge);
        /***    结束（添加边）    ***/ 
    }

    // 开始优化
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // 打印优化结果
    std::cout << "Optimization results:" << std::endl;
    std::cout << "Estimated parameters: " << parameters->estimate().transpose() << std::endl;

    return 0;
}