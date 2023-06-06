#include <Eigen/Dense>
#include <iostream>
#include <cmath>

int main() {
    // 定义一个点在世界坐标系中的坐标P_w
    Eigen::Vector4f P_w(1.0, 2.0, 3.0, 1.0);

    // 定义机器人在世界坐标系中的位置t_wr
    Eigen::Vector3f t_wr(4.0, 5.0, 6.0);

    // --------- 开始你的代码	---------------//


    // 构建旋转矩阵R_wc
    Eigen::AngleAxisf rotation_vector1(45 * M_PI / 180, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotation_vector2(30 * M_PI / 180, Eigen::Vector3f::UnitY());
    auto rotation_vector3 = rotation_vector2 * rotation_vector1;
    auto R_wc = rotation_vector3.toRotationMatrix();

    // 构建变换矩阵T_wc
    Eigen::Isometry3f T_wc = Eigen::Isometry3f::Identity();
    T_wc.rotate(R_wc);
    T_wc.pretranslate(t_wr);

    // 计算点在机器人坐标系中的坐标P_c = T_cw * P_w
    auto P_c = T_wc.inverse() * P_w;

    // --------- 结束你的代码	---------------//

    std::cout << "The point in the robot coordinate system is: \n" << P_c << std::endl;

    return 0;
}
