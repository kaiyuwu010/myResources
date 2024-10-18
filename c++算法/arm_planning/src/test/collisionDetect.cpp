#include "arm_planning/collisionDetect.h"

void collisionDetectTest() {
    clock_t start, end;
    CollisionDetection cd;                                                    // 45度0.785398163；90度1.570796327；
    std::vector<double> theta_input{0, -1.570796327, -1.570796327, 0, 0, 0};  //机械臂初始位姿
    bool a;
    start = clock();
    a = cd.isStateValid(theta_input);
    end = clock();
    double elapsedTime = static_cast<double>(end - start) / 1000;
    std::cout << "运行时间为:" << elapsedTime << "ms" << std::endl;
    std::cout << "第一段碰撞检测结果(1不碰撞;0碰撞):" << a << std::endl;
}

int main(int argc, char** argv) {
    collisionDetectTest();
    return 0;
}
