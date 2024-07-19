#include <Eigen/Dense>
#include <iostream>

/*
解算AX=XB方程步骤:
1.方程两边同乘旋转矩阵B的旋转向量Vb得到: AX*Vb=λ*X*Vb
2.由于X*Vb也是一个向量，并且旋转矩阵A也存在旋转轴，假设A的旋转向量为Va，可以得到: X*Vb=λ*Va
3.只要求出三个相互独立的Vb就可以得到: X*[Vb1、Vb2、Vb3] = λ*[Va1、Va2、Va3] ==> X = λ*[Va1、Va2、Va3]*（[Vb1、Vb2、Vb3]的逆）
*/
class AXEqualXBSolution
{
    AXEqualXBSolution(){}
    Eigen::Matrix3d getX(std::array<Eigen::Matrix3d, 3>& A, std::array<Eigen::Matrix3d, 3>& B)
    {
        Eigen::Matrix3d X;
        Eigen::Matrix3d Va;
        Eigen::Matrix3d Vb;
        //创建EigenSolver对象，计算旋转矩阵的旋转轴(特征向量)
        for(size_t i=0; i<3; i++)
        {
            Eigen::EigenSolver<Eigen::Matrix3d> solverA(A[i]);
            Eigen::EigenSolver<Eigen::Matrix3d> solverB(B[i]);
            if (solverA.info() != Eigen::Success || solverB.info() != Eigen::Success) {
                std::cerr << "Eigenvalue decomposition failed!" << std::endl;
            }
            //计算A矩阵的特征值和特征向量
            Eigen::Vector3cd eigenvaluesA = solverA.eigenvalues();
            Eigen::Matrix3cd eigenvectorsA = solverA.eigenvectors();
            std::cout << "Eigenvectors a :\n" <<i<< eigenvalues << std::endl;
            //计算B矩阵的特征值和特征向量
            Eigen::Vector3cd eigenvaluesB = solverB.eigenvalues();
            Eigen::Matrix3cd eigenvectorsB = solverB.eigenvectors();
            std::cout << "Eigenvectors b :\n" <<i<< eigenvectors << std::endl;
            //提取实数特征值及其对应的特征向量
            if (eigenvalues[j].imag() == 0) { 
                Va.col[i] = eigenvectorsA.col(j).real();
                Vb.col[i] = eigenvectorsB.col(j).real();
                std::cout << "Real Eigenvalue: " <<i<< eigenvalues[i].real() << std::endl;
                std::cout << "Corresponding Eigenvector:\n" <<i<< eigenvectors.col(i).real() << std::endl;
            }
        }
        X = Va*Vb.inverse();
        std::cout << "x is: " << X << std::endl;
        return X;
    }
};

int main(int argc, char **argv)
{
    // 定义一个 3D 旋转矩阵（绕 z 轴旋转 45 度）
    double angle = M_PI / 4; // 45度
    Eigen::Matrix3d rot;
    rot << cos(angle)+0.0001, -sin(angle)-0.1, 0,
           sin(angle)+0.001, cos(angle),  0,
           0,          0,           1+0.001;
    std::cout << "rot:\n" << rot << std::endl;
    Eigen::EigenSolver<Eigen::Matrix3d> solver(rot);
    if (solver.info() != Eigen::Success) {
        std::cerr << "Eigenvalue decomposition failed!" << std::endl;
        return -1;
    }
    Eigen::Vector3cd eigenvalues = solver.eigenvalues();
    std::cout << "Eigenvalues:\n" << eigenvalues << std::endl;
    Eigen::Matrix3cd eigenvectors = solver.eigenvectors();
    std::cout << "Eigenvectors:\n" << eigenvectors << std::endl;
    // 提取实数特征值及其对应的特征向量
    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues[i].imag() == 0) { // 检查是否为实数
            std::cout << "Real Eigenvalue: " << eigenvalues[i].real() << std::endl;
            std::cout << "Corresponding Eigenvector:\n" << eigenvectors.col(i).real() << std::endl;
        }
    }
    return 0;
}
