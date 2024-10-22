#include <Eigen/Dense>
#include <iostream>
#include <array>
/*
眼在手方程:
1.由位姿关系可以得到: Tend_to_base1 * Tcam_to_hand * Tobj_to_cam1 = Tend_to_base2 * Tcam_to_hand * Tobj_to_cam2
2.由上式可以得到: (Tend_to_base2的逆）* Tend_to_base1 * Tcam_to_hand = Tcam_to_hand * Tobj_to_cam2 * (Tobj_to_cam1的逆)
3.令(Tend_to_base2的逆）* Tend_to_base1 = Ta，Tobj_to_cam2 * (Tobj_to_cam1的逆) = Tb，Tcam_to_hand = Tx，可以得到方程:TaTx=TxTb
4.该方程可以把位置部分提取出来，另变换矩阵X、A、B的位置向量分别为Px、Pa、Pb，旋转矩阵分别为X、A、B，可以得到:A*Px+Pa = X*Pb+Px
5.可以得到X的位置部分为: (A-I)*Px = (X*Pb-Pa) ==> Px = ((A-I)的逆)*(X*Pb-Pa)
解算旋转矩阵部分的AX=XB方程步骤:
1.方程两边同乘旋转矩阵B的旋转向量Vb得到: AX*Vb=λ*X*Vb
2.由于X*Vb也是一个向量，并且旋转矩阵A也存在旋转轴，假设A的旋转向量为Va，可以得到: X*Vb=λ*Va
3.只要求出三个相互独立的Vb就可以得到: X*[Vb1、Vb2、Vb3] = λ*[Va1、Va2、Va3] ==> X = λ*[Va1、Va2、Va3]*（[Vb1、Vb2、Vb3]的逆）
*/
class AXEqualXBSolution
{
    AXEqualXBSolution(){}
    //求解AX=XB的旋转部分
    Eigen::Matrix3d getX(std::array<Eigen::Matrix3d, 3>& A, std::array<Eigen::Matrix3d, 3>& B)
    {
        Eigen::Matrix3d X;
        Eigen::Matrix3d Va = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d Vb = Eigen::Matrix3d::Zero();
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
            std::cout << "Eigenvectors a :\n" <<i<< eigenvaluesA << std::endl;
            //计算B矩阵的特征值和特征向量
            Eigen::Vector3cd eigenvaluesB = solverB.eigenvalues();
            Eigen::Matrix3cd eigenvectorsB = solverB.eigenvectors();
            std::cout << "Eigenvectors b :\n" <<i<< eigenvaluesB << std::endl;
            //提取实数特征值及其对应的特征向量
            for(size_t j=0; j<eigenvaluesA.size(); j++)
            {
                if (eigenvaluesA[j].imag() == 0) { 
                    Va.col(i) = eigenvectorsA.col(j).real();
                    std::cout << "Real Eigenvalue: " <<i<< eigenvaluesA[i].real() << std::endl;
                    break;
                }
                if (eigenvaluesB[j].imag() == 0) { 
                    Vb.col(i) = eigenvectorsB.col(j).real();
                    std::cout << "Real Eigenvalue: " <<i<< eigenvaluesB[i].real() << std::endl;
                    break;
                }                
            }

        }
        X = Va*Vb.inverse();
        std::cout << "x is: " << X << std::endl;
        return X;
    }
    //求位置部分Px = ((A-I)的逆)*(X*Pb-Pa)
    Eigen::Vector3d getPx(Eigen::Matrix3d& X, Eigen::Matrix4d& Ta, Eigen::Matrix4d& Tb)
    {
        Eigen::Matrix3d C = (Ta.block<3, 3>(0, 0) - Eigen::Matrix3d::Identity()).inverse();
        Eigen::Vector3d D = X*Tb.block<3, 1>(0, 3) - Ta.block<3, 1>(0, 3);
        Eigen::Vector3d Px = C*D;
        std::cout << "Px is: " << Px << std::endl;
        return Px;
    }
    Eigen::Matrix4d getTx(std::array<Eigen::Matrix4d, 3>& Ta, std::array<Eigen::Matrix4d, 3>& Tb)
    {

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
