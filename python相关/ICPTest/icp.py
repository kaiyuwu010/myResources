import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 定义一个函数计算最近点
def find_closest_points(source, target):
    closest_points = []
    for point in source:
        distances = np.linalg.norm(target - point, axis=1)
        closest_point = target[np.argmin(distances)]
        closest_points.append(closest_point)
    return np.array(closest_points)

# 计算刚体变换矩阵 (包括旋转和平移)
def compute_transformation(source, closest_points):
    centroid_source = np.mean(source, axis=0)
    centroid_target = np.mean(closest_points, axis=0)
    
    source_centered = source - centroid_source
    target_centered = closest_points - centroid_target

    H = np.dot(source_centered.T, target_centered)
    
    U, _, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)

    t = centroid_target.T - np.dot(R, centroid_source.T)
    
    return R, t

# 将刚体变换应用于点云
def apply_transformation(source, R, t):
    return np.dot(source, R.T) + t.T

# ICP主循环
def icp(source, target, max_iterations=100, tolerance=1e-6, visualize=True):
    prev_error = float('inf')

    for i in range(max_iterations):
        closest_points = find_closest_points(source, target)
        R, t = compute_transformation(source, closest_points)
        source = apply_transformation(source, R, t)
        
        mean_error = np.mean(np.linalg.norm(closest_points - source, axis=1))
        
        if visualize:
            plot_point_clouds(source, target, iteration=i)

        if np.abs(prev_error - mean_error) < tolerance:
            print(f"ICP收敛于第{i}次迭代，误差为{mean_error}")
            break
        
        prev_error = mean_error

    return source, R, t

# 点云可视化函数
def plot_point_clouds(source, target, iteration=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制源点云（蓝色）
    ax.scatter(source[:, 0], source[:, 1], source[:, 2], c='b', s=20, label='Source')
    
    # 绘制目标点云（红色）
    ax.scatter(target[:, 0], target[:, 1], target[:, 2], c='r', s=20, label='Target')

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')

    title = "ICP Point Cloud Alignment"
    if iteration is not None:
        title += f" - Iteration {iteration}"
    ax.set_title(title)

    ax.legend()
    plt.show()

# 读取 .ply 文件 (ASCII格式)
def read_ply(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    # 解析头文件
    header = []
    vertex_count = 0
    for i, line in enumerate(lines):
        header.append(line.strip())
        if line.startswith("element vertex"):
            # 获取点的数量
            vertex_count = int(line.split()[-1])
        elif line.startswith("end_header"):
            # 点数据从这行之后开始
            start = i + 1
            break
    
    # 检查头文件是否符合预期
    if vertex_count == 0:
        raise ValueError("未能正确解析头文件中的顶点数量。")

    # 读取点数据
    points = []
    for line in lines[start:start + vertex_count]:
        # 解析每个点的x, y, z坐标 (假设文件格式为ASCII)
        point = list(map(float, line.strip().split()))[:3]
        points.append(point)
    
    return np.array(points)

# 示例数据
if __name__ == "__main__":
    # 读取源点云和目标点云
    source_ply_file = "bunny0.ply"  # 替换为源点云的路径
    target_ply_file = "bunny45.ply"  # 替换为目标点云的路径
    
    source_points = read_ply(source_ply_file)
    target_points = read_ply(target_ply_file)
    
    # 绘制初始的源点云和目标点云
    print("初始点云：")
    plot_point_clouds(source_points, target_points)
    
    # 运行ICP并可视化每次迭代的点云变化
    aligned_source, R, t = icp(source_points, target_points, max_iterations=20, tolerance=1e-6)

    print("最终的旋转矩阵R：")
    print(R)
    print("最终的平移向量t：")
    print(t)
