import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

# 点云可视化函数
def plot_point_cloud(points, title="Point Cloud", color='b'):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制点云
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=color, s=20)
    
    # 设置坐标轴标签
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    
    # 设置标题
    ax.set_title(title)
    
    # 显示图形
    plt.show()

# 示例数据
if __name__ == "__main__":
    # 读取一个.ply文件 (ASCII格式)
    ply_file = "bunny0.ply"  # 请替换为你自己的文件路径
    points = read_ply(ply_file)

    # 显示点云
    plot_point_cloud(points)
