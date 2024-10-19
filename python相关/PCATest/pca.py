import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

def load_image(image_path):
    image = Image.open(image_path).convert('L')  # 将图像转换为灰度图
    image_data = np.asarray(image, dtype=np.float32)
    print("图片维度： ", image_data.shape)
    return image_data
    
# 计算协方差矩阵
def cov(data):
    data = np.asarray(data)
    if data.ndim == 1:
        data = data.reshape(-1, 1)  # 如果是 1D 数组，转换为 2D 数组
    # 计算每列的均值
    mean = np.mean(data, axis=0)
    print("计算协方差矩阵，每列的均值维度： ", mean.shape)
    # 均值归一化，NumPy中，不同维度的矩阵可以通过广播机制进行运算
    data_centered = data - mean
    # 计算协方差矩阵
    n_samples = data.shape[0]
    cov_matrix = np.dot(data_centered.T, data_centered) / (n_samples - 1)
    print("协方差矩阵维度： ", cov_matrix.shape)
    return cov_matrix

def mean_normalization(data):
    mean = np.mean(data, axis=0)  # 计算每列的均值
    data_normalized = data - mean
    print("normalization后矩阵的维度： ", data_normalized.shape)
    return data_normalized, mean

# 使用自定义协方差矩阵计算函数
def compute_covariance_matrix(data_normalized):
    covariance_matrix = cov(data_normalized)
    return covariance_matrix

def compute_eigenvalues_and_eigenvectors(covariance_matrix):
    eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)  # 计算特征值和特征向量
    print("协方差特征值维度： ", eigenvalues.shape)
    print("协方差特征向量维度： ", eigenvectors.shape)
    return eigenvalues, eigenvectors

def reduce_dimensions(data_normalized, eigenvectors, num_components):
    selected_vectors = eigenvectors[:, :num_components]  # 选择前num_components个特征向量
    reduced_data = np.dot(data_normalized, selected_vectors)
    print("选择的特征向量维度： ", selected_vectors.shape)
    print("降维后的数据维度： ", reduced_data.shape)
    return reduced_data, selected_vectors

def reconstruct_image(reduced_data, eigenvectors, mean):
    reconstructed_data = np.dot(reduced_data, eigenvectors.T) + mean
    return reconstructed_data

# 主函数：执行PCA
def pca_image_compression(image_path, num_components=50):
    # 1. 加载并转换图像
    image_data = load_image(image_path)
    
    # 2. 将二维图像展平为一维数组
    original_shape = image_data.shape
    data_flattened = image_data.reshape(-1, original_shape[1])  # 每行代表一个像素行
    
    # 3. 均值归一化
    data_normalized, mean = mean_normalization(data_flattened)
    
    # 4. 计算协方差矩阵
    covariance_matrix = compute_covariance_matrix(data_normalized)
    
    # 5. 计算特征值和特征向量
    eigenvalues, eigenvectors = compute_eigenvalues_and_eigenvectors(covariance_matrix)
    
    # 6. 根据特征值的大小排序特征向量
    idx = np.argsort(eigenvalues)[::-1]  # 从大到小排序
    eigenvectors = eigenvectors[:, idx]  # 排序后的特征向量
    eigenvalues = eigenvalues[idx]  # 排序后的特征值
    
    # 7. 降维处理
    reduced_data, selected_vectors = reduce_dimensions(data_normalized, eigenvectors, num_components)
    
    # 8. 重构图像
    reconstructed_data = reconstruct_image(reduced_data, selected_vectors, mean)
    reconstructed_image = reconstructed_data.reshape(original_shape)  # 还原图像
    
    return image_data, reconstructed_image

# 显示原始图像和PCA压缩后的图像
def display_images(original_image, reconstructed_image):
    plt.subplot(1, 2, 1)
    plt.title("Original Image")
    plt.imshow(original_image, cmap='gray')

    plt.subplot(1, 2, 2)
    plt.title("Reconstructed Image")
    plt.imshow(reconstructed_image, cmap='gray')

    plt.show()

# 测试PCA算法
image_path = 'wky.jpeg'  # 替换为你的图像路径
original_image, reconstructed_image = pca_image_compression(image_path, num_components=200)
display_images(original_image, reconstructed_image)

