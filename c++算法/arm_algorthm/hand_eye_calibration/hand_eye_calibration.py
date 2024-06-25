import cv2
import numpy as np

# 相机内参(replace with your actual camera parameters)
camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]])
dist_coeffs = np.array([0, 0, 0, 0, 0])

# Marker尺寸（米）
marker_length = 0.05

# 根据图片进行位姿估计
def estimatePose(image, aruco_dict_type=cv2.aruco.DICT_6X6_250):
    # Load the dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Detect markers
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # 3D位姿估计
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
    return corners, ids, rvecs, tvecs

# 在图片上画出检测出的坐标
def draw_aruco_markers(frame, corners, ids, rvecs, tvecs, camera_matrix, dist_coeffs):
    for i in range(len(ids)):
        cv2.aruco.drawDetectedMarkers(frame, corners)
        cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length * 0.5)
        center = (int(corners[i][0][:, 0].mean()), int(corners[i][0][:, 1].mean()))
        cv2.circle(frame, center, 5, (0, 255, 0), -1)
        cv2.putText(frame, f"ID: {ids[i]}", (center[0] + 10, center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.putText(frame, f"tvec: {tvecs[i].ravel()}", (center[0] + 10, center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
# 输入图片和末端夹爪位姿，计算相机相对末端夹爪的位姿
def calculateResult(images, poses):
    for image, pose in zip(images, poses):
        rvecs, tvecs = estimatePose(image)
        gripper_rvecs, _ = cv2.Rodrigues(pose)
        # The rotation vector's direction is the axis of rotation
        axis = rotation_vector.flatten()
        # The norm of the rotation vector is the rotation angle in radians
        angle = np.linalg.norm(axis)
        return pose
    
def main():
    image = cv2.imread('/home/wky/Downloads/1.jpeg')
    corners, ids, rvecs, tvecs = estimatePose(image)
    draw_aruco_markers(image, corners, ids, rvecs, tvecs, camera_matrix, dist_coeffs)
if __name__ == "__main__":
    main()
