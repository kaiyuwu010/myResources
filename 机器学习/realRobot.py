from xarm.wrapper import XArmAPI
from robot_controller import RobotContorller
from control_openRB150_with_modbus_rtu import ParalleGripperOpenRB150
import pyrealsense2 as rs
import cv2

import torch
import pickle
import time
import argparse
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
from policy import ACTPolicy

def main(args):
    # load model
    policy_config = {'lr': 1e-5,
                        'num_queries': 100,
                        'kl_weight': 10,
                        'hidden_dim': 512,
                        'dim_feedforward': 3200,
                        'lr_backbone': 1e-5,
                        'backbone': 'resnet18',
                        'enc_layers': 4,
                        'dec_layers': 7,
                        'nheads': 8,
                        'camera_names': ['top'],
                        'ckpt_dir': "/home/wky/myRosWorkspace/rl/act/data/policy_best.ckpt",
                        'policy_class': "ACT",
                        'task_name': "sim_transfer_cube_scripted",
                        'num_epochs': 2000,
                        'seed': 0,
                        }
    policy = ACTPolicy(policy_config)
    loading_status = policy.load_state_dict(torch.load("/home/wky/myRosWorkspace/rl/act/data/policy_best.ckpt"))
    policy.cuda()
    policy.eval()
    torch.inference_mode()
        
    # model param
    time_stamp = 400000
    num_query = 100
    with open("/home/wky/myRosWorkspace/rl/act/data/dataset_stats.pkl", 'rb') as f:
        stats = pickle.load(f)
    pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean']) / stats['qpos_std']
    post_process = lambda a: a * stats['action_std'] + stats['action_mean']
    
    # real robot and gripper setup
    ip_robot = "192.168.1.206"
    arm = XArmAPI(ip_robot, is_radian=True)
    arm.clean_error()
    DH_params = None
    rc = RobotContorller(arm, DH_params, filter_size=5, filter_type=None)
    pga = ParalleGripperOpenRB150(arm)
    max_delta = 0.02

    # setup camera
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        dev.hardware_reset()
    time.sleep(2)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)            
    flip = False
    
    #开启plt交互模式，允许动态更新图像，而无需关闭窗口。
    plt.ion()
    # control loop
    for t in range(time_stamp):
        # inference
        if t % num_query == 0:
            # get image
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            # rotate 180 degree's because everything is upside down in order to center the camera
            if flip:
                color_image = cv2.rotate(color_image, cv2.ROTATE_180)
            plt.imshow(color_image)
            plt.axis('off')
            plt.show()
            plt.pause(0.05)
            plt.clf()
            # print("+++++++++++++++++++++shape: \n", image.shape)
            tensor_camera_tensor = torch.from_numpy(color_image / 255.0).float().cuda().unsqueeze(0)
            tensor_camera_tensor = torch.einsum('k h w c -> k c h w', tensor_camera_tensor)
            print("+++++++++++++++++++++shape: \n", tensor_camera_tensor.shape)
            tensor_camera_tensor = tensor_camera_tensor.unsqueeze(0)
            print("+++++++++++++++++++++shape: \n", tensor_camera_tensor.shape)
            
            # get joints angels
            code, angles = arm.get_servo_angle(is_radian=True)
            while code != 0:
                print(f"Error code {code} in get_servo_angle().")
                continue
            print("获取的机械臂角度:",[round(angle * 57.29578, 1) for angle in angles])

            fill_data = np.array([0, 0, 0, 0, 0, 0, 0])
            qpos = np.concatenate((angles, fill_data))
            pre_process(qpos)
            qpos_data = torch.from_numpy(qpos).float().cuda().unsqueeze(0) #shape(1, 14)
            qpos_data = qpos_data.to(torch.float32)
            print("+++++++++++++++++++++qpos: \n", qpos_data.shape)
            # inference
            print("预测新的序列！！！")
            all_actions = policy(qpos_data, tensor_camera_tensor)
            
        print("+++++++++++++++++++++all_actions: \n", all_actions.shape)
        action_data = all_actions[:, t % num_query]
        print("+++++++++++++++++++++action_data: \n", action_data.shape)
        action_data = action_data.squeeze(0).cpu().detach().numpy()
        print("+++++++++++++++++++++action_data: \n", action_data)
        post_process(action_data)
        
        # drive real arm
        arm.set_mode(1)
        arm.set_state(0)
        action_angles = np.array([action_data[0], action_data[1], action_data[2], action_data[3], action_data[4], action_data[5]])
        # approach initial position
        for _ in range(30):
            code, current_joints = arm.get_servo_angle(is_radian=True)
            current_joints = np.array(current_joints[:6])
            while code != 0:
                print(f"Error code {code} in get_servo_angle().")
                continue
            print("+++++++++++++++++获取的机械臂角度:",[round(angle * 57.29578, 1) for angle in current_joints])
            print("+++++++++++++++++目标角度:",[round(angle * 57.29578, 1) for angle in action_angles])
            delta = np.array(action_angles - current_joints)
            max_joint_delta = np.abs(delta).max()
            if max_joint_delta > max_delta:
                delta = delta / max_joint_delta * max_delta
            step_result = current_joints + delta
            print("+++++++++++++++++执行角度:",[round(angle * 57.29578, 1) for angle in step_result])
            rc.move_robot_joint(step_result, is_radian=True)
            time.sleep(0.1)
            if max_joint_delta < 0.01:
                break
        print("角度:",[round(angle * 57.29578, 1) for angle in action_angles])
        print("弧度:",[angle for angle in action_angles])
        rc.move_robot_joint(action_angles[:6], is_radian=True)
        # 模型操作手打开时的齿轮弧度是0.34，关闭时的齿轮弧度是-0.6135
        # lite6末端夹爪打开时齿轮角度是230°，关闭时齿轮角度是0°
        # 先把模型操作手的齿轮弧度转化到[0, 0.9535]的范围，然后乘以尺度缩放230÷0.9535=241.2166
        gripper_pos = int((action_data[6] + 0.6135) * 241.2166)
        gripper_pos = max(min(gripper_pos, 300), 0)
        pga.move(gripper_pos)   

    print("Done")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval', action='store_true')
    parser.add_argument('--onscreen_render', action='store_true')
    parser.add_argument('--ckpt_dir', action='store', type=str, help='ckpt_dir', required=True)
    parser.add_argument('--policy_class', action='store', type=str, help='policy_class, capitalize', required=True)
    parser.add_argument('--task_name', action='store', type=str, help='task_name', required=True)
    parser.add_argument('--batch_size', action='store', type=int, help='batch_size', required=True)
    parser.add_argument('--seed', action='store', type=int, help='seed', required=True)
    parser.add_argument('--num_epochs', action='store', type=int, help='num_epochs', required=True)
    parser.add_argument('--lr', action='store', type=float, help='lr', required=True)

    # for ACT
    parser.add_argument('--kl_weight', action='store', type=int, help='KL Weight', required=False)
    parser.add_argument('--chunk_size', action='store', type=int, help='chunk_size', required=False)
    parser.add_argument('--hidden_dim', action='store', type=int, help='hidden_dim', required=False)
    parser.add_argument('--dim_feedforward', action='store', type=int, help='dim_feedforward', required=False)
    parser.add_argument('--temporal_agg', action='store_true')
    main(vars(parser.parse_args()))