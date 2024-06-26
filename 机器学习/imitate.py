from isaacgym import gymapi
from isaacgym import gymutil
from isaacgym import gymtorch
import pyrealsense2 as rs
import time
import torch
import pickle
import argparse
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
from policy import ACTPolicy
from isaacgymenvs.utils.torch_jit_utils import to_torch

def main(args):
    # Initialize gym
    gym = gymapi.acquire_gym()

    # Configure sim
    sim_params = gymapi.SimParams()
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.dt = 1.0 / 60.0
    sim_params.substeps = 2
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 4
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.use_gpu = True
    sim_params.physx.num_threads = 4

    # Create sim
    sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)
    if sim is None:
        print("*** Failed to create sim")
        quit()

    # Create environment
    env = gym.create_env(sim, gymapi.Vec3(-1.0, -1.0, 0.0), gymapi.Vec3(1.0, 1.0, 2.0), 1)

    # Create viewer
    viewer_props = gymapi.CameraProperties()
    viewer_props.horizontal_fov = 75.0
    viewer_props.width = 1920
    viewer_props.height = 1080
    viewer = gym.create_viewer(sim, viewer_props)
    if viewer is None:
        print("*** Failed to create viewer")
        quit()

    # Add ground plane
    plane_params = gymapi.PlaneParams()
    plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
    gym.add_ground(sim, plane_params)

    # Load and add arm asset
    asset_root = "/home/wky/myRosWorkspace/rl/act"
    asset_file = "urdf-lite6-parallelGripper/urdf/LiteParallelGriper13.urdf"
    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = True
    asset_options.collapse_fixed_joints = True
    asset_options.disable_gravity = True
    asset_options.thickness = 0.001
    robot_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
    robot_pose = gymapi.Transform()
    robot_pose.p = gymapi.Vec3(0.0, 0.0, 0.36)  #机器人距离桌面16cm
    robot_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
    robot_handle = gym.create_actor(env, robot_asset, robot_pose, "robot", 0, 0)
        
    # Set up DOF properties
    dof_props = gym.get_actor_dof_properties(env, robot_handle)
    num_arm_dofs = gym.get_asset_dof_count(robot_asset)
    arm_dof_stiffness = [5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000]
    arm_dof_damping = [100, 100, 100, 100, 100, 100, 100, 100]
    for i in range(num_arm_dofs):
        dof_props['driveMode'][i] = gymapi.DOF_MODE_POS
        dof_props['stiffness'][i] = arm_dof_stiffness[i]
        dof_props['damping'][i] = arm_dof_damping[i]
    gym.set_actor_dof_properties(env, robot_handle, dof_props)
    
    # # Create camera attach to hand
    # camera_props = gymapi.CameraProperties()
    # camera_props.width = 640
    # camera_props.height = 480
    # camera_handle = gym.create_camera_sensor(env, camera_props)
    # local_transform = gymapi.Transform()
    # local_transform.p = gymapi.Vec3(0.00, -0.14, 0.77)
    # local_transform.r = gymapi.Quat.from_euler_zyx(np.deg2rad(-90), np.deg2rad(90), 0)
    # # local_transform.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(0, 1, 0), np.deg2rad(90))from_euler_zyx
    # actor_handle = gym.get_actor_handle(env, 0)
    # attachedBody_handle = gym.get_actor_rigid_body_handle(env, actor_handle, 6)
    # gym.attach_camera_to_body(camera_handle, env, attachedBody_handle, local_transform, gymapi.FOLLOW_TRANSFORM)
    
    # Create stable camera 
    camera_props = gymapi.CameraProperties()
    camera_props.width = 640
    camera_props.height = 480
    camera_handle = gym.create_camera_sensor(env, camera_props)
    # camera_transform = gymapi.Transform()
    # camera_transform.p = gymapi.Vec3(0.00, 0.30, 0.32)
    # camera_transform.r = gymapi.Quat.from_euler_zyx(0, np.deg2rad(90), np.deg2rad(90)) #这里的欧拉角是固定系欧拉角
    # gym.set_camera_transform(camera_handle, env, camera_transform)
    gym.set_camera_location(camera_handle, env, gymapi.Vec3(0.00, 0.40, 0.42), gymapi.Vec3(0, -0.3, -1))
    # # Create cap 
    # try:
    #     asset_root = "/home/wky/myRosWorkspace/rl/act"
    #     asset_file = "urdf-lite6-parallelGripper/urdf/cap.urdf"
    #     asset_options = gymapi.AssetOptions()
    #     asset_options.fix_base_link = False
    #     asset_options.collapse_fixed_joints = True
    #     cap_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
    #     cap_pose = gymapi.Transform()
    #     cap_pose.p = gymapi.Vec3(0.1, 0.1, 0.01)  
    #     cap_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
    #     cap_handle = gym.create_actor(env, cap_asset, cap_pose, "cap", 0, 1)
    #     cap_color = gymapi.Vec3(0.0, 0.5, 0.0)
    #     gym.set_rigid_body_color(env, cap_handle, 0, gymapi.MESH_VISUAL, cap_color)
    # except:
    #     print("*** Failed to create cap asset")
    #     quit()
    
    # Create table 
    try:
        table_opts = gymapi.AssetOptions()
        table_opts.fix_base_link = True
        table_opts.thickness = 0.005
        table_asset = gym.create_box(sim, 0.46, 0.40, 0.40, table_opts)
        # Add the bottle cap to the environment
        table_pose = gymapi.Transform()
        table_pose.p = gymapi.Vec3(0.0, 0.13, 0.0)  # Place the cap slightly above the ground
        table_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
        table_handle = gym.create_actor(env, table_asset, table_pose, "box2", 0, 1)
        table_color = gymapi.Vec3(0.4, 0.4, 0.4)
        gym.set_rigid_body_color(env, table_handle, 0, gymapi.MESH_VISUAL, table_color)
    except:
        print("*** Failed to create box2 asset")
        quit()  
    # Create box 1 
    try:
        box1_opts = gymapi.AssetOptions()
        box1_opts.fix_base_link = False
        box1_opts.thickness = 0.005
        box1_asset = gym.create_box(sim, 0.0305, 0.021, 0.021, box1_opts)
        # Add the bottle cap to the environment
        box1_pose = gymapi.Transform()
        box1_pose.p = gymapi.Vec3(0.06, 0.22, 0.65)  # Place the cap slightly above the ground
        box1_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
        box1_handle = gym.create_actor(env, box1_asset, box1_pose, "box2", 0, 2)
        box1_color = gymapi.Vec3(0.00, 0.00, 0.00)
        gym.set_rigid_body_color(env, box1_handle, 0, gymapi.MESH_VISUAL, box1_color)
    except:
        print("*** Failed to create box2 asset")
        quit()
    # Create box 2 
    try:
        box2_opts = gymapi.AssetOptions()
        box2_opts.fix_base_link = False
        box2_opts.thickness = 0.005
        box2_asset = gym.create_box(sim, 0.037, 0.027, 0.02, box2_opts)
        # Add the bottle cap to the environment
        box2_pose = gymapi.Transform()
        box2_pose.p = gymapi.Vec3(-0.05, 0.20, 0.65)  # Place the cap slightly above the ground
        box2_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
        box2_handle = gym.create_actor(env, box2_asset, box2_pose, "box2", 0, 4)
        box2_color = gymapi.Vec3(0.35, 0.0, 0.0)
        gym.set_rigid_body_color(env, box2_handle, 0, gymapi.MESH_VISUAL, box2_color)
    except:
        print("*** Failed to create box2 asset")
        quit()

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
        
    # Simulate
    time_stamp = 10000
    num_query = 100
    dof_index = gym.get_actor_dof_index(env, robot_handle, 0, gymapi.DOMAIN_SIM)
    dof0_handle = gym.find_actor_dof_handle(env, robot_handle, "Joint1")
    dof1_handle = gym.find_actor_dof_handle(env, robot_handle, "Joint2")
    dof2_handle = gym.find_actor_dof_handle(env, robot_handle, "Joint3")
    dof3_handle = gym.find_actor_dof_handle(env, robot_handle, "Joint4")
    dof4_handle = gym.find_actor_dof_handle(env, robot_handle, "Joint5")
    dof5_handle = gym.find_actor_dof_handle(env, robot_handle, "Joint6")
    dof_f1_handle = gym.find_actor_dof_handle(env, robot_handle, "Joint_f1")
    dof_f2_handle = gym.find_actor_dof_handle(env, robot_handle, "Joint_f2")
    with open("/home/wky/myRosWorkspace/rl/act/data/dataset_stats.pkl", 'rb') as f:
        stats = pickle.load(f)
    pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean']) / stats['qpos_std']
    post_process = lambda a: a * stats['action_std'] + stats['action_mean']

    # 确定viewer相机的位姿
    cam_pos = gymapi.Vec3(0, 1, 1)
    cam_target = gymapi.Vec3(0, -1, -1)#相机z轴对准的方向
    gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

    # # setup camera
    # ctx = rs.context()
    # devices = ctx.query_devices()
    # for dev in devices:
    #     dev.hardware_reset()
    # time.sleep(2)
    # pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # pipeline.start(config)
    #开启plt交互模式，允许动态更新图像，而无需关闭窗口。
    plt.ion()
    for t in range(time_stamp):
        if gym.query_viewer_has_closed(viewer):
            print("viewer_has_closed!!!")
            exit()
        # Step the physics
        gym.simulate(sim)
        gym.fetch_results(sim, True)

        # Update the viewer
        gym.step_graphics(sim)
        gym.draw_viewer(viewer, sim, True)
        
        # inference
        if t % num_query == 0:
            # get and show image(imitate enviroment)
            gym.render_all_camera_sensors(sim)
            gym.start_access_image_tensors(sim)
            camera_tensor = gym.get_camera_image(sim, env, camera_handle, gymapi.IMAGE_COLOR)
            print("+++++++++++++++++++++shape: \n", camera_tensor.shape)
            camera_tensor = camera_tensor.reshape(480, 640, 4)
            # image = Image.fromarray(camera_tensor, 'RGBA')
            camera_tensor = camera_tensor[:, :, :3]
            image = Image.fromarray(camera_tensor, 'RGB')
            # # get and show image(real enviroment)
            # frames = pipeline.wait_for_frames()
            # color_frame = frames.get_color_frame()
            # color_image = np.asanyarray(color_frame.get_data())
            # color_image = color_image[:, :, ::-1]
            plt.imshow(image)
            plt.axis('off')
            plt.show()
            plt.pause(0.05)
            plt.clf()
            print("+++++++++++++++++++++shape: \n", camera_tensor.shape)
            tensor_camera_tensor = torch.from_numpy(camera_tensor / 255.0).float().cuda().unsqueeze(0)
            tensor_camera_tensor = torch.einsum('k h w c -> k c h w', tensor_camera_tensor)
            print("+++++++++++++++++++++shape: \n", tensor_camera_tensor.shape)
            tensor_camera_tensor = tensor_camera_tensor.unsqueeze(0)
            print("+++++++++++++++++++++shape: \n", tensor_camera_tensor.shape)
            gym.end_access_image_tensors(sim)
            
            # get joints angels
            gym.refresh_dof_state_tensor(sim)
            _dof_states = gym.acquire_dof_state_tensor(sim)
            dof_states = gymtorch.wrap_tensor(_dof_states)
            print("关节0角度°:", float(dof_states[dof_index, 0])*57.29578,
                "\n关节1角度°:", float(dof_states[dof_index+1, 0])*57.29578,
                "\n关节2角度°:", float(dof_states[dof_index+2, 0])*57.29578,
                "\n关节3角度°:", float(dof_states[dof_index+3, 0])*57.29578,
                "\n关节4角度°:", float(dof_states[dof_index+4, 0])*57.29578,
                "\n关节5角度°:", float(dof_states[dof_index+5, 0])*57.29578)
            joints = dof_states.numpy()[0:7, 0]
            fill_data = np.array([0, 0, 0, 0, 0, 0, 0])
            qpos = np.concatenate((joints, fill_data))
            pre_process(qpos)
            qpos_data = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
            qpos_data = qpos_data.to(torch.float32)
            print("+++++++++++++++++++++qpos: \n", qpos_data.shape)
            print("预测新的序列！！！")
            all_actions = policy(qpos_data, tensor_camera_tensor)
        print("+++++++++++++++++++++all_actions: \n", all_actions.shape)
        action_data = all_actions[:, t % num_query]
        print("+++++++++++++++++++++action_data: \n", action_data.shape)
        action_data = action_data.squeeze(0).cpu().detach().numpy()
        print("+++++++++++++++++++++action_data: \n", action_data)
        post_process(action_data)
        # control sim arm
        gym.set_dof_target_position(env, dof0_handle, action_data[0])
        gym.set_dof_target_position(env, dof1_handle, action_data[1])
        gym.set_dof_target_position(env, dof2_handle, -action_data[2])
        gym.set_dof_target_position(env, dof3_handle, -action_data[3])
        gym.set_dof_target_position(env, dof4_handle, action_data[4])
        gym.set_dof_target_position(env, dof5_handle, -action_data[5])
        gym.set_dof_target_position(env, dof_f1_handle, action_data[6]*0.033) #gello里面采集的数据会把夹爪的张合距离转化到(0,1)之间，张开最大为1最小为0
        gym.set_dof_target_position(env, dof_f2_handle, action_data[6]*0.033) #这里的0.033m表示仿真环境中单个手指移动的最大距离
        print("action_data[0]°:", float(action_data[0])*57.29578,
             "\naction_data[1]°:", float(action_data[1])*57.29578,
             "\naction_data[2]°:", float(action_data[2])*57.29578,
             "\naction_data[3]°:", float(action_data[3])*57.29578,
             "\naction_data[4]°:", float(action_data[4])*57.29578,
             "\naction_data[5]°:", float(action_data[5])*57.29578,
             "\naction_data[6]°:", float(action_data[6])*57.29578,
             "\naction_data[7]°:", float(action_data[7])*57.29578,
             "\naction_data[8]°:", float(action_data[8])*57.29578,
             "\naction_data[9]°:", float(action_data[9])*57.29578,
             "\naction_data[10]°:", float(action_data[10])*57.29578,
             "\naction_data[11]°:", float(action_data[11])*57.29578,
             "\naction_data[12]°:", float(action_data[12])*57.29578,
             "\naction_data[13]°:", float(action_data[13])*57.29578)        

    print("Done")
    # Cleanup
    gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)

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