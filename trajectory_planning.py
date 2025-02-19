import pinocchio as pin
import matplotlib.pyplot as plt
import numpy as np
from pinocchio import casadi as cpin
import casadi
# import toppra_fun
# import eyou_pose_fun
# import eyou_utils
from scipy.spatial.transform import Rotation, Slerp
from mpl_toolkits.mplot3d import Axes3D

def plot_interpolated_poses(interpolated_poses, target_poses=None):
    """
    Plot the interpolated poses from the interpolate_poses function.

    Args:
        interpolated_poses (list): Output of interpolate_poses function.
        target_poses (list, optional): Original target poses, for comparison. Defaults to None.
    """
    # 提取插值姿势的位置和旋转
    interp_positions = np.array([pos for pos, _ in interpolated_poses])
    interp_rotations = [rot for _, rot in interpolated_poses]

    # 创建 3D 图形
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制插值位置的轨迹 (线)
    ax.plot(interp_positions[:, 0], interp_positions[:, 1], interp_positions[:, 2], label='Interpolated Path', marker='o', markersize=2)

    # 可视化插值姿势的旋转 (坐标轴)
    axis_length = 0.1  # 坐标轴的长度
    for i, (pos, rot) in enumerate(interpolated_poses):
        # 定义局部坐标轴方向 (x, y, z)
        x_axis = rot[:, 0] * axis_length
        y_axis = rot[:, 1] * axis_length
        z_axis = rot[:, 2] * axis_length

        # 绘制 x, y, z 轴
        ax.quiver(pos[0], pos[1], pos[2], x_axis[0], x_axis[1], x_axis[2], color='r', length=0.2, arrow_length_ratio=0.1, alpha=0.5) # x 轴红色
        ax.quiver(pos[0], pos[1], pos[2], y_axis[0], y_axis[1], y_axis[2], color='g', length=0.2, arrow_length_ratio=0.1, alpha=0.5) # y 轴绿色
        ax.quiver(pos[0], pos[1], pos[2], z_axis[0], z_axis[1], z_axis[2], color='b', length=0.2, arrow_length_ratio=0.1, alpha=0.5) # z 轴蓝色X

    # 绘制原始目标姿势 (如果提供)
    if target_poses:
        target_positions = np.array([pos for pos, _ in target_poses])
        ax.scatter(target_positions[:, 0], target_positions[:, 1], target_positions[:, 2], color='red', marker='^', s=100, label='Target Poses')

    # 设置图表属性
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Interpolated Poses and Rotations')
    ax.legend()
    ax.grid(True)
    plt.show(block=True)

def interpolate_poses_bspline(target_poses, T):
    """
    Interpolate between target poses to get T intermediate poses using B-spline interpolation
    for positions and SLERP for rotations.
    
    Args:
        target_poses (list): List of (position, rotation) tuples
        T (int): Number of desired interpolation points
        
    Returns:
        list: Interpolated poses as (position, rotation) tuples
    """
    from scipy.interpolate import BSpline, make_interp_spline
    
    positions = np.array([pos for pos, _ in target_poses])
    rotations = np.array([rot for _, rot in target_poses])
    
    # Create time arrays
    original_times = np.linspace(0, 1, len(target_poses))
    interp_times = np.linspace(0, 1, T + 1)
    

    if len(target_poses) == 2:
        k = 1
    elif len(target_poses) == 3:
        k = 2
    else:
        k = 3

    # Interpolate positions using B-spline
    interpolated_positions = []
    for i in range(3):  # x, y, z
        pos_component = positions[:, i]
        # Create B-spline interpolator
        # k=3 for cubic B-spline
        bspline = make_interp_spline(original_times, pos_component, k=k)
        interp_pos = bspline(interp_times)
        interpolated_positions.append(interp_pos)
    interpolated_positions = np.array(interpolated_positions).T
    
    # Convert rotation matrices to quaternions for interpolation
    quats = []
    for rot in rotations:
        r = Rotation.from_matrix(rot)
        quats.append(r.as_quat())
    quats = np.array(quats)
    
    # Create rotation interpolator
    slerp = Slerp(original_times, Rotation.from_quat(quats))
    
    # Interpolate rotations
    interpolated_rots = slerp(interp_times).as_matrix()
    
    # Combine interpolated positions and rotations
    interpolated_poses = list(zip([pos for pos in interpolated_positions], 
                                [rot for rot in interpolated_rots]))
    return interpolated_poses

def main():
    # 创建示例目标点
    # 每个目标点包含位置和旋转矩阵
    target_poses = [
        # 位置1：原点，旋转矩阵：单位矩阵
        (np.array([0, 0, 0]), np.eye(3)),
        # 位置2：x方向移动，绕z轴旋转45度
        (np.array([1, 0, 0]), Rotation.from_euler('z', 45, degrees=True).as_matrix()),
        # 位置3：xy平面移动，绕x轴旋转30度
        (np.array([1, 1, 0]), Rotation.from_euler('x', 60, degrees=True).as_matrix())
    ]

    # 设置插值点数量
    T = 50

    # 使用B样条插值生成轨迹
    interpolated_poses = interpolate_poses_bspline(target_poses, T)
    print(interpolated_poses)

    # 可视化结果
    plot_interpolated_poses(interpolated_poses, target_poses)

if __name__ == "__main__":
    main()

