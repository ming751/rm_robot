import mujoco
import numpy as np
from mujoco import viewer
import time
import os
import pinocchio as pin
import math
from datetime import datetime
from controller import RobotKinematics

def main():
    # 加载模型
    model_path = "urdf/rm_65.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    viewer = mujoco.viewer.launch_passive(model, data)
    rcontroller = RobotKinematics(model_path, "Link6")

    zeros_pos, zeros_rot = rcontroller.forward_kinematics(np.zeros(6))
    desired_rot = pin.utils.rotate('y', np.pi/2) @ zeros_rot

    inv_joint = np.zeros(6)
    
    # 获取site的ID
    trace_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "trace")
    print(trace_id)
    
    # 创建轨迹点存储列表
    trajectory_points = []
    max_points = 1000  # 最大存储点数

    while True:
        dt = datetime.now()
        t = (dt.second / 60.) * 2. * math.pi

        # 计算目标位置
        current_pos = [0.2, 0.2 * math.sin(t), 0.2 * math.cos(t) + 0.4]
        
        # 求解逆运动学
        inv_joint, flag = rcontroller.solve_inverse_kinematics(inv_joint, current_pos, desired_rot)
        
        if True:  # 如果逆运动学求解成功
            # 更新机器人位置
            data.qpos[:] = inv_joint
            
            # 更新site位置到当前末端位置
            data.site_xpos[trace_id] = current_pos
            #print(data.site_xpos[trace_id])
            
            # 存储轨迹点（如果需要的话）
            trajectory_points.append(np.array(current_pos))
            if len(trajectory_points) > max_points:
                trajectory_points.pop(0)
            
        mujoco.mj_step(model, data)
        viewer.sync()
        
        time.sleep(0.01)

if __name__ == "__main__":
    main()