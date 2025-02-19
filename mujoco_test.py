import mujoco
import numpy as np
from mujoco import viewer
import time
import os


from controller import RobotKinematics


def main():
    # 加载模型
    model_path = "urdf/rm_65.xml"  # MJCF文件路径
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    viewer = mujoco.viewer.launch_passive(model, data)

    rcontroller = RobotKinematics(model_path, "Link6")

    initial_joint  = np.array([90, -60, 120, 00, -60, 0])

    initial_joint = np.deg2rad(initial_joint)

    initial_pos, initial_rot = rcontroller.forward_kinematics(initial_joint)

    print(initial_pos, initial_rot)

    target_pos = np.array(initial_pos)

    target_pos[2] = target_pos[2] + 0.05

    inv_joint ,_= rcontroller.solve_optimization_ik(initial_joint, target_pos, initial_rot)

    print(inv_joint)

    z = initial_pos[2] + 0.05
    

    start_time = time.time()

    # while True:

    #     data.qpos[:] = initial_joint
    #     data.qvel[:] = 0



    for i in range(181):
        target_pos = initial_pos.copy()
        target_pos[2] += + np.sin(i * np.pi / 30) * 0.03

        #z = initial_pos[2] - 0.1
        #z = initial_pos[2] +0.01 # + np.sin(i * np.pi / 30) * 0.03

        #print(target_pos)

        
        time1 = time.time()

        inv_joint , flag = rcontroller.solve_optimization_ik(initial_joint, target_pos,initial_rot)

        print(time.time() - time1,flag)

        #print(inv_joint)

        data.qpos[:] = inv_joint

        mujoco.mj_forward(model, data)
        viewer.sync()



        time.sleep(0.02)
        #print(time.time() - start_time)

        initial_joint = inv_joint
    
    while True:
        data.qpos[:] = initial_joint
        data.qvel[:] = 0




            

if __name__ == "__main__":
    main()