# 下面是一个如何注册机械臂事件回调函数的示例：
# 在这个示例中，我们定义了一个名为`event_callback`的函数，用于处理机械臂的事件，并将其注册为回调函数。
# 当机械臂事件发生时，`event_callback`函数将被调用，并接收一个包含事件数据的对象作为参数
from Robotic_Arm.rm_robot_interface import *
import time

def event_func(data:rm_event_push_data_t) -> None:
    print("The motion is complete, the arm is in place.")
    # 判断接口类型
    if data.event_type == 1:  # 轨迹规划完成
        print("运动结果:", data.trajectory_state)
        print("当前设备:", data.device)
        print("是否连接下一条轨迹:", data.trajectory_connect)
    elif data.codeKey == 2:  # 在线编程文件运行完成
        print("在线编程文件结束id:", data.program_id)

# 初始化为三线程模式
arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)

# 创建机械臂连接，打印连接id
handle = arm.rm_create_robot_arm("192.168.1.18", 8080)
print(handle.id)

event_callback = rm_event_callback_ptr(event_func)
arm.rm_get_arm_event_call_back(event_callback)

# 非阻塞关节运动
ret = arm.rm_movej([0, 30, 60, 0, 90, 0], 20, 0, 0, 0)
print("movej: ", ret)

# 等待打印数据
time.sleep(10)

# 删除指定机械臂对象
arm.rm_delete_robot_arm()