import pinocchio as pin
import numpy as np
from pinocchio import casadi as cpin
import casadi
import os

import pybullet as p
import pybullet_data

from typing import Optional, Tuple


os.environ['MESA_GL_VERSION_OVERRIDE'] = '3.3'  # 兼容OpenGL

"""
文件描述: 一个包含机械臂运动学计算的类，包括正运动学计算，逆运动学计算(pybullet求逆以及单步求逆),对轨迹优化求逆等等
四元数格式: scipy pybullet pinocchio 均使用相同的四元数格式 (x,y,z,w)
创建日期: 2025-02-19
版本: 1.0
更新记录:
    2025-02-19: 初始版本
"""
class RobotKinematics:
    def __init__(self, model_path, end_frame_name):
        """
        Initialize the robot kinematics class.
        
        Args:
            model_path (str): Path to the URDF model file
            end_frame_name (str): Name of the end effector frame
        """
        # Load URDF model
        #self.model = pin.buildModelFromMJCF(model_path)
        self.model = pin.buildModelFromUrdf(model_path)
        self.data = self.model.createData()
        
        # Store end effector frame info
        self.end_frame_name = end_frame_name
        self.endEffector_ID = self.model.getFrameId(end_frame_name)
        
        # Initialize Casadi model
        self.cmodel = cpin.Model(self.model)
        self.cdata = self.cmodel.createData()
        
        # Define symbolic variables
        self.cq = casadi.SX.sym("x", self.model.nq, 1)  # joint angles
        self.ceef_pos = casadi.SX.sym("eef_pos", 3, 1)  # target position
        self.ceef_rot = casadi.SX.sym("eef_rot", 3, 3)  # target rotation matrix
        
        # Update kinematics
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        
        # Create error functions
        self._create_error_functions()

        self.joint_limits = {

            'lower': np.array([-3.099,-2.2,-2.355,-3.1,-2.233,-6.28]),
            'upper': np.array([3.099, 2.2, 2.355, 3.1, 2.233, 6.28]),

        }
        
        # 添加优化问题相关属性
        self.opti_ik = None          # 存储优化问题实例
        self.opti_variables = {}     # 存储优化变量
        self.opti_parameters = {}    # 存储优化参数
        self._setup_ik_optimizer()  # 初始化优化问题

        self.client = None

    def get_eef_pos(self, q:np.ndarray):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.endEffector_ID].translation
    
    def forward_kinematics(self,q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.endEffector_ID].translation,self.data.oMf[self.endEffector_ID].rotation
    
    def _create_error_functions(self):
        """Create the 3D and 6D error functions using Casadi"""
        # 3D position error function
        self.error3_tool = casadi.Function(
            "etool3",
            [self.cq, self.ceef_pos],
            [self.cdata.oMf[self.endEffector_ID].translation - self.ceef_pos]
        )
        
        # 6D pose error function
        self.error6_tool = casadi.Function(
            "etool6",
            [self.cq, self.ceef_pos, self.ceef_rot],
            [cpin.log6(self.cdata.oMf[self.endEffector_ID].inverse() * 
             cpin.SE3(self.ceef_rot, self.ceef_pos)).vector]
        )

    def compute_position_error(self, q, target_pos):
        """返回3D位置误差"""
        return np.asarray(self.error3_tool(q, target_pos)).flatten()[:3]  # 确保返回前3个元素

    def compute_pose_error(self, q, target_pos, target_rot):
        """返回6D位姿误差"""
        return np.asarray(self.error6_tool(q, target_pos, target_rot)).flatten()

    def solve_trajectory_optimization(self, initial_q, target_poses, T=10,
                                      w_run_vel=0.01,w_run_pose=1,w_term=100.0):
        """
        Solve trajectory optimization problem.
        
        Args:
            initial_q (np.ndarray): Initial joint configuration
            target_pose (List): Target pose composed with pos and rotation matrix
            T (int): Number of timesteps
            w_run_vel (float): Running cost weight of velocity
            w_run_term (float): Running cost weight of pose error
            w_term(float):Terminal cost of pose error            
        Returns:
            list , float: Optimized joint trajectories, terminal error
        """
        opti = casadi.Opti()
        var_qs = [opti.variable(self.model.nq) for t in range(T + 1)]
        
        # Define cost function
        totalcost = 0
        for t in range(T):
            target_pos ,target_rot = target_poses[t]
            totalcost += w_run_vel * casadi.sumsqr(var_qs[t] - var_qs[t + 1])
            totalcost += w_run_pose *casadi.sumsqr(self.error6_tool(var_qs[T], target_pos, target_rot))
        target_pos , target_rot = target_poses[t]
        totalcost += w_term * casadi.sumsqr(self.error6_tool(var_qs[T], target_pos, target_rot))
        
        # Add constraints
        opti.subject_to(var_qs[0] == initial_q)
        for t in range(T + 1):
            for j in range(self.model.nq):           
                #Apply joint position limits with margin
                opti.subject_to(var_qs[t][j] <= self.joint_limits['upper'][j] )
                opti.subject_to(var_qs[t][j] >= self.joint_limits['lower'][j] ) 
        
        # Set up and solve optimization
        opti.minimize(totalcost)
        opti.solver('ipopt', {
            'print_time': False,
            'ipopt': {
                'print_level': 0,  # This goes inside the 'ipopt' dictionary
                'sb': 'yes'        # Suppress IPOPT banner
            }
        })
        
        try:
            sol = opti.solve_limited()
            sol_qs = [opti.value(var_q) for var_q in var_qs]
        except:
            print("ERROR in convergence, using debug values.")
            sol_qs = [opti.debug.value(var_q) for var_q in var_qs]

        # 计算终端误差
        term_error = self.compute_pose_error(sol_qs[T], target_pos, target_rot)
            
        return sol_qs , term_error
    
    def solve_inverse_kinematics(self, current_q, target_pos, target_rot=None, 
                                urdf_path = "urdf/rm_65.urdf",
                                base_position = [0,0,0],
                                max_iter=1000, tol=1e-5):
        """
        Solve the inverse kinematics problem to find joint angles for a target pose.
        
        Args:
            current_q (np.ndarray): Current joint configuration as starting point
            target_pos (np.ndarray): Target position (3D vector)
            target_rot (np.ndarray, optional): Target rotation matrix (3x3). 
                                                If None, only position is considered.
            max_iter (int): Maximum number of iterations
            tol (float): Tolerance for convergence
            
        Returns:
            np.ndarray: Solved joint configuration
            bool: Whether the solution converged
        """

        if self.client is None:
            self.client = p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 设置模型路径

            # 加载机械臂模型
            self.robot_id = p.loadURDF(urdf_path, base_position, useFixedBase=True)
            self.num_joints = p.getNumJoints(self.robot_id)

            self.movable_joints = [i for i in range(self.num_joints) if p.getJointInfo(self.robot_id, i)[2] != p.JOINT_FIXED]

            self.eef_id_p = self.num_joints - 1


            self.joint_lower_bounds = self.joint_limits['lower']
            self.joint_upper_bounds = self.joint_limits['upper']

            self.joint_ranges = self.joint_upper_bounds - self.joint_lower_bounds

        jd = [0.1] * self.model.nq

        if target_rot is not None:
            # 旋转矩阵转pybullet四元数
            target_rot = pin.Quaternion(target_rot)
            #target_rot = self.pin_to_pybullet_quat(target_rot)

            

            joint_angles = p.calculateInverseKinematics(
                            self.robot_id,
                            self.eef_id_p,
                            target_pos,
                            target_rot,
                            lowerLimits=self.joint_lower_bounds.tolist(),
                            upperLimits=self.joint_upper_bounds.tolist(),
                            jointRanges=self.joint_ranges.tolist(),
                            jointDamping=jd,
                            maxNumIterations=1000,
                            # restPoses=initial_guess,
                            residualThreshold=0.0001  # 适当放宽收敛阈值
                        )
        else:
            joint_angles = p.calculateInverseKinematics(
                            self.robot_id,
                            self.eef_id_p,
                            target_pos,
                            lowerLimits=self.joint_lower_bounds.tolist(),
                            upperLimits=self.joint_upper_bounds.tolist(),
                            jointRanges=self.joint_ranges.tolist(),
                            jointDamping=jd,
                            maxNumIterations=100,
                            residualThreshold=0.001  # 适当放宽收敛阈值
                        )
            
        joint_angles = np.array(joint_angles)
        
        fk_pos , fk_rot = self.forward_kinematics(joint_angles)
        # 计算fk与target_pos的误差
        error = np.linalg.norm(fk_pos - target_pos)
        fk_rot = pin.Quaternion(fk_rot)
        # 四元数误差
        #ori_error_quat = pin.Quaternion(target_rot).inverse() * fk_rot
       # ori_error = pin.log6(ori_error_quat).vector
        # print(f"fk: {fk_pos}, target_pos: {target_pos}, error: {error}")
        # print(f"ori_error: {ori_error_quat}")

        return joint_angles , error

    def _setup_ik_optimizer(self):
        """一次性定义优化问题结构"""
        if self.opti_ik is None:
            self.opti_ik = casadi.Opti()
            
            # 定义优化变量
            self.opti_variables['q'] = self.opti_ik.variable(self.model.nq)
            
            # 定义优化参数（可动态更新）
            self.opti_parameters.update({
                'target_pos': self.opti_ik.parameter(3),
                'target_rot': self.opti_ik.parameter(3, 3),
                'q_ref': self.opti_ik.parameter(self.model.nq),
                'reg_weight': self.opti_ik.parameter(),
                'use_orientation': self.opti_ik.parameter()
            })
            
            # 修改误差维度统一部分
            error_6d = self.error6_tool(self.opti_variables['q'], 
                                      self.opti_parameters['target_pos'],
                                      self.opti_parameters['target_rot'])
            error_3d = casadi.vertcat(
                self.error3_tool(self.opti_variables['q'],
                               self.opti_parameters['target_pos']),
                casadi.MX.zeros(3)  # 补充3个零元素使维度对齐
            )
            
            error = casadi.if_else(self.opti_parameters['use_orientation'], error_6d, error_3d)
            
            # 构建目标函数
            cost = casadi.sumsqr(error)
            cost += self.opti_parameters['reg_weight'] * casadi.sumsqr(
                self.opti_variables['q'] - self.opti_parameters['q_ref'])
            
            # 添加关节约束
            for j in range(self.model.nq):
                self.opti_ik.subject_to(
                    self.opti_variables['q'][j] >= self.joint_limits['lower'][j])
                self.opti_ik.subject_to(
                    self.opti_variables['q'][j] <= self.joint_limits['upper'][j])
            
            # 配置求解器选项
            self.opti_ik.minimize(cost)
            self.opti_ik.solver('ipopt', {
                'print_time': False,
                'ipopt': {
                    'max_iter': 100,
                    'print_level': 0,
                    'acceptable_tol': 1e-4,
                    'linear_solver': 'mumps',
                    'sb': 'yes'
                },
            })

    def solve_optimization_ik(self, q_init, target_pos, target_rot=None,
                             max_iter=100, tol=1e-3, reg_weight=0.0001):
        """求解预定义的优化问题"""
        # 设置参数值
        if q_init is None:
            q_init = np.zeros(self.model.nq)
        self.opti_ik.set_value(self.opti_parameters['target_pos'], target_pos)
        self.opti_ik.set_value(self.opti_parameters['q_ref'], q_init)
        self.opti_ik.set_value(self.opti_parameters['reg_weight'], reg_weight)
        
        if target_rot is not None:
            self.opti_ik.set_value(self.opti_parameters['target_rot'], target_rot)
            self.opti_ik.set_value(self.opti_parameters['use_orientation'], 1)
        else:
            self.opti_ik.set_value(self.opti_parameters['target_rot'], np.eye(3))  # 填充默认值
            self.opti_ik.set_value(self.opti_parameters['use_orientation'], 0)
        
        # 设置初始猜测和迭代限制
        self.opti_ik.set_initial(self.opti_variables['q'], q_init)
        #self.opti_ik.solver('ipopt', {'ipopt': {'max_iter': max_iter}})
        
        try:
            sol = self.opti_ik.solve()
            q_opt = sol.value(self.opti_variables['q'])
            error = sol.value(casadi.norm_2(self.opti_ik.g[0]))  # 获取主要误差项
            
            # 修改误差计算方式
            if target_rot is not None:
                error = self.compute_pose_error(q_opt, target_pos, target_rot)
            else:
                # 仅取前3个元素作为位置误差
                error = self.compute_position_error(q_opt, target_pos)
            
            error_norm = np.linalg.norm(error)
            return q_opt, error_norm < tol
        
        except Exception as e:
            print(f"优化失败: {str(e)}")
            q_debug = self.opti_ik.debug.value(self.opti_variables['q'])

            return q_debug, False
        
    def compute_joint_jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        计算雅可比矩阵
        
        Args:
            q: 当前关节角度
            frame_id: 末端执行器frame ID
            
        Returns:
            雅可比矩阵
        """
        pin.computeJointJacobians(self.model, self.data, q)
        J = pin.getFrameJacobian(self.model, self.data, self.endEffector_ID, pin.ReferenceFrame.WORLD)

        return J
    
    def compute_error(self, q: np.ndarray, 
                    target_position: np.ndarray,
                    target_orientation: np.ndarray,
                    frame_id: int) -> Tuple[np.ndarray, float]:
        """
        计算当前位姿与目标位姿之间的误差
        
        Args:
            q: 当前关节角度
            target_position: 目标位置 [x, y, z]
            target_orientation: 目标姿态(旋转矩阵)
            frame_id: 末端执行器frame ID
            
        Returns:
            误差向量和误差范数
        """
        # 更新机器人状态
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        # 获取当前末端执行器位姿
        current_pose = self.data.oMf[self.endEffector_ID]
        current_position = current_pose.translation
        current_orientation = current_pose.rotation
        
        # 计算位置误差(在世界坐标系下)
        position_error = target_position - current_position
        
        # 计算姿态误差(在世界坐标系下)
        orientation_error = pin.log3(current_orientation.T @ target_orientation)
        
        # 组合误差向量
        error = np.concatenate([position_error, orientation_error])
        error_norm = np.linalg.norm(error)
        
        return error, error_norm

    def solve_ik(self, 
                 q_init: np.ndarray,
                 target_position: np.ndarray,
                 target_orientation: np.ndarray,
                 damping: float = 1e-6,
                 max_iter: int = 1000,
                 eps: float = 1e-5) -> Tuple[np.ndarray, bool]:
        """
        求解逆运动学
        
        Args:
            q_init: 初始关节角度
            target_position: 目标位置 [x, y, z]
            target_orientation: 目标姿态(旋转矩阵)
            frame_id: 末端执行器frame ID
            damping: 阻尼因子
            
        Returns:
            求解得到的关节角度和是否成功的标志
        """
        q = q_init.copy()
        
        for i in range(max_iter):
            # 计算误差
            error, error_norm = self.compute_error(q, target_position, target_orientation, self.endEffector_ID)
            
            # 检查是否收敛
            if error_norm < eps:
                return q, True
                
            # 计算雅可比矩阵(在世界坐标系下)
            J = self.compute_joint_jacobian(q)
            
            # 使用阻尼最小二乘法求解
            JJt = J @ J.T
            lambda_eye = damping * np.eye(6)
            pinv_J = J.T @ np.linalg.inv(JJt + lambda_eye)
            
            # 更新关节角度
            dq = pinv_J @ error
            q = pin.integrate(self.model, q, dq)
            
        return q, False

    def solve_acados_ik(self, q_init, target_pos, target_rot=None, 
                       max_iter=50, tol=1e-4, reg_weight=0.01):
        """
        使用Acados加速求解逆运动学（带关节约束）
        
        Args:
            q_init (np.ndarray): 初始关节角猜测
            target_pos (np.ndarray): 目标位置
            target_rot (np.ndarray, optional): 目标旋转矩阵
            max_iter (int): 最大迭代次数
            tol (float): 收敛容差
            reg_weight (float): 正则化项权重
            
        Returns:
            np.ndarray: 优化后的关节角度
            bool: 是否收敛
        """
        try:
            from acados_template import AcadosOcp, AcadosOcpSolver
        except ImportError:
            print("未检测到Acados安装，请参考https://docs.acados.org/installation/")
            return q_init, False

        # 创建OCP问题
        ocp = AcadosOcp()
        ocp.model = self._build_acados_model(target_pos, target_rot)
        
        # 设置维度参数
        ocp.dims.N = 1
        ocp.dims.nx = 2 * self.model.nq  # 状态维度=12
        ocp.dims.nu = self.model.nq       # 控制维度=6
        
        # 设置状态约束（关节角度限制）
        ocp.constraints.lbx = self.joint_limits['lower']
        ocp.constraints.ubx = self.joint_limits['upper']
        ocp.constraints.idxbx = np.arange(self.model.nq)  # 约束前nq个状态变量
        
        # 修改求解器选项
        ocp.solver_options.hessian_approx = 'EXACT'  # 使用精确Hessian
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'  # 更稳定的求解器
        
        # 添加时间范围设置
        ocp.solver_options.tf = 1.0  # 总时间范围设为1秒
        ocp.solver_options.sim_method_num_stages = 1  # 积分器阶段数
        ocp.solver_options.sim_method_num_steps = 1   # 积分步数
        
        # 创建求解器
        solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

        # 设置初始状态（包含速度项）
        x_init = np.concatenate([q_init, np.zeros(self.model.nq)])
        solver.set(0, "x", x_init)

        # 求解并提取结果
        status = solver.solve()
        q_opt_full = solver.get(0, "x")
        q_opt = q_opt_full[:self.model.nq]  # 提取前6个关节角度
        
        # 修正错误计算逻辑
        if target_rot is not None:
            error = self.compute_pose_error(q_opt, target_pos, target_rot)
        else:
            error = self.compute_position_error(q_opt, target_pos)
        
        error_norm = np.linalg.norm(error)
        return q_opt, error_norm < tol

    def _build_acados_model(self, target_pos, target_rot):
        """构建Acados模型"""
        from acados_template import AcadosModel
        model = AcadosModel()
        
        # 状态变量（关节角度+速度）
        q = casadi.SX.sym('q', self.model.nq)
        qdot = casadi.SX.sym('qdot', self.model.nq)
        x = casadi.vertcat(q, qdot)  # 总维度为12
        
        # 控制输入变量
        u = casadi.SX.sym('u', self.model.nq)
        model.u = u
        
        # 目标函数
        if target_rot is not None:
            error = self.error6_tool(q, target_pos, target_rot)
        else:
            error = self.error3_tool(q, target_pos)
            
        cost = casadi.dot(error, error) + 0.01 * casadi.dot(u, u)  # 添加控制输入正则项
        
        # 动力学模型（包含控制输入）
        f_expl = casadi.vertcat(qdot, u - 0.1*q)  # 二阶系统模型
        
        model.name = 'robot_ik'
        model.x = x
        model.f_expl_expr = f_expl
        model.cost_expr_ext_cost = cost
        
        return model

if __name__ == "__main__":
    # Example usage
    model_path = "urdf/rm_65.xml"
    robot = RobotKinematics(model_path, "Link6")
    
    # Create example target pose
    target_pos = np.array([0.35 ,0 ,0.6])
    target_rot = pin.utils.rotate('y', 0)
    target_SE3 = pin.SE3(target_rot, target_pos)
    
    # Example joint configuration
    q = np.zeros(robot.model.nq)
    
    # Compute errors
    pos_error = robot.compute_position_error(q, target_pos)
    pose_error = robot.compute_pose_error(q, target_pos, target_rot)
    
    print("Position error:", pos_error)
    print("Pose error:", pose_error)
    
    # Solve trajectory optimization
    sol_qs = robot.solve_trajectory_optimization(q, target_pos, target_rot)

    pin.forwardKinematics(robot.model, robot.data, sol_qs[-1])
    pin.updateFramePlacements(robot.model, robot.data)
    eef_pos = robot.data.oMf[robot.endEffector_ID].translation
    print(eef_pos)

    print(sol_qs[-1])

    # Acados求解测试
    q_acados, acados_conv = robot.solve_acados_ik(
        q_init=q,
        target_pos=target_pos,
        target_rot=target_rot,
        tol=1e-4
    )
    print(f"\nAcados求解结果: 收敛={acados_conv}, 关节角度={q_acados}")