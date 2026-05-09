from humanoid.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class TWBotCfg(LeggedRobotCfg):
    class env(LeggedRobotCfg.env):
        frame_stack = 15
        c_frame_stack = 3
        num_single_obs = 47
        num_observations = int(frame_stack * num_single_obs)
        single_num_privileged_obs = 73
        num_privileged_obs = int(c_frame_stack * single_num_privileged_obs)
        num_actions = 12
        num_envs = 2048
        episode_length_s = 24
        use_ref_actions = False
        env_spacing = 2.

    class safety:
        pos_limit = 1.0
        vel_limit = 1.0
        torque_limit = 0.85

    class asset(LeggedRobotCfg.asset):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/TWBot/urdf/Assembly.urdf'
        name = "TWBot"
        foot_name = "foot"
        knee_name = "knee_linkage"
        terminate_after_contacts_on = ["base_link"]
        penalize_contacts_on = ["base_link"]
        self_collisions = 1  # 1 to disable, 0 to enable...bitwise filter
        # collision已换成primitive(box/cylinder)，但相邻link primitive仍可能轻微重叠，暂保持禁用
        flip_visual_attachments = False
        replace_cylinder_with_capsule = False
        fix_base_link = False
        # hip_linkage=0.9g, knee_linkage=0.5g等极轻连杆，armature=0时关节角加速度达50万rad/s²
        # armature给每个DOF添加0.001 kg⋅m²虚拟惯量，防止物理爆炸
        armature = 0.001

        controlled_joint_names = [
            'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
            'left_knee_pitch_joint',  'left_ankle_yaw_joint', 'left_ankle_pitch_joint',
            'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
            'right_knee_pitch_joint', 'right_ankle_yaw_joint', 'right_ankle_pitch_joint',
        ]
                                                                                           
    
    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'plane'
        curriculum = False
        measure_heights = False
        static_friction = 0.6
        dynamic_friction = 0.6
        terrain_length = 8.
        terrain_width = 8.
        num_rows = 20  # number of terrain rows (levels)
        num_cols = 20  # number of terrain cols (types)
        max_init_terrain_level = 10  # starting curriculum state
        # plane; obstacles; uniform; slope_up; slope_down, stair_up, stair_down
        terrain_proportions = [0.2, 0.2, 0.4, 0.1, 0.1, 0, 0]
        restitution = 0.

    class noise:
        add_noise = True
        noise_level = 0.6    # scales other values
        class noise_scales:
            dof_pos = 0.05
            dof_vel = 0.5
            ang_vel = 0.1
            lin_vel = 0.05
            quat = 0.03
            height_measurements = 0.1

    class init_state(LeggedRobotCfg.init_state):
        # URDF关节z轴累加: hip(-0.011)+knee(-0.023)+ankle(-0.061)+foot(-0.011) ≈ -0.22m
        # 所有关节为0时脚在base_link以下0.22m，初始z=0.25保证脚在地面上方约3cm，自然落地
        pos = [0.0, 0.0, 0.25]
        default_joint_angles = {
            # 腿部 12个（从站立视频重定向获得）
            'left_hip_pitch_joint':   0.0611,
            'left_hip_roll_joint':   -0.0893,
            'left_hip_yaw_joint':     0.0453,
            'left_knee_pitch_joint':  0.0000,
            'left_ankle_yaw_joint':  -0.0327,
            'left_ankle_pitch_joint': -0.1500,
            'right_hip_pitch_joint':  0.0294,
            'right_hip_roll_joint':   0.0926,
            'right_hip_yaw_joint':    0.0257,
            'right_knee_pitch_joint': -0.1050,
            'right_ankle_yaw_joint':  -0.0000,
            'right_ankle_pitch_joint': 0.1500,
            # 腰部 3个
            'waist_yaw_joint':   0.3302,
            'waist_pitch_joint': -0.0204,
            'waist_roll_joint':  -0.0249,
            # 左臂 5个
            'left_shoulder_pitch_joint': -0.0234,
            'left_shoulder_roll_joint':  -0.2854,
            'left_shoulder_yaw_joint':   -0.2664,
            'left_elbow_pitch_joint':    -0.0000,
            'left_wrist_yaw_joint':      -0.1901,
            # 右臂 5个
            'right_shoulder_pitch_joint': -0.0573,
            'right_shoulder_roll_joint':   0.2854,
            'right_shoulder_yaw_joint':   -0.2108,
            'right_elbow_pitch_joint':     0.0000,
            'right_wrist_yaw_joint':      -0.1833,
            # 头部 3个
            'neck_yaw_joint':   0.,
            'neck_roll_joint':  0.,
            'neck_pitch_joint': 0.,
        }
        
    class control(LeggedRobotCfg.control):
        # PD Drive parameters
        # TWBot ~1kg，腿部关节实际所需扭矩 0.1-0.5 Nm
        # 目标：max_torque = Kp × action_scale × clip_actions ≈ 0.5 Nm (hip), 0.2 Nm (ankle)
        # 1.5 × 0.1 × 3.0 = 0.45 Nm (hip) — 合理
        # 验算（armature=0.001时）：ζ = Kd / (2*sqrt(Kp*0.001))
        # hip:   ζ = 0.05/(2*sqrt(1.5*0.001)) = 0.65 ✓ 阻尼良好
        # knee:  ζ = 0.05/(2*sqrt(1.5*0.001)) = 0.65 ✓
        # ankle: ζ = 0.05/(2*sqrt(2.0*0.001)) = 0.56 ✓
        # 踝关节最大扭矩: 2.0×0.1×3.0 = 0.60 Nm
        # 机器人648g，COM在踝上0.22m，10°倾斜需0.25Nm → 0.60Nm有2.4倍余量
        stiffness = {
            'hip_pitch': 1.5,
            'hip_roll': 1.5,
            'hip_yaw': 1.0,
            'knee_pitch': 1.5,
            'ankle_yaw': 1.0,
            'ankle_pitch': 2.0,  # 原0.5→2.0: 头重258g，0.5时最大扭矩0.15Nm不够平衡，必倒
            'waist': 1.0,
            'shoulder': 0.5,
            'elbow': 0.3,
            'wrist': 0.3,
            'neck': 3.0,   # 原0.1→3.0: head=258g，0.1时max_torque=0.03Nm撑不住，头必然乱晃
        }
        damping = {
            'hip_pitch': 0.05,
            'hip_roll': 0.05,
            'hip_yaw': 0.05,
            'knee_pitch': 0.05,
            'ankle_yaw': 0.02,
            'ankle_pitch': 0.05,
            'waist': 0.05,
            'shoulder': 0.01,
            'elbow': 0.005,
            'wrist': 0.01,
            'neck': 0.1,   # 配合Kp=3.0，ζ=0.1/(2*sqrt(3.0*0.001))=0.91，过阻尼，头部静止不抖
        }
        action_scale = 0.1  # max_torque = 1.5×0.1×3 = 0.45 Nm/joint
        # 控制频率90Hz
        # sim dt=0.001，decimation=1000/90≈11
        decimation = 10  # 约90Hz

    class sim(LeggedRobotCfg.sim):
        dt = 0.001  # 1000 Hz
        substeps = 1
        up_axis = 1  # 0 is y, 1 is z

        class physx(LeggedRobotCfg.sim.physx):
            num_threads = 10
            solver_type = 1  # 0: pgs, 1: tgs
            num_position_iterations = 8  # 28-DOF复杂机器人需要更多迭代保证约束精度
            num_velocity_iterations = 1
            contact_offset = 0.005  # [m]
            rest_offset = 0.0   # [m]
            bounce_threshold_velocity = 0.5  # [m/s] 提高阈值，低速接触不产生弹力
            max_depenetration_velocity = 0.1  # 1.0→0.1：轻体(50g base_link)被1m/s推开会飞出去
            max_gpu_contact_pairs = 2**22  # 2048 envs用2**22，4096需2**23会OOM
            default_buffer_size_multiplier = 5
            # 0: never, 1: last sub-step, 2: all sub-steps (default=2)
            contact_collection = 2

    class domain_rand:
        # ======== 调试阶段：全部关闭，确认基础物理稳定后再逐步开启 ========
        # 开启顺序建议：① push_robots → ② randomize_friction → ③ randomize_base_mass → ④ action_delay/noise
        randomize_friction = False
        friction_range = [0.1, 2.0]
        randomize_base_mass = False
        added_mass_range = [-0.01, 0.05]  # base_link仅50g，开启时不能设太大
        push_robots = False
        push_interval_s = 4
        max_push_vel_xy = 0.05
        max_push_ang_vel = 0.4
        # dynamic randomization
        action_delay = 0.0
        action_noise = 0.0

    class commands(LeggedRobotCfg.commands):
        # Vers: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        num_commands = 4
        resampling_time = 8.  # time before command are changed[s]
        heading_command = True  # if true: compute ang vel command from heading error

        class ranges:
            lin_vel_x = [-0.2, 0.4]   # min max [m/s]
            lin_vel_y = [-0.2, 0.2]   # min max [m/s]
            ang_vel_yaw = [-0.3, 0.3] # min max [rad/s]
            heading = [-3.14, 3.14]

    class rewards:
        # 关节链z向累加: hip_pitch(+0.011)+hip_roll(-0.022)+hip_yaw(-0.060)+knee(-0.023)+ankle_yaw(-0.050)+ankle_pitch(-0.061) = -0.204m
        # 脚底面距ankle_pitch额外-0.024m，共-0.228m。足底接触地面时base约在z=0.228m
        # _reward_base_height计算: root_z - feet_z + 0.01 ≈ 0.228+0.010 = 0.238m
        base_height_target = 0.23
        
        min_dist = 0.06             # 改小，两脚最小间距，迷你机器人腿间距小
        max_dist = 0.15            # 改小，两脚最大间距，XBot是0.5
        
        target_joint_pos_scale = 0.17  # 暂时不用改
        target_feet_height = 0.02     # 改小，脚抬起目标高度
                                        # XBot是0.06m，迷你机器人腿短改成0.03m
        cycle_time = 0.64              # 暂时不用改，后续根据实际步频调整
        
        only_positive_rewards = False  # True时奖励被clip到0，消除梯度；False才能从负奖励中学习
        tracking_sigma = 5             # exp(-sigma*err²)，sigma越大越严格
        max_contact_force = 40        # 改小！XBot是700N，你们迷你机器人
                                        # 总质量就几kg，接触力不会那么大
                                        # 大概设成总质量×重力加速度×3倍左右

        class scales:
            # === 阶段三：强制走路，站立奖励压到最低 ===
            orientation = 1.5       # 1→1.5: 补回平衡激励，走路时能撑住不立刻倒
            base_height = 0.5       # 0.2→0.5: 维持一定高度防止趴倒
            default_joint_pos = 0.2
            base_acc = 0.2
            # 步态奖励
            joint_pos = 0.
            feet_contact_number = 0.
            feet_distance = 0.2
            knee_distance = 0.2
            vel_mismatch_exp = 0.2
            feet_clearance = 0.3
            feet_air_time = 5.0
            foot_slip = -0.02
            feet_contact_forces = -0.005
            # 速度跟踪：主导奖励
            tracking_lin_vel = 2.0  # 1.5→2.0
            tracking_ang_vel = 1.0
            low_speed = 1.5         # 0.2→1.5: 站着不动时每步-1.5，2400步=-3600，必须走
            track_vel_hard = 0.
            # 惩罚项
            action_smoothness = -0.02
            torques = -1e-5
            dof_acc = 0.
            dof_vel = 0.
            collision = -1.

    class normalization:
        class obs_scales:
            lin_vel = 2.
            ang_vel = 1.
            dof_pos = 1.
            dof_vel = 0.05
            quat = 1.
            height_measurements = 5.0
        clip_observations = 18.
        clip_actions = 3.  # 3×0.1=0.3 rad ≈ 17°，限制随机policy初始扭矩
    
class TWBotCfgPPO(LeggedRobotCfgPPO):
    seed = 5
    runner_class_name = 'OnPolicyRunner'   # DWLOnPolicyRunner

    class policy:
        init_noise_std = 1.0
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [768, 256, 128]

    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.001
        learning_rate = 3e-5
        num_learning_epochs = 4
        gamma = 0.994
        lam = 0.9
        num_mini_batches = 4

    class runner:
        policy_class_name = 'ActorCritic'
        algorithm_class_name = 'PPO'
        num_steps_per_env = 120  # per iteration，更长的rollout减少梯度估计方差
        max_iterations = 3001  # number of policy updates

        # logging
        save_interval = 100  # Please check for potential savings every `save_interval` iterations.
        experiment_name = 'TWBot_ppo'
        run_name = ''
        # Load and resume
        resume = False
        load_run = -1  # -1 = last run
        checkpoint = -1  # -1 = last saved model
        resume_path = None  # updated from load_run and chkpt
