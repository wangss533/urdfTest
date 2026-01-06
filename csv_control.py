import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import csv
import time
import os

class DualArmCsvController(Node):
    def __init__(self):
        super().__init__('dual_arm_csv_controller')
        
        # ================= 配置区域 =================
        
        # 1. CSV 文件路径 (请修改为你电脑上的实际绝对路径)
        self.csv_file_path = '/home/desaysv/WorkSpace/wss/code/xsens_control/parallel-control/mycode/left_arm_joints.csv' 
        
        # 2. 播放频率 (Hz) 
        # 如果你的数据是每 0.02秒 记录一次，这里就填 50
        self.frequency = 10 
        
        # 3. 是否循环播放
        self.loop_playback = True
        
        # ===========================================

        # 定义所有可能的关节名称 (用于匹配 CSV 表头)
        self.supported_joints = [
            'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3', 'left_arm_joint4',
            'left_arm_joint5', 'left_arm_joint6', 'left_arm_joint7',
            'right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3', 'right_arm_joint4',
            'right_arm_joint5', 'right_arm_joint6', 'right_arm_joint7',
            'torso_joint1', 'torso_joint2' # 如果 CSV 里有躯干数据也能控
        ]
        
        # 存储发布者
        self.publishers_dict = {}
        
        # 存储读取到的 CSV 数据
        self.motion_data = [] 
        self.headers = []
        
        # 1. 读取 CSV 文件
        self.load_csv_data()
        
        # 2. 为 CSV 中出现的关节创建发布者
        self.init_publishers()

        # 3. 启动定时器
        if len(self.motion_data) > 0:
            period = 1.0 / self.frequency
            self.timer = self.create_timer(period, self.timer_callback)
            self.current_index = 0
            self.get_logger().info(f'开始播放，频率: {self.frequency}Hz, 总帧数: {len(self.motion_data)}')
        else:
            self.get_logger().error('数据为空，未启动定时器')

    def load_csv_data(self):
        """ 读取 CSV 文件到内存 """
        if not os.path.exists(self.csv_file_path):
            self.get_logger().error(f'找不到文件: {self.csv_file_path}')
            return

        try:
            with open(self.csv_file_path, 'r') as f:
                reader = csv.DictReader(f)
                self.headers = reader.fieldnames
                
                # 检查 CSV 表头是否有空
                if not self.headers:
                    self.get_logger().error('CSV 文件为空或格式错误')
                    return

                # 逐行读取数据
                for row in reader:
                    # 将字符串转换为浮点数
                    float_row = {}
                    for joint, value in row.items():
                        if joint in self.supported_joints:
                            try:
                                float_row[joint] = float(value)
                            except ValueError:
                                self.get_logger().warn(f'无法解析数值: {value} 在列 {joint}')
                    self.motion_data.append(float_row)
                    
            self.get_logger().info(f'成功加载 CSV 文件，共 {len(self.motion_data)} 行数据')
            
        except Exception as e:
            self.get_logger().error(f'读取 CSV 失败: {e}')

    def init_publishers(self):
        """ 根据 CSV 表头创建对应的 ROS 发布者 """
        if not self.headers:
            return

        for joint_name in self.headers:
            # 只有在支持列表里的关节才创建发布者
            # 这里的 strip() 是为了防止表头有空格
            clean_name = joint_name.strip()
            
            if clean_name in self.supported_joints:
                # 话题格式： /model/r1_pro/joint/<name>/cmd_pos
                topic_name = f'/model/r1_pro/joint/{clean_name}/cmd_pos'
                self.publishers_dict[clean_name] = self.create_publisher(Float64, topic_name, 10)
                self.get_logger().info(f'已初始化关节控制器: {clean_name}')

    def timer_callback(self):
        """ 定时发送每一帧数据 """
        if self.current_index < len(self.motion_data):
            # 获取当前帧的数据字典
            current_frame = self.motion_data[self.current_index]
            
            # 遍历字典，发送指令
            for joint_name, angle in current_frame.items():
                if joint_name in self.publishers_dict:
                    msg = Float64()
                    msg.data = angle
                    self.publishers_dict[joint_name].publish(msg)
            
            # 指针后移
            self.current_index += 1
            
        else:
            # 播放结束
            if self.loop_playback:
                self.current_index = 0
                self.get_logger().info('重新开始循环播放...')
            else:
                self.get_logger().info('播放完成')
                # 可以在这里选择停止定时器
                # self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = DualArmCsvController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()