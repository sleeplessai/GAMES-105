import numpy as np
from scipy.spatial.transform import Rotation as R

class Joint:
    def __init__(self, name=None, offset=None, seq=None, parent=None, index=None):
        self.name: str = name
        self.offset: str = offset
        self.seq: str = seq
        self.parent: int = parent
        self.index: int = -1

    def _dump(self):
        print(self.index, self.name, self.parent, self.offset, self.seq)

class Skeleton:
    def __init__(self):
        self.root: int = None
        self.skel: list[Joint] = []

    def append(self, x: Joint):
        self.skel.append(x)

    def present(self):
        jo_name, jo_parent, jo_offset = [], [], []

        for _, x in enumerate(self.skel):
            jo_name.append(x.name)
            jo_parent.append(x.parent)
            jo_offset.append(x.offset)

        return jo_name, jo_parent, jo_offset

    def _dump(self):
        print('Skel_tree root:', self.root)
        for x in self.skel:
            x._dump()

    # def from_bvh_file(bvh_file_path): pass

def part1_calculate_T_pose(bvh_file_path):
    """请填写以下内容
    输入： bvh 文件路径
    输出:
        joint_name: List[str]，字符串列表，包含着所有关节的名字
        joint_parent: List[int]，整数列表，包含着所有关节的父关节的索引,根节点的父关节索引为-1
        joint_offset: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的偏移量

    Tips:
        joint_name顺序应该和bvh一致
    """
    bvh_ctx = []

    with open(bvh_file_path, 'r') as f:
        buffer = f.readlines()
        for i, x in enumerate(buffer):
            x = x.strip()
            if x.startswith('MOTION'): break
            if x.startswith('HIERARCHY'): continue
            bvh_ctx.append(x)

    skel_tree = Skeleton()
    root: int = None
    jo: Joint = None
    _stack = []
    _index = 0

    for _, x in enumerate(bvh_ctx):
        x = [xx for xx in x.split(' ') if xx]
        if x[0] == 'ROOT':
            jo = Joint(x[1], parent=-1)
            skel_tree.root = _index
        elif x[0] == 'JOINT':
            jo = Joint(x[1], parent=_stack[-1])
        elif x[0] == 'End':
            jo = Joint(x[1], parent=_stack[-1])
        elif x[0] == 'OFFSET':
            o = np.array([float(a) for a in x[1:]])
            jo.offset = o
        elif x[0] == 'CHANNELS':
            seq = ''.join([a.replace('rotation', '') for a in x[2:] if 'rotation' in a])
            jo.seq = seq
        elif x[0] == '{':
            jo.index = _index
            skel_tree.append(jo)
            _stack.append(_index)
            _index += 1
        elif x[0] == '}':
            _stack.pop()

    return skel_tree.present()


def load_motion_data(bvh_file_path):
    """part2 辅助函数，读取bvh文件"""
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].startswith('Frame Time'):
                break
        motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            motion_data.append(np.array(data).reshape(1,-1))
        motion_data = np.concatenate(motion_data, axis=0)
    return motion_data


def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    """请填写以下内容
    输入: part1 获得的关节名字，父节点列表，偏移量列表
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数
        frame_id: int，需要返回的帧的索引
    输出:
        joint_positions: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
        joint_orientations: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
    Tips:
        1. joint_orientations的四元数顺序为(x, y, z, w)
        2. from_euler时注意使用大写的XYZ
    """
    joint_positions = None
    joint_orientations = None
    return joint_positions, joint_orientations


def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    """
    将 A-pose的bvh重定向到T-pose上
    输入: 两个bvh文件的路径
    输出:
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数。retarget后的运动数据
    Tips:
        两个bvh的joint name顺序可能不一致哦(
        as_euler时也需要大写的XYZ
    """
    motion_data = None
    return motion_data
