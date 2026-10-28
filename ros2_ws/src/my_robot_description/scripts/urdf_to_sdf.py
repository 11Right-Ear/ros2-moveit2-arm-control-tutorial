#!/usr/bin/env python3
"""把 URDF 转成 Gazebo Classic 能直接用的 SDF。

为什么要多这一步？
----------------
1) 位姿语义
   sdformat 自带的 URDF -> SDF 转换器（``gz sdf -p``）输出的是 SDF 1.7 写法：

       <joint name='joint1' type='revolute'>
         <pose relative_to='base_link'>0 0 0.1 0 -0 0</pose>
       <link name='shoulder_link'>
         <pose relative_to='joint1'>0 0 0 0 -0 0</pose>

   也就是：link 的位姿写成"相对某个 joint"，joint 的位姿写成"相对父 link"。
   但 Gazebo Classic 11 的物理引擎只实现了 SDF 1.6 的位姿语义，解析不了
   指向 link / joint 的 ``relative_to``，结果所有连杆都被摆到模型原点，
   表现就是"只看得见底座、看不见机械臂"。
   用 ``gz sdf -v 1.6 -p`` 也不行：sdformat 只是把 ``relative_to`` 属性丢掉，
   link 的位姿会变成 0 0 0（相对于 joint 的偏移被当成模型原点的偏移）。

   所以这里自己把位姿换算成 Gazebo Classic 认的写法：

     * link  <pose>：**模型坐标系下的绝对位姿**
       （Gazebo Classic 就是按 link 自己的 <pose> 摆放连杆的）
     * joint <pose>：**关节坐标系相对父连杆的位姿**，也就是 URDF 里 joint 的
       <origin>，原样保留即可（这正是 sdformat 转换器写 relative_to='父link'
       时想表达的东西）。

       注意别把 joint 的 <pose> 写成单位位姿：只有当手臂摆成 URDF 的零位时
       两者才等价，一旦关节有转角（见下面第 2 条），关节轴的位置就会和连杆
       实际位置对不上，物理引擎会认为约束被违反，关节链会被硬拽成一团。

2) 出生姿态
   手臂笔直竖着出生是不稳定的平衡状态（质心在底座上方，绕各转轴的力臂
   又接近 0），ODE 求解器会直接发散，模型不是炸开就是整条塌回原点。
   这里把 URDF 里 ``<ros2_control>`` 中各关节的 ``initial_value`` 当作
   "出生姿态"，按运动学正解算好每个连杆的绝对位姿写进 SDF，
   这样模型一进 Gazebo 就是稳定的（gazebo_ros2_control 也会用同一个
   initial_value 设置关节位置，两边天然一致）。

用法：
    urdf_to_sdf.py <input.urdf> <output.sdf>
"""

import math
import subprocess
import sys
import xml.etree.ElementTree as ET

# 齐次变换：平移 + 四元数
IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def rpy_to_quat(roll, pitch, yaw):
    """把 URDF/SDF 的 rpy 角转成四元数 (x, y, z, w)。"""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_mul(a, b):
    """四元数乘法 a * b。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * by - ay * bz - az * bx)


def quat_inv(q):
    """四元数求逆（单位四元数就是共轭）。"""
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_rot(q, v):
    """用四元数 q 旋转向量 v。"""
    qx, qy, qz, qw = q
    ux, uy, uz = qx, qy, qz
    tx = 2.0 * (uy * v[2] - uz * v[1])
    ty = 2.0 * (uz * v[0] - ux * v[2])
    tz = 2.0 * (ux * v[1] - uy * v[0])
    return (v[0] + qw * tx + uy * tz - uz * ty,
            v[1] + qw * ty + uz * tx - ux * tz,
            v[2] + qw * tz + ux * ty - uy * tx)


def parse_vector(text, default=(0.0, 0.0, 0.0)):
    """解析 'x y z' 形式的字符串。"""
    if not text:
        return default
    values = [float(v) for v in text.split()]
    return tuple(values) + (0.0,) * (3 - len(values))


def parse_pose(text):
    """解析 'x y z roll pitch yaw'，返回 ((x,y,z), quat)。"""
    if not text:
        return IDENTITY
    values = [float(v) for v in text.split()]
    if len(values) != 6:
        raise ValueError('位姿格式不对: %r' % text)
    return (tuple(values[0:3]), rpy_to_quat(*values[3:6]))


def compose(a, b):
    """位姿相乘：a * b（先走 a 再走 b）。"""
    (ta, qa), (tb, qb) = a, b
    rotated = quat_rot(qa, tb)
    return (tuple(ta[i] + rotated[i] for i in range(3)), quat_mul(qa, qb))


def format_pose(pose):
    """把位姿转回 'x y z roll pitch yaw' 字符串。"""
    (t, q) = pose
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return '%g %g %g %g %g %g' % (t[0], t[1], t[2], roll, pitch, yaw)


def axis_quat(axis, angle):
    """绕单位轴 axis 转 angle 弧度的四元数。"""
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    x, y, z = x / norm, y / norm, z / norm
    s = math.sin(angle / 2.0)
    return (x * s, y * s, z * s, math.cos(angle / 2.0))


# ------------------------------------------------------------------ 读 URDF
def read_urdf(urdf_path):
    """从 URDF 里取运动学结构和各关节的出生角度（initial_value）。"""
    root = ET.parse(urdf_path).getroot()

    # 各关节的"出生角度"：来自 <ros2_control> 里 position 状态接口的 initial_value
    angles = {}
    control = root.find('ros2_control')
    if control is not None:
        for joint in control.findall('joint'):
            for interface in joint.findall('state_interface'):
                if interface.get('name') != 'position':
                    continue
                for param in interface.findall('param'):
                    if param.get('name') == 'initial_value':
                        angles[joint.get('name')] = float(param.text)

    joints = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        xyz = parse_vector(origin.get('xyz') if origin is not None else None)
        rpy = parse_vector(origin.get('rpy') if origin is not None else None)
        axis_element = joint.find('axis')
        axis = parse_vector(axis_element.get('xyz') if axis_element is not None else None,
                            (0.0, 0.0, 1.0))
        joints[joint.get('name')] = {
            'parent': joint.find('parent').get('link'),
            'child': joint.find('child').get('link'),
            'type': joint.get('type'),
            'origin': ((xyz), rpy_to_quat(*rpy)),
            'axis': axis,
            'angle': angles.get(joint.get('name'), 0.0),
        }
    return joints


def solve_pose(joints, root_links):
    """正解：算出每个连杆的绝对位姿和每个关节相对子连杆的位姿。"""
    link_pose = {}
    for name in root_links:
        link_pose[name] = IDENTITY

    # 按父子关系排出关节的处理顺序
    pending = dict(joints)
    while pending:
        progressed = False
        for name in list(pending):
            joint = pending[name]
            parent = joint['parent']
            if parent not in link_pose:
                continue
            del pending[name]
            progressed = True

            rotation = (IDENTITY[1] if joint['type'] == 'fixed'
                        else axis_quat(joint['axis'], joint['angle']))
            parent_pose = link_pose[parent]
            # 子连杆坐标系 = 父连杆坐标系 * 原点偏移 * 绕轴旋转
            link_pose[joint['child']] = compose(compose(parent_pose, joint['origin']),
                                                ((0.0, 0.0, 0.0), rotation))
        if not progressed:
            raise RuntimeError('URDF 的关节结构里有环或者找不到根: %s' % list(pending))
    return link_pose


# ------------------------------------------------------------- URDF -> SDF
def urdf_to_sdf_text(urdf_path):
    """调用 sdformat 的 URDF -> SDF 转换器（只用来转换几何/质量/惯量）。"""
    result = subprocess.run(['gz', 'sdf', '-p', urdf_path],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            universal_newlines=True)
    if result.returncode != 0:
        raise RuntimeError('gz sdf -p 转换失败:\n%s' % result.stderr)
    if '<model' not in result.stdout:
        raise RuntimeError('转换结果里没有 <model>:\n%s' % result.stderr)
    return result.stdout


def build_sdf(urdf_path, sdf_text):
    """把转换出来的 SDF 改成 Gazebo Classic 认的位姿写法。"""
    root = ET.fromstring(sdf_text)
    model = root.find('model')
    if model is None:
        raise RuntimeError('SDF 里找不到 <model>')

    # 运动学（关节结构 + 出生角度）从 URDF 取，几何从转换结果取
    joints = read_urdf(urdf_path)
    urdf_root = ET.parse(urdf_path).getroot()
    child_links = set(j['child'] for j in joints.values())
    root_links = [link.get('name') for link in urdf_root.findall('link')
                  if link.get('name') not in child_links]
    link_pose = solve_pose(joints, root_links)

    # link：写绝对位姿
    for link in model.findall('link'):
        name = link.get('name')
        pose_element = link.find('pose')
        if pose_element is None:
            pose_element = ET.Element('pose')
            link.insert(0, pose_element)
        pose_element.attrib.pop('relative_to', None)
        pose_element.text = format_pose(link_pose.get(name, IDENTITY))

    # joint：写"关节坐标系相对子连杆"的位姿
    #   Gazebo Classic 里 joint 的 <pose> 是关节坐标系在**子连杆**里的位置
    #   （见 gazebo/physics/Joint.cc: WorldPose() = anchorPose + 子连杆位姿）。
    #   URDF 里子连杆的坐标系就坐在关节坐标系上，所以零位时是单位位姿；
    #   关节转了 q 之后，子连杆相对关节转了 q，关节坐标系在子连杆里就转 -q。
    for joint in model.findall('joint'):
        name = joint.get('name')
        pose_element = joint.find('pose')
        if pose_element is None:
            pose_element = ET.Element('pose')
            joint.insert(0, pose_element)
        pose_element.attrib.pop('relative_to', None)
        info = joints.get(name)
        if info is None or info['type'] == 'fixed':
            pose_element.text = format_pose(IDENTITY)
        else:
            rotation = axis_quat(info['axis'], info['angle'])
            pose_element.text = format_pose(((0.0, 0.0, 0.0), quat_inv(rotation)))

    # 其它地方如果还有 relative_to（例如 visual/collision），一并去掉
    for element in root.iter():
        element.attrib.pop('relative_to', None)

    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode'), link_pose


def main(argv):
    if len(argv) != 3:
        print(__doc__)
        print('用法: urdf_to_sdf.py <input.urdf> <output.sdf>')
        return 1

    urdf_path, sdf_path = argv[1], argv[2]
    sdf_text, link_pose = build_sdf(urdf_path, urdf_to_sdf_text(urdf_path))

    with open(sdf_path, 'w') as handle:
        handle.write(sdf_text)

    print('[urdf_to_sdf] 已生成 %s' % sdf_path)
    for name in sorted(link_pose):
        print('  %-24s %s' % (name, format_pose(link_pose[name])))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
