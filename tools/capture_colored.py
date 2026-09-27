#!/usr/bin/env python3
"""订阅 /cones/colored，把收到的点云（点 + RGB）持续覆盖写入文本文件。

节点每帧都会重发「当前已锁定」的全部锥桶，因此文件里始终保存着最新结果。
必须在 ros_noetic 环境里运行（需要 rospy）。

用法：
    python tools/capture_colored.py <输出文件>
"""
import struct
import sys

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

out_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/_cones_colored.txt"
trend_path = out_path + ".trend"


LOCKED_RED = (255, 0, 0)
LOCKED_BLUE = (0, 80, 255)
PROV_RED = (255, 120, 120)
PROV_BLUE = (120, 170, 255)


def cb(msg):
    rows = []
    for p in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=False):
        packed = struct.unpack("I", struct.pack("f", p[3]))[0]
        b = packed & 0xFF
        g = (packed >> 8) & 0xFF
        r = (packed >> 16) & 0xFF
        rows.append((p[0], p[1], p[2], r, g, b))
    with open(out_path, "w") as f:
        f.write("# x y z r g b  (frame_id=%s, n=%d)\n" % (msg.header.frame_id, len(rows)))
        for q in rows:
            f.write("%.3f %.3f %.3f %d %d %d\n" % q)

    # 按颜色精确分类：锁定（全亮）/ 暂定（降亮）
    def cnt(rgb):
        return sum(1 for q in rows if (q[3], q[4], q[5]) == rgb)
    with open(trend_path, "a") as f:          # 每次消息追加，用于观察输出是否稳定
        f.write("%.3f %d %d %d %d %d\n" % (
            msg.header.stamp.to_sec(), len(rows),
            cnt(LOCKED_RED), cnt(LOCKED_BLUE), cnt(PROV_RED), cnt(PROV_BLUE)))


rospy.init_node("capture_colored", anonymous=True)
rospy.Subscriber("/cones/colored", PointCloud2, cb, queue_size=1)
rospy.loginfo("capture_colored 已启动 -> %s", out_path)
rospy.spin()
