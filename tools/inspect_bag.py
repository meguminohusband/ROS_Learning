"""列出 timu.bag 里的话题，并查看第一帧点云的结构。

用法：
    cd ~/Documents/Python_Project/ROS
    ./.venv/bin/python tools/inspect_bag.py
"""
from pathlib import Path
from rosbags.highlevel import AnyReader

BAG = Path("timu.bag")

with AnyReader([BAG]) as reader:
    # ---------- 1. 列出所有话题 ----------
    print("=" * 60)
    print("话题列表")
    print("=" * 60)
    for conn in reader.connections:
        print(f"  话题名 : {conn.topic}")
        print(f"  类型   : {conn.msgtype}")
        print(f"  消息数 : {conn.msgcount}")
        print()

    # ---------- 2. 看第一帧点云 ----------
    print("=" * 60)
    print("第一帧点云的详细信息")
    print("=" * 60)
    conns = [c for c in reader.connections
             if c.topic == "/only_lidar_points_pub"]

    for conn, timestamp, rawdata in reader.messages(connections=conns):
        msg = reader.deserialize(rawdata, conn.msgtype)

        print(f"  坐标系 (frame_id)  : {msg.header.frame_id}")
        print(f"  点云尺寸           : {msg.width} x {msg.height}")
        print(f"  总共多少个点       : {msg.width * msg.height}")
        print(f"  每个点占多少字节   : {msg.point_step}")
        print(f"  有没有无效点       : {'没有' if msg.is_dense else '有'}")
        print()
        print("  每个点包含哪些字段：")
        # datatype 7 = FLOAT32, 2 = UINT8 等
        type_name = {1: "INT8", 2: "UINT8", 3: "INT16", 4: "UINT16",
                     5: "INT32", 6: "UINT32", 7: "FLOAT32", 8: "FLOAT64"}
        for f in msg.fields:
            print(f"    - {f.name:10s} 偏移={f.offset:3d} "
                  f"类型={type_name.get(f.datatype, f.datatype)}")
        break
