#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import os

XML_PATH = os.path.join(os.path.dirname(__file__), "test_color_sensor.xml")

model = mujoco.MjModel.from_xml_path(XML_PATH)
data  = mujoco.MjData(model)

color_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "color_site")
joint_id      = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "sensor_slide")
qpos_adr      = model.jnt_qposadr[joint_id]
print(f"color_site id={color_site_id}  sensor_slide qpos_adr={qpos_adr}")

def rgb_to_hsv(r, g, b):
    max_c = max(r, g, b)
    min_c = min(r, g, b)
    delta = max_c - min_c
    val = int(max_c * 1000)
    sat = int((delta / max_c) * 1000) if max_c > 0 else 0
    if delta < 1e-6:
        return 0, sat, val
    if max_c == r:   h = 60.0 * (((g - b) / delta) % 6)
    elif max_c == g: h = 60.0 * ((b - r) / delta + 2)
    else:            h = 60.0 * ((r - g) / delta + 4)
    if h < 0: h += 360
    return int(h), sat, val

def read_color():
    sp = data.site_xpos[color_site_id].copy()
    down = np.array([0.0, 0.0, -1.0])
    site_bodyid = model.site_bodyid[color_site_id]
    geomid = np.array([-1], dtype=np.int32)
    normal = np.zeros(3)
    dist = mujoco.mj_ray(model, data, sp, down, None, 1,
                         site_bodyid, geomid, normal)
    if dist < 0 or geomid[0] < 0:
        return 0, 0, 0, "none"
    rgba = model.geom_rgba[geomid[0]]
    h, s, v = rgb_to_hsv(rgba[0], rgba[1], rgba[2])
    gname = mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_GEOM, geomid[0]) or "none"
    return h, s, v, gname

print("ビューアを起動します。sensor_slide_ctrl を動かしてください。")
print("ブロック配置（左→右）: 赤 / 青 / 黄 / 白 / 黒 / 緑")

prev_gname = None

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

        h, s, v, gname = read_color()

        # geom名が変わったときだけ表示
        if gname != prev_gname:
            slide_pos = data.qpos[qpos_adr]
            print(f"slide={slide_pos:+.4f}  geom={gname:15s}  "
                  f"HSV: hue={h:3d} sat={s:3d} val={v:3d}")
            prev_gname = gname
