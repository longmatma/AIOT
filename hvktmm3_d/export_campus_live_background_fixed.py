# -*- coding: utf-8 -*-
"""
Tạo ảnh nền campus TOP-DOWN cố định cho giao diện Digital Twin trên Raspberry Pi.

Chạy MỘT LẦN trên Windows bằng Blender:
    & "E:\blender\blender.exe" --background --python ".\export_campus_live_background.py" -- ^
        --glb ".\campus_chinh_chieu_cao.glb"

Kết quả:
    campus_live_background.png
    campus_live_map_config.json

Hai file này sau đó copy lên Raspberry Pi cùng position_monitor_campus.py.
"""

import bpy
import sys
import json
import math
import argparse
from pathlib import Path
from mathutils import Vector


def parse_args():
    # Blender có thể truyền cả tham số riêng của Blender vào sys.argv.
    # Vì vậy chỉ lấy các tham số mà script này nhận biết và bỏ qua phần còn lại.
    p = argparse.ArgumentParser(add_help=True)
    p.add_argument("--glb", default="campus_chinh_chieu_cao.glb")
    p.add_argument("--width", type=int, default=2400)
    p.add_argument("--height", type=int, default=1700)
    p.add_argument("--padding", type=float, default=1.18)
    p.add_argument("--png", default="campus_live_background.png")
    p.add_argument("--json", default="campus_live_map_config.json")

    argv = sys.argv[1:]
    args, _unknown = p.parse_known_args(argv)
    return args


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)


def object_world_bounds(obj):
    pts = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    mn = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    mx = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return mn, mx


def scene_bounds(objects):
    mins, maxs = [], []
    for obj in objects:
        if obj.type != "MESH":
            continue
        mn, mx = object_world_bounds(obj)
        mins.append(mn)
        maxs.append(mx)

    if not mins:
        raise RuntimeError("Không tìm thấy mesh trong scene.")

    mn = Vector((
        min(v.x for v in mins),
        min(v.y for v in mins),
        min(v.z for v in mins),
    ))
    mx = Vector((
        max(v.x for v in maxs),
        max(v.y for v in maxs),
        max(v.z for v in maxs),
    ))
    return mn, mx


def setup_camera(mn, mx, width, height, padding):
    center = (mn + mx) * 0.5
    sx = mx.x - mn.x
    sy = mx.y - mn.y

    bpy.ops.object.camera_add()
    cam = bpy.context.active_object
    cam.name = "CAMPUS_LIVE_TOPDOWN_CAMERA"
    cam.data.type = "ORTHO"
    cam.location = (center.x, center.y, mx.z + max(sx, sy) + 80.0)

    # Camera Blender mặc định nhìn theo local -Z, nên rotation 0 là nhìn thẳng xuống.
    cam.rotation_euler = (0.0, 0.0, 0.0)

    aspect = width / height

    # Giữ cùng công thức với renderer đã dùng để tránh crop campus.
    ortho_width = sx * padding
    ortho_height_as_width = sy * padding * aspect
    visible_width = max(ortho_width, ortho_height_as_width)
    visible_height = visible_width / aspect

    cam.data.ortho_scale = visible_width
    cam.data.clip_start = 0.1
    cam.data.clip_end = 10000.0

    bpy.context.scene.camera = cam
    return cam, center, visible_width, visible_height


def setup_lighting(center, max_z):
    bpy.ops.object.light_add(
        type="SUN",
        location=(center.x, center.y, max_z + 100.0),
    )
    sun = bpy.context.active_object
    sun.name = "CAMPUS_LIVE_SUN"
    sun.data.energy = 2.0
    sun.rotation_euler = (
        math.radians(28.0),
        math.radians(-15.0),
        math.radians(-18.0),
    )

    world = bpy.context.scene.world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs["Color"].default_value = (0.17, 0.17, 0.17, 1.0)
        bg.inputs["Strength"].default_value = 0.82


def main():
    a = parse_args()
    base = Path(__file__).resolve().parent

    glb = Path(a.glb)
    if not glb.is_absolute():
        glb = base / glb

    out_png = Path(a.png)
    if not out_png.is_absolute():
        out_png = base / out_png

    out_json = Path(a.json)
    if not out_json.is_absolute():
        out_json = base / out_json

    if not glb.exists():
        raise FileNotFoundError(f"Không tìm thấy GLB: {glb}")

    clear_scene()
    bpy.ops.import_scene.gltf(filepath=str(glb))

    imported = list(bpy.context.scene.objects)
    mn, mx = scene_bounds(imported)

    cam, center, visible_width, visible_height = setup_camera(
        mn, mx, a.width, a.height, a.padding
    )
    setup_lighting(center, mx.z)

    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.eevee.taa_render_samples = 64
    scene.eevee.use_gtao = True
    scene.eevee.gtao_distance = 3.0
    scene.eevee.gtao_factor = 1.1
    scene.render.resolution_x = a.width
    scene.render.resolution_y = a.height
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(out_png)
    scene.view_settings.look = "Medium High Contrast"

    print("=" * 72)
    print("[CAMPUS LIVE BACKGROUND]")
    print(f"[GLB] {glb}")
    print(f"[SCENE BOUNDS] X=({mn.x:.3f}, {mx.x:.3f})")
    print(f"[SCENE BOUNDS] Y=({mn.y:.3f}, {mx.y:.3f})")
    print(f"[CAMERA CENTER] ({center.x:.3f}, {center.y:.3f})")
    print(f"[VISIBLE] W={visible_width:.3f}m H={visible_height:.3f}m")
    print(f"[PNG] {out_png}")

    bpy.ops.render.render(write_still=True)

    metadata = {
        "version": 1,
        "image_file": out_png.name,
        "image_width_px": a.width,
        "image_height_px": a.height,
        "camera_center_x_m": float(center.x),
        "camera_center_y_m": float(center.y),
        "visible_width_m": float(visible_width),
        "visible_height_m": float(visible_height),
        "scene_min_x_m": float(mn.x),
        "scene_max_x_m": float(mx.x),
        "scene_min_y_m": float(mn.y),
        "scene_max_y_m": float(mx.y),
        "padding": float(a.padding),
        "coordinate_note": "scene +X = image right, scene +Y = image up",
    }

    out_json.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print(f"[JSON] {out_json}")
    print("HOÀN TẤT.")


if __name__ == "__main__":
    main()
