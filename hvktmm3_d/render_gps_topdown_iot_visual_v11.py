# -*- coding: utf-8 -*-
"""
Render GPS top-down for campus GLB - V11 trusted-synchronized GPS visualization.

Mục tiêu:
- Giữ nguyên logic GPS đã đúng: campus_geo_config.py là anchor, CSV chọn cặp SU/DU gần thời gian nhất.
- Làm phần hiển thị dễ đọc hơn nhiều:
  + marker rBS / SU / DU lớn và rõ
  + có cột mast cắm xuống map
  + label ngắn gọn trên card nền đậm
  + giữ khoảng cách link nhưng đặt gọn giữa đường nối
  + bỏ title / trục / chữ thừa

Run:
& "E:\\blender\\blender.exe" --background --python ".\\render_gps_topdown_iot_visual_v11.py" -- --glb ".\\campus_chinh_chieu_cao.glb" --gps-csv ".\\rbs_lien_ket_dinh_ky_v11.csv"
"""

import bpy
import sys
import csv
import math
import argparse
import importlib.util
from bisect import bisect_left
from pathlib import Path
from datetime import datetime
from mathutils import Vector

EARTH_R = 6_371_000.0


def parse_args():
    argv = sys.argv
    argv = argv[argv.index("--") + 1:] if "--" in argv else []
    p = argparse.ArgumentParser()
    p.add_argument("--glb", default="campus_chinh_chieu_cao.glb")
    p.add_argument("--gps-csv", required=True)
    p.add_argument("--width", type=int, default=2400)
    p.add_argument("--height", type=int, default=1700)
    p.add_argument("--padding", type=float, default=1.18)
    p.add_argument("--max-pair-delta-sec", type=float, default=10.0)
    p.add_argument("--overlay-height", type=float, default=6.0,
                   help="Độ cao lớp hiển thị SU/rBS/DU so với vật thể cao nhất của campus (m)")
    return p.parse_args(argv)


def load_geo_config():
    cfg_path = Path(__file__).resolve().parent / "campus_geo_config.py"
    if not cfg_path.exists():
        raise FileNotFoundError(f"Không tìm thấy campus_geo_config.py: {cfg_path}")
    spec = importlib.util.spec_from_file_location("campus_geo_config", str(cfg_path))
    if spec is None or spec.loader is None:
        raise ImportError(f"Không nạp được campus_geo_config.py từ {cfg_path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)


def object_world_bounds(obj):
    pts = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    mn = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    mx = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    return mn, mx


def scene_bounds(objects, mesh_only=True):
    mins, maxs = [], []
    for obj in objects:
        if mesh_only and obj.type != "MESH":
            continue
        if obj.type not in ("MESH", "CURVE", "FONT"):
            continue
        mn, mx = object_world_bounds(obj)
        mins.append(mn)
        maxs.append(mx)
    if not mins:
        raise RuntimeError("Không tìm thấy object phù hợp trong scene.")
    mn = Vector((min(v.x for v in mins), min(v.y for v in mins), min(v.z for v in mins)))
    mx = Vector((max(v.x for v in maxs), max(v.y for v in maxs), max(v.z for v in maxs)))
    return mn, mx


def find_ground_z():
    for obj in bpy.context.scene.objects:
        if obj.type == "MESH" and obj.name.startswith("Ground_Aerial"):
            mn, mx = object_world_bounds(obj)
            return (mn.z + mx.z) * 0.5
    mn, _ = scene_bounds(list(bpy.context.scene.objects), mesh_only=True)
    return mn.z


def make_material(name, rgba, metallic=0.0, roughness=0.35, emission_rgb=None, emission_strength=0.0, alpha=1.0):
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = (rgba[0], rgba[1], rgba[2], alpha)
        bsdf.inputs["Metallic"].default_value = metallic
        bsdf.inputs["Roughness"].default_value = roughness
        if "Alpha" in bsdf.inputs:
            bsdf.inputs["Alpha"].default_value = alpha
        if emission_rgb is not None and "Emission" in bsdf.inputs:
            bsdf.inputs["Emission"].default_value = (*emission_rgb, 1.0)
            if "Emission Strength" in bsdf.inputs:
                bsdf.inputs["Emission Strength"].default_value = emission_strength
    mat.blend_method = 'BLEND' if alpha < 0.999 else 'OPAQUE'
    mat.shadow_method = 'HASHED' if alpha < 0.999 else 'OPAQUE'
    return mat


def add_material(obj, mat):
    if obj.data and hasattr(obj.data, "materials"):
        if len(obj.data.materials) == 0:
            obj.data.materials.append(mat)
        else:
            obj.data.materials[0] = mat


def add_cylinder_between(name, p0, p1, radius, material, vertices=24):
    p0 = Vector(p0)
    p1 = Vector(p1)
    vec = p1 - p0
    if vec.length <= 1e-9:
        return None
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=vec.length, location=(p0 + p1) * 0.5)
    obj = bpy.context.active_object
    obj.name = name
    obj.rotation_mode = "QUATERNION"
    obj.rotation_quaternion = vec.to_track_quat("Z", "Y")
    add_material(obj, material)
    return obj


def add_box(name, pos, dims, material):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=pos)
    obj = bpy.context.active_object
    obj.name = name
    obj.dimensions = dims
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    add_material(obj, material)
    return obj


def add_sphere(name, pos, radius, material):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=48, ring_count=24, radius=radius, location=pos)
    obj = bpy.context.active_object
    obj.name = name
    add_material(obj, material)
    return obj


def add_torus(name, pos, major_radius, minor_radius, material):
    bpy.ops.mesh.primitive_torus_add(major_segments=56, minor_segments=16, major_radius=major_radius, minor_radius=minor_radius, location=pos)
    obj = bpy.context.active_object
    obj.name = name
    add_material(obj, material)
    return obj


def add_plane(name, pos, size_x, size_y, material):
    bpy.ops.mesh.primitive_plane_add(size=1.0, location=pos)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = (size_x * 0.5, size_y * 0.5, 1.0)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    add_material(obj, material)
    return obj


def add_text_flat(name, text, pos, material, size=1.0, align='CENTER'):
    bpy.ops.object.text_add(location=pos)
    obj = bpy.context.active_object
    obj.name = name
    obj.data.body = text
    obj.data.align_x = align
    obj.data.align_y = "CENTER"
    obj.data.size = size
    obj.data.extrude = 0.02
    obj.data.bevel_depth = 0.004
    add_material(obj, material)
    # Keep text readable from strict top-down camera.
    obj.rotation_euler = (0.0, 0.0, 0.0)
    return obj


def add_label_card(name, lines, anchor_xy, offset_xy, ground_z, text_mat, bg_mat, card_size=(9.5, 4.0), text_size=1.0):
    x = anchor_xy[0] + offset_xy[0]
    y = anchor_xy[1] + offset_xy[1]
    card_z = ground_z + 0.06
    text_z = ground_z + 0.09
    add_plane(f"{name}_CARD", (x, y, card_z), card_size[0], card_size[1], bg_mat)
    body = "\n".join(lines)
    # Slight X offset compensates default text origin to look centered on card.
    add_text_flat(f"{name}_TEXT", body, (x - (card_size[0] * 0.16), y - 0.25, text_z), text_mat, size=text_size, align='LEFT')
    return Vector((x, y, text_z))


def make_node(node_name, x, y, mast_height, ground_z, color_mat, dark_mat, ring_mat=None):
    # Stronger top-down visibility: ground pad + mast + top beacon.
    add_cylinder_between(f"{node_name}_PAD", (x, y, ground_z + 0.01), (x, y, ground_z + 0.14), 1.10, dark_mat, vertices=48)
    if ring_mat is None:
        ring_mat = color_mat
    add_torus(f"{node_name}_GROUND_RING", (x, y, ground_z + 0.18), 1.55, 0.12, ring_mat)
    add_cylinder_between(f"{node_name}_MAST", (x, y, ground_z + 0.14), (x, y, ground_z + mast_height), 0.18, dark_mat, vertices=32)
    top = Vector((x, y, ground_z + mast_height))
    add_sphere(f"{node_name}_BEACON", top, 0.95, color_mat)
    add_torus(f"{node_name}_TOP_RING", top, 1.45, 0.09, ring_mat)
    return top


def setup_camera_from_campus_bounds(mn, mx, width, height, padding):
    center = (mn + mx) * 0.5
    sx = mx.x - mn.x
    sy = mx.y - mn.y

    bpy.ops.object.camera_add()
    cam = bpy.context.active_object
    cam.name = "AUTO_CAMERA_TOPDOWN"
    cam.data.type = "ORTHO"

    # Camera nhìn thẳng từ trên xuống toàn bộ campus
    cam.location = (
        center.x,
        center.y,
        mx.z + max(sx, sy) + 80.0
    )

    cam.rotation_euler = (0.0, 0.0, 0.0)

    # Tính ORTHO scale theo cả chiều ngang và chiều dọc
    # để không bị crop mép campus
    aspect = width / height

    ortho_width = sx * padding
    ortho_height_convert = sy * padding * aspect

    cam.data.ortho_scale = max(
        ortho_width,
        ortho_height_convert
    )

    cam.data.clip_start = 0.1
    cam.data.clip_end = 10000.0

    bpy.context.scene.camera = cam
    return cam


def setup_lighting(center, max_z):
    bpy.ops.object.light_add(type="SUN", location=(center.x, center.y, max_z + 100.0))
    sun = bpy.context.active_object
    sun.data.energy = 2.0
    sun.rotation_euler = (math.radians(28.0), math.radians(-15.0), math.radians(-18.0))
    world = bpy.context.scene.world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs["Color"].default_value = (0.17, 0.17, 0.17, 1.0)
        bg.inputs["Strength"].default_value = 0.82


def safe_float(v, default=None):
    try:
        return float(v)
    except Exception:
        return default


def safe_int(v, default=None):
    try:
        return int(float(v))
    except Exception:
        return default


def parse_time_iso(s):
    try:
        return datetime.fromisoformat((s or "").strip())
    except Exception:
        return None


def valid_coordinate(lat, lon):
    return lat is not None and lon is not None and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0 and not (lat == 0.0 and lon == 0.0)


def load_latest_sync_pair(csv_path, max_pair_delta_sec=10.0):
    """
    Chọn cặp SU/DU MỚI NHẤT có timestamp gần nhau trong ngưỡng.
    Không ưu tiên một cặp rất cũ chỉ vì delta của nó nhỏ hơn vài mili-giây.
    """
    csv_path = Path(csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(f"Không tìm thấy GPS CSV: {csv_path}")

    su_rows, du_rows = [], []

    with csv_path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)

        if not reader.fieldnames:
            raise RuntimeError("CSV GPS không có header.")

        required = {
            "Thiet_bi",
            "GPS_hop_le",
            "GPS_tin_cay_2D",
            "Vi_do_tho",
            "Kinh_do_tho",
            "Thoi_gian_rBS_nhan_UTC",
        }
        missing = required - set(reader.fieldnames)

        if missing:
            raise RuntimeError(
                "CSV GPS thiếu cột: " + ", ".join(sorted(missing))
            )

        for row in reader:
            device = (row.get("Thiet_bi") or "").strip().upper()

            if device not in ("SU", "DU"):
                continue

            # Chỉ dùng GPS đủ tin cậy để đưa lên Digital Twin:
            # GPS_hop_le=1 mới chỉ nói rằng có tọa độ;
            # GPS_tin_cay_2D=1 mới đạt bộ lọc chất lượng của hệ thống.
            if (row.get("GPS_hop_le") or "").strip() != "1":
                continue
            if (row.get("GPS_tin_cay_2D") or "").strip() != "1":
                continue

            lat = safe_float(row.get("Vi_do_tho"), None)
            lon = safe_float(row.get("Kinh_do_tho"), None)
            ts = parse_time_iso(row.get("Thoi_gian_rBS_nhan_UTC"))

            if not valid_coordinate(lat, lon) or ts is None:
                continue

            item = {
                "lat": lat,
                "lon": lon,
                "alt": safe_float(row.get("Do_cao_tho_m"), 0.0),
                "hdop": safe_float(row.get("HDOP"), None),
                "sat": safe_int(row.get("So_ve_tinh"), None),
                "time": row.get("Thoi_gian_rBS_nhan_UTC", ""),
                "time_obj": ts,
                "distance_m": safe_float(row.get("Khoang_cach_rBS_node_m"), None),
                "seq": safe_int(row.get("So_thu_tu_bao_cao"), None),
            }

            (su_rows if device == "SU" else du_rows).append(item)

    if not su_rows and not du_rows:
        raise RuntimeError("Không tìm thấy GPS hợp lệ cho cả SU và DU trong CSV.")

    su_rows.sort(key=lambda x: x["time_obj"])
    du_rows.sort(key=lambda x: x["time_obj"])

    if not su_rows:
        return None, du_rows[-1], None

    if not du_rows:
        return su_rows[-1], None, None

    su_times = [x["time_obj"] for x in su_rows]
    candidates = []

    for du in du_rows:
        idx = bisect_left(su_times, du["time_obj"])

        for j in (idx - 1, idx):
            if j < 0 or j >= len(su_rows):
                continue

            su = su_rows[j]
            delta = abs(
                (su["time_obj"] - du["time_obj"]).total_seconds()
            )

            if delta <= max_pair_delta_sec:
                latest_time = max(su["time_obj"], du["time_obj"])
                candidates.append((latest_time, -delta, su, du, delta))

    if not candidates:
        raise RuntimeError(
            "Không tìm được cặp SU/DU đồng bộ trong "
            f"{max_pair_delta_sec:.1f} giây."
        )

    candidates.sort(
        key=lambda item: (item[0], item[1]),
        reverse=True,
    )

    _, _, su, du, delta = candidates[0]
    return su, du, delta


def gps_to_local_en(lat, lon, lat0, lon0):
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)
    east = EARTH_R * (lon_rad - lon0_rad) * math.cos(lat0_rad)
    north = EARTH_R * (lat_rad - lat0_rad)
    return east, north


def en_to_scene_xy(east_m, north_m, cfg):
    if getattr(cfg, "FLIP_NORTH", False):
        north_m = -north_m
    theta = math.radians(float(getattr(cfg, "MAP_ROTATION_DEG", 0.0)))
    scene_dx = math.cos(theta) * east_m - math.sin(theta) * north_m
    scene_dy = math.sin(theta) * east_m + math.cos(theta) * north_m
    x = float(getattr(cfg, "RBS_SCENE_X_M", 0.0)) + scene_dx
    y = float(getattr(cfg, "RBS_SCENE_Y_M", 0.0)) + scene_dy
    return x, y


def gps_to_scene(lat, lon, cfg):
    east, north = gps_to_local_en(lat, lon, float(getattr(cfg, "RBS_LAT")), float(getattr(cfg, "RBS_LON")))
    x, y = en_to_scene_xy(east, north, cfg)
    return x, y, east, north


def point_in_bounds(x, y, mn, mx, margin=0.0):
    return (mn.x - margin) <= x <= (mx.x + margin) and (mn.y - margin) <= y <= (mx.y + margin)


def format_xy(x, y):
    return f"({x:.1f},{y:.1f})m"


def add_distance_card(name, p0, p1, text, ground_z, text_mat, bg_mat):
    p0 = Vector(p0)
    p1 = Vector(p1)
    mid = (p0 + p1) * 0.5
    vec = p1 - p0
    if vec.length < 1e-6:
        offset = Vector((0.0, 2.4, 0.0))
    else:
        perp = Vector((-vec.y, vec.x, 0.0))
        perp.normalize()
        offset = perp * 2.4
    card_pos = mid + offset
    add_plane(f"{name}_CARD", (card_pos.x, card_pos.y, ground_z + 0.05), 5.7, 1.9, bg_mat)
    add_text_flat(f"{name}_TEXT", text, (card_pos.x - 1.6, card_pos.y - 0.18, ground_z + 0.08), text_mat, size=0.82, align='LEFT')


def main():
    a = parse_args()
    cfg = load_geo_config()

    glb = Path(a.glb).resolve()
    gps_csv = Path(a.gps_csv).resolve()
    if not glb.exists():
        raise FileNotFoundError(f"Không tìm thấy GLB: {glb}")

    su, du, pair_delta_sec = load_latest_sync_pair(gps_csv, a.max_pair_delta_sec)

    preview_dir = glb.parent / "preview"
    preview_dir.mkdir(parents=True, exist_ok=True)
    out_png = preview_dir / f"gps_topdown_iot_visual_v11_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"

    print("=" * 72)
    print("[INFO] GEO ANCHOR FROM campus_geo_config.py ONLY")
    print(f"[CFG] RBS_LAT={float(getattr(cfg, 'RBS_LAT')):.7f}, RBS_LON={float(getattr(cfg, 'RBS_LON')):.7f}")
    print(f"[GPS CSV] {gps_csv}")
    print(f"[SU SNAPSHOT] {su}")
    print(f"[DU SNAPSHOT] {du}")
    if pair_delta_sec is not None:
        print(f"[PAIR DELTA] {pair_delta_sec:.3f} sec")
        if pair_delta_sec > a.max_pair_delta_sec:
            print(f"[WARNING] Cặp SU/DU gần nhất vẫn lệch {pair_delta_sec:.2f}s > ngưỡng {a.max_pair_delta_sec:.2f}s")

    clear_scene()
    bpy.ops.import_scene.gltf(filepath=str(glb))
    imported = list(bpy.context.scene.objects)
    campus_mn, campus_mx = scene_bounds(imported, mesh_only=True)
    ground_z = find_ground_z()
    campus_center = (campus_mn + campus_mx) * 0.5

    # ========================================================
    # LỚP HIỂN THỊ NỔI
    # ========================================================
    # SU / rBS / DU vẫn giữ nguyên X/Y thật.
    # Chỉ phần HIỂN THỊ được đưa lên cao hơn tòa nhà cao nhất
    # để marker, đường nối và nhãn không bị nhà/cây che khuất.
    # Việc này KHÔNG làm thay đổi tọa độ dùng cho Sionna.
    visual_base_z = campus_mx.z + float(a.overlay_height)

    print(f"[CAMPUS] Z max = {campus_mx.z:.3f} m")
    print(f"[VISUAL OVERLAY] base Z = {visual_base_z:.3f} m")

    cam = setup_camera_from_campus_bounds(campus_mn, campus_mx, a.width, a.height, a.padding)
    setup_lighting(campus_center, campus_mx.z)

    mat_su = make_material("MAT_SU", (0.11, 0.88, 0.26), roughness=0.20, emission_rgb=(0.05, 0.38, 0.08), emission_strength=2.0)
    mat_rbs = make_material("MAT_RBS", (1.00, 0.78, 0.07), roughness=0.18, emission_rgb=(0.55, 0.34, 0.02), emission_strength=2.0)
    mat_du = make_material("MAT_DU", (0.22, 0.52, 1.00), roughness=0.20, emission_rgb=(0.05, 0.10, 0.45), emission_strength=2.0)
    mat_link = make_material("MAT_LINK", (0.96, 0.90, 0.18), roughness=0.30, emission_rgb=(0.25, 0.22, 0.02), emission_strength=1.2)
    mat_dark = make_material("MAT_DARK", (0.06, 0.07, 0.08), metallic=0.55, roughness=0.30)
    mat_card = make_material("MAT_CARD", (0.09, 0.10, 0.12), roughness=0.45, alpha=0.96)
    mat_text_white = make_material("MAT_TEXT_WHITE", (0.98, 0.98, 0.98), roughness=0.35)
    mat_text_su = make_material("MAT_TEXT_SU", (0.80, 1.00, 0.84), roughness=0.35)
    mat_text_rbs = make_material("MAT_TEXT_RBS", (1.00, 0.93, 0.45), roughness=0.35)
    mat_text_du = make_material("MAT_TEXT_DU", (0.78, 0.88, 1.00), roughness=0.35)

    nodes = {}
    # rBS from scene anchor
    rbs_x = float(getattr(cfg, "RBS_SCENE_X_M", 0.0))
    rbs_y = float(getattr(cfg, "RBS_SCENE_Y_M", 0.0))
    rbs_h = float(getattr(cfg, "RBS_HEIGHT_M", 2.0))  # tọa độ vật lý; marker V10 dùng lớp hiển thị nổi
    nodes["RBS"] = make_node("RBS", rbs_x, rbs_y, 2.2, visual_base_z, mat_rbs, mat_dark)
    add_label_card("RBS_LABEL", ["rBS", format_xy(rbs_x, rbs_y)], (rbs_x, rbs_y), (0.0, 6.0), visual_base_z, mat_text_rbs, mat_card, card_size=(7.0, 3.3), text_size=0.95)

    su_xy = None
    if su is not None:
        su_x, su_y, su_e, su_n = gps_to_scene(su["lat"], su["lon"], cfg)
        print(f"[SU SCENE] X={su_x:.3f}, Y={su_y:.3f}, E={su_e:.3f}, N={su_n:.3f}, T={su['time']}")
        if not point_in_bounds(su_x, su_y, campus_mn, campus_mx, margin=5.0):
            print("[WARNING] SU nằm ngoài bounds campus. Kiểm tra campus_geo_config.py")
        su_xy = (su_x, su_y)
        su_h = float(getattr(cfg, "SU_HEIGHT_M", 2.0))  # tọa độ vật lý; marker V10 dùng lớp hiển thị nổi
        nodes["SU"] = make_node("SU", su_x, su_y, 2.2, visual_base_z, mat_su, mat_dark)
        add_label_card("SU_LABEL", ["SU", format_xy(su_x, su_y)], su_xy, (-8.5, -5.0), visual_base_z, mat_text_su, mat_card, card_size=(7.0, 3.3), text_size=0.95)

    du_xy = None
    if du is not None:
        du_x, du_y, du_e, du_n = gps_to_scene(du["lat"], du["lon"], cfg)
        print(f"[DU SCENE] X={du_x:.3f}, Y={du_y:.3f}, E={du_e:.3f}, N={du_n:.3f}, T={du['time']}")
        if not point_in_bounds(du_x, du_y, campus_mn, campus_mx, margin=5.0):
            print("[WARNING] DU nằm ngoài bounds campus. Kiểm tra campus_geo_config.py")
        du_xy = (du_x, du_y)
        du_h = float(getattr(cfg, "DU_HEIGHT_M", 2.0))  # tọa độ vật lý; marker V10 dùng lớp hiển thị nổi
        nodes["DU"] = make_node("DU", du_x, du_y, 2.2, visual_base_z, mat_du, mat_dark)
        add_label_card("DU_LABEL", ["DU", format_xy(du_x, du_y)], du_xy, (8.5, 5.0), visual_base_z, mat_text_du, mat_card, card_size=(7.0, 3.3), text_size=0.95)

    if "SU" in nodes:
        add_cylinder_between("LINK_SU_RBS", nodes["SU"], nodes["RBS"], 0.12, mat_link, vertices=24)
        d = (nodes["SU"] - nodes["RBS"]).length
        add_distance_card("DIST_SU_RBS", nodes["SU"], nodes["RBS"], f"SU-rBS {d:.0f} m", visual_base_z, mat_text_white, mat_card)
    if "DU" in nodes:
        add_cylinder_between("LINK_RBS_DU", nodes["RBS"], nodes["DU"], 0.12, mat_link, vertices=24)
        d = (nodes["DU"] - nodes["RBS"]).length
        add_distance_card("DIST_RBS_DU", nodes["RBS"], nodes["DU"], f"rBS-DU {d:.0f} m", visual_base_z, mat_text_white, mat_card)

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

    print(f"[PNG] {out_png}")
    bpy.ops.render.render(write_still=True)
    print("=" * 72)
    print(f"RENDER HOÀN TẤT: {out_png}")
    print("=" * 72)


if __name__ == "__main__":
    main()
