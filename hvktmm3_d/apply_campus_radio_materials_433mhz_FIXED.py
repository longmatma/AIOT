"""
Vật liệu RF cho khuôn viên trường tại 433 MHz.

LÝ DO FILE NÀY DÙNG RadioMaterial THAY VÌ ITURadioMaterial:
Sionna RT kiểm tra dải tần của ITURadioMaterial. Ở 433 MHz (0.433 GHz),
concrete / medium_dry_ground / metal nằm ngoài dải ITU danh định và
Sionna sẽ báo:
    ValueError: Properties of ITU material 'concrete' are not defined
                for this frequency

Vì vậy file này dùng RadioMaterial với epsilon_r và conductivity được
tính cố định tại 433 MHz.

Công thức ITU-R P.2040:
    epsilon_r = a * f_GHz**b
    sigma     = c * f_GHz**d

Các giá trị concrete và medium_dry_ground ở 433 MHz là NGOẠI SUY ngoài
dải danh định ITU; dùng làm baseline mô phỏng ban đầu, không phải số đo
thực địa.

Object mapping:
    Buildings*       -> mat-building
    Boundary_Walls*  -> mat-boundary-wall
    Ground_Aerial    -> mat-ground
    Roads_Paved*     -> mat-road
    Tree_Trunks*     -> mat-tree-trunk
    Tree_Canopies*   -> mat-tree-canopy
    ATM_Kiosk        -> mat-atm-metal
    Sports_Courts*   -> mat-sports-court
"""

from sionna.rt import RadioMaterial

FREQUENCY_HZ = 433e6

# Giá trị điện môi tại 433 MHz.
# concrete:
#   eps_r = 5.24
#   sigma = 0.0462 * 0.433**0.7822 = ~0.024005 S/m
# wood:
#   eps_r = 1.99
#   sigma = 0.0047 * 0.433**1.0718 = ~0.001916 S/m
# medium_dry_ground:
#   eps_r = 15 * 0.433**(-0.1) = ~16.3096
#   sigma = 0.035 * 0.433**1.63 = ~0.008944 S/m
# metal:
#   eps_r = 1
#   sigma = 1e7 S/m (proxy)

MATERIALS = {
    "mat-building": dict(
        relative_permittivity=5.24,
        conductivity=0.0240051,
        thickness=0.20,
        scattering_coefficient=0.15,
        xpd_coefficient=0.00,
    ),
    "mat-boundary-wall": dict(
        relative_permittivity=5.24,
        conductivity=0.0240051,
        thickness=0.15,
        scattering_coefficient=0.20,
        xpd_coefficient=0.00,
    ),
    "mat-ground": dict(
        relative_permittivity=16.3096,
        conductivity=0.00894424,
        thickness=0.30,
        scattering_coefficient=0.35,
        xpd_coefficient=0.00,
    ),
    # Đường lát/asphalt: dùng concrete proxy ở bước đầu
    "mat-road": dict(
        relative_permittivity=5.24,
        conductivity=0.0240051,
        thickness=0.10,
        scattering_coefficient=0.30,
        xpd_coefficient=0.00,
    ),
    "mat-tree-trunk": dict(
        relative_permittivity=1.99,
        conductivity=0.00191640,
        thickness=0.15,
        scattering_coefficient=0.25,
        xpd_coefficient=0.00,
    ),
    # Tán cây là proxy bề mặt, chưa phải mô hình môi trường thể tích
    "mat-tree-canopy": dict(
        relative_permittivity=1.99,
        conductivity=0.00191640,
        thickness=0.50,
        scattering_coefficient=0.60,
        xpd_coefficient=0.10,
    ),
    "mat-atm-metal": dict(
        relative_permittivity=1.0,
        conductivity=1.0e7,
        thickness=0.005,
        scattering_coefficient=0.05,
        xpd_coefficient=0.00,
    ),
    "mat-sports-court": dict(
        relative_permittivity=5.24,
        conductivity=0.0240051,
        thickness=0.10,
        scattering_coefficient=0.25,
        xpd_coefficient=0.00,
    ),
}


def classify(name: str):
    if name.startswith("Buildings"):
        return "mat-building"
    if name.startswith("Boundary_Walls"):
        return "mat-boundary-wall"
    if name.startswith("Ground_Aerial"):
        return "mat-ground"
    if name.startswith("Roads_Paved"):
        return "mat-road"
    if name.startswith("Tree_Trunks"):
        return "mat-tree-trunk"
    if name.startswith("Tree_Canopies"):
        return "mat-tree-canopy"
    if name.startswith("ATM_Kiosk"):
        return "mat-atm-metal"
    if name.startswith("Sports_Courts"):
        return "mat-sports-court"
    return None


def apply_campus_materials(scene, verbose=True):
    scene.frequency = FREQUENCY_HZ

    for mat_name, kwargs in MATERIALS.items():
        if mat_name not in scene.radio_materials:
            scene.add(RadioMaterial(name=mat_name, **kwargs))

    assigned = 0
    skipped = []

    for obj_name, obj in scene.objects.items():
        mat_name = classify(obj_name)
        if mat_name is None:
            skipped.append(obj_name)
            continue

        obj.radio_material = mat_name
        assigned += 1

        if verbose:
            print(f"[MATERIAL] {obj_name} -> {mat_name}")

    if verbose:
        print(f"[MATERIAL] scene.frequency = {scene.frequency/1e6:.3f} MHz")
        print(f"[MATERIAL] assigned objects = {assigned}")
        if skipped:
            print("[MATERIAL] skipped:", ", ".join(skipped))

    return assigned, skipped
