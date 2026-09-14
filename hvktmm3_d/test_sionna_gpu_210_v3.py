import sys
import importlib.metadata as md

print("=" * 76)
print("SIONNA GPU 2.1.0 - REAL OPTIX/PATHSOLVER TEST V3")
print("=" * 76)
print("Python    =", sys.version.split()[0])
for pkg in ("sionna-rt", "mitsuba", "drjit"):
    try:
        print(f"{pkg:10s} =", md.version(pkg))
    except Exception as exc:
        print(f"{pkg:10s} = <khong doc duoc> {exc}")

import drjit as dr
import mitsuba as mi

print()
print("[1] CUDA backend =", bool(dr.has_backend(dr.JitBackend.CUDA)))
mi.set_variant("cuda_ad_mono_polarized")
print("[2] Mitsuba variant =", mi.variant())

if not dr.has_backend(dr.JitBackend.CUDA):
    raise RuntimeError("Dr.Jit CUDA backend khong kha dung")

# ------------------------------------------------------------
# Test A: Mitsuba/OptiX thuan
# ------------------------------------------------------------
print()
print("[3] TEST MITSUBA/OPTIX THUAN ...")

scene_mi = mi.load_dict({
    "type": "scene",
    "sphere": {
        "type": "sphere",
        "center": [0, 0, 0],
        "radius": 1.0,
    },
})

ray = mi.Ray3f(
    o=mi.Point3f(0, 0, -3),
    d=mi.Vector3f(0, 0, 1),
)
si = scene_mi.ray_intersect(ray)
dr.eval(si.t)

print("[PASS] Mitsuba/OptiX ray_intersect chay duoc.")
print("       t =", si.t)

# ------------------------------------------------------------
# Test B: Sionna RT PathSolver that su
# ------------------------------------------------------------
print()
print("[4] TEST SIONNA RT PATHSOLVER ...")

# CUDA da duoc chon truoc khi import sionna.rt
import sionna.rt
from sionna.rt import (
    load_scene,
    PlanarArray,
    Transmitter,
    Receiver,
    PathSolver,
)

scene = load_scene(
    sionna.rt.scene.simple_reflector,
    merge_shapes=True,
)

# Khong doi frequency cua scene demo.
# Muc tieu chi la test GPU PathSolver thuc su.

scene.tx_array = PlanarArray(
    num_rows=1,
    num_cols=1,
    pattern="iso",
    polarization="V",
)
scene.rx_array = scene.tx_array

tx = Transmitter(
    name="tx",
    position=[0.0, -3.0, 1.5],
)
rx = Receiver(
    name="rx",
    position=[0.0, 3.0, 1.5],
)

scene.add(tx)
scene.add(rx)
tx.look_at(rx)

solver = PathSolver()

print("       Dang chay PathSolver tren GPU...")
paths = solver(
    scene=scene,
    max_depth=2,
    samples_per_src=2000,
    los=True,
    specular_reflection=True,
    diffuse_reflection=False,
    refraction=True,
    diffraction=False,
    edge_diffraction=False,
    seed=42,
)

valid = paths.valid.numpy()
print("[PASS] Sionna PathSolver GPU chay xong.")
print("       valid shape =", valid.shape)
print("       valid paths =", int(valid.sum()))

print()
print("=" * 76)
print("GPU STACK PASS HOAN TOAN")
print("Buoc tiep theo: dua campus 433 MHz custom-material sang env GPU moi.")
print("=" * 76)
