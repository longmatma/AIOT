from __future__ import annotations

from pathlib import Path
from datetime import datetime
import shutil
import py_compile
import subprocess

GATEWAY = Path("/home/pi5/rBS_AIOT/rbs_gateway.py")
SERVICE = Path("/etc/systemd/system/rbs-gateway.service")

if not GATEWAY.exists():
    raise SystemExit(f"[FAIL] Khong tim thay {GATEWAY}")

if not SERVICE.exists():
    raise SystemExit(f"[FAIL] Khong tim thay {SERVICE}")

t = GATEWAY.read_text(encoding="utf-8")

if "ADAPTIVE SECURITY STAGE 2" not in t:
    raise SystemExit(
        "[FAIL] rbs_gateway.py khong con marker ADAPTIVE SECURITY STAGE 2. "
        "Dung lai de tranh patch nham version."
    )

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

gw_backup = GATEWAY.with_name(
    f"rbs_gateway_backup_before_fixed_aes128_B1_{stamp}.py"
)
svc_backup = SERVICE.with_name(
    f"rbs-gateway.service.backup_before_fixed_aes128_B1_{stamp}"
)

shutil.copy2(GATEWAY, gw_backup)
shutil.copy2(SERVICE, svc_backup)

print("[BACKUP]", gw_backup)
print("[BACKUP]", svc_backup)


def reverse_once(text: str, adaptive: str, fixed: str, label: str) -> str:
    count = text.count(adaptive)
    if count != 1:
        raise RuntimeError(
            f"[STOP] {label}: can tim dung 1 block Adaptive, tim thay {count}"
        )

    print("[PATCH]", label)
    return text.replace(adaptive, fixed, 1)


fixed_anchor = '''TYPE_VOICE = 0x01
TYPE_SESSION_START = 0x02
TYPE_READY = 0x03
TYPE_AUDIO_END_SU = 0x04
TYPE_FEC = 0x05
'''

adaptive_block = '''TYPE_VOICE = 0x01
TYPE_SESSION_START = 0x02
TYPE_READY = 0x03
TYPE_AUDIO_END_SU = 0x04
TYPE_FEC = 0x05

# ============================================================
# ADAPTIVE SECURITY STAGE 2
# rBS chon profile theo SESSION.
# Stage 2: ENV/MANUAL. Stage 3: rule-based auto controller.
# ============================================================
SEC_PROFILE_NORMAL = 0
SEC_PROFILE_STRONG = 1
SEC_PROFILE_HIGH = 2

SEC_PROFILE_NAMES = {
    SEC_PROFILE_NORMAL: "NORMAL",
    SEC_PROFILE_STRONG: "STRONG",
    SEC_PROFILE_HIGH: "HIGH",
}

SEC_PROFILE_AES_BITS = {
    SEC_PROFILE_NORMAL: 128,
    SEC_PROFILE_STRONG: 256,
    SEC_PROFILE_HIGH: 256,
}

SEC_PROFILE_REKEY = {
    SEC_PROFILE_NORMAL: 256,
    SEC_PROFILE_STRONG: 128,
    SEC_PROFILE_HIGH: 64,
}


def lay_security_profile_cau_hinh():
    raw = os.environ.get(
        "RBS_SECURITY_PROFILE",
        "NORMAL",
    ).strip().upper()

    aliases = {
        "0": SEC_PROFILE_NORMAL,
        "NORMAL": SEC_PROFILE_NORMAL,
        "1": SEC_PROFILE_STRONG,
        "STRONG": SEC_PROFILE_STRONG,
        "2": SEC_PROFILE_HIGH,
        "HIGH": SEC_PROFILE_HIGH,
    }

    if raw not in aliases:
        print(
            f"[CẢNH BÁO] RBS_SECURITY_PROFILE={raw!r} khong hop le "
            "-> fallback NORMAL"
        )
        return SEC_PROFILE_NORMAL

    return aliases[raw]


def tao_session_packet_cho_du(
    goi_session_su,
    security_profile,
):
    if security_profile not in SEC_PROFILE_NAMES:
        raise ValueError(
            f"Security profile khong hop le: {security_profile}"
        )

    p = bytearray(bytes(goi_session_su))

    if (
        len(p) != SIZE_SESSION
        or p[2] != TYPE_SESSION_START
        or p[3] != 8
    ):
        raise ValueError(
            "SESSION_START SU khong dung baseline 12B/len=8"
        )

    # bits7..6 = profile; bits5..0 = payload length = 8.
    p[3] = (
        ((int(security_profile) & 0x03) << 6)
        | 8
    )

    return bytes(p)
'''

t = reverse_once(
    t,
    adaptive_block,
    fixed_anchor,
    "remove Adaptive Security constants + selector",
)

adaptive = '''def parse_session_ready_du(
    packet_bytes,
    expected_session_id,
    expected_security_profile,
):

    if packet_bytes is None:
        return None
'''

fixed = '''def parse_session_ready_du(packet_bytes, expected_session_id):

    if packet_bytes is None:
        return None
'''

t = reverse_once(
    t,
    adaptive,
    fixed,
    "READY parser signature",
)

adaptive = '''    if (
        p[0] != ID_TRAM_RBS
        or p[1] != ID_TRAM_DU
        or p[2] != TYPE_SESSION_READY
        or (p[3] & 0x03) != expected_security_profile
    ):
        return None
'''

fixed = '''    if (
        p[0] != ID_TRAM_RBS
        or p[1] != ID_TRAM_DU
        or p[2] != TYPE_SESSION_READY
    ):
        return None
'''

t = reverse_once(
    t,
    adaptive,
    fixed,
    "remove READY profile verification",
)

adaptive = '''def gui_session_ready_cho_su(
    rfm9x,
    session_id,
    security_profile,
):

    rfm9x.send(
        int(session_id).to_bytes(8, byteorder="big", signed=False),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_SESSION_READY,
        flags=int(security_profile) & 0x03,
    )

    print(
        f"[rBS GỬI] SESSION_READY -> SU | SESSION={session_id:016X} | "
        f"PROFILE={SEC_PROFILE_NAMES.get(security_profile, 'INVALID')}({security_profile})"
    )
'''

fixed = '''def gui_session_ready_cho_su(rfm9x, session_id):

    rfm9x.send(
        int(session_id).to_bytes(8, byteorder="big", signed=False),
        destination=ID_TRAM_SU,
        node=ID_TRAM_RBS,
        identifier=TYPE_SESSION_READY,
        flags=0,
    )

    print(
        f"[rBS GỬI] SESSION_READY -> SU | SESSION={session_id:016X}"
    )
'''

t = reverse_once(
    t,
    adaptive,
    fixed,
    "SESSION_READY to SU flags=0",
)

adaptive = '''def thiet_lap_session_voi_du(
    rfm9x,
    goi_session,
    session_id,
    security_profile,
    diag_session=None,
    gps_manager=None,
):
'''

fixed = '''def thiet_lap_session_voi_du(rfm9x, goi_session, session_id, diag_session=None, gps_manager=None):
'''

t = reverse_once(
    t,
    adaptive,
    fixed,
    "handshake signature",
)

adaptive = '''            sid = parse_session_ready_du(
                p,
                session_id,
                security_profile,
            )
'''

fixed = '''            sid = parse_session_ready_du(
                p,
                session_id,
            )
'''

t = reverse_once(
    t,
    adaptive,
    fixed,
    "READY verify call",
)

adaptive = '''            gui_session_ready_cho_su(
                rfm9x,
                sid,
                security_profile,
            )
'''

fixed = '''            gui_session_ready_cho_su(
                rfm9x,
                sid,
            )
'''

t = reverse_once(
    t,
    adaptive,
    fixed,
    "READY forward call",
)

adaptive = '''    t_ack_end_us = None
    diag_session = None
    diag_hoan_tat_gan_nhat = None

    session_security_profile = None

    print(
        "[BẢO MẬT] ADAPTIVE STAGE 2 BAT | "
        "CONTROLLER=rBS | SELECTOR=ENV/MANUAL | "
        f"PROFILE_NEXT={SEC_PROFILE_NAMES[lay_security_profile_cau_hinh()]}"
    )
'''

fixed = '''    t_ack_end_us = None
    diag_session = None
    diag_hoan_tat_gan_nhat = None

    print(
        "[BẢO MẬT] FIXED AES-128-GCM | SESSION KEY THEO SESSION_ID"
    )
'''

t = reverse_once(
    t,
    adaptive,
    fixed,
    "main fixed AES state",
)

adaptive = '''                    session_id_hien_tai = session_moi

                    session_security_profile = (
                        lay_security_profile_cau_hinh()
                    )

                    goi_session = (
                        tao_session_packet_cho_du(
                            info["raw"],
                            session_security_profile,
                        )
                    )

                    print(
                        f"[BẢO MẬT] SESSION={session_moi:016X} | "
                        f"PROFILE={SEC_PROFILE_NAMES[session_security_profile]}({session_security_profile}) | "
                        f"AES={SEC_PROFILE_AES_BITS[session_security_profile]} | "
                        f"REKEY_EVERY={SEC_PROFILE_REKEY[session_security_profile]} packet"
                    )

                    session_da_gui = False
'''

fixed = '''                    session_id_hien_tai = session_moi
                    goi_session = info["raw"]
                    session_da_gui = False
'''

t = reverse_once(
    t,
    adaptive,
    fixed,
    "new session raw SESSION_START",
)

adaptive = '''                else:
                    # Cung session: GIU NGUYEN profile da chon.
                    if diag_session is None:
'''

fixed = '''                else:
                    goi_session = info["raw"]

                    if diag_session is None:
'''

t = reverse_once(
    t,
    adaptive,
    fixed,
    "duplicate session raw packet",
)

adaptive = '''                    gui_session_ready_cho_su(
                        rfm9x,
                        session_id_hien_tai,
                        session_security_profile,
                    )
'''

fixed = '''                    gui_session_ready_cho_su(
                        rfm9x,
                        session_id_hien_tai,
                    )
'''

t = reverse_once(
    t,
    adaptive,
    fixed,
    "duplicate START resend READY",
)

adaptive = '''                ket_qua_bat_tay = thiet_lap_session_voi_du(
                    rfm9x,
                    goi_session,
                    session_id_hien_tai,
                    session_security_profile,
                    diag_session,
                    gps_manager,
                )
'''

fixed = '''                ket_qua_bat_tay = thiet_lap_session_voi_du(
                    rfm9x,
                    goi_session,
                    session_id_hien_tai,
                    diag_session,
                    gps_manager,
                )
'''

t = reverse_once(
    t,
    adaptive,
    fixed,
    "main handshake fixed AES",
)

GATEWAY.write_text(t, encoding="utf-8")

py_compile.compile(
    str(GATEWAY),
    doraise=True,
)

print("[CHECK] rbs_gateway.py syntax PASS")

s = SERVICE.read_text(encoding="utf-8")

lines = s.splitlines()

filtered = [
    line for line in lines
    if not line.strip().startswith(
        "Environment=RBS_SECURITY_PROFILE="
    )
]

if len(filtered) != len(lines):
    SERVICE.write_text(
        "\n".join(filtered) + "\n",
        encoding="utf-8",
    )
    print("[PATCH] service remove RBS_SECURITY_PROFILE")
else:
    print("[SKIP] service da khong con RBS_SECURITY_PROFILE")

subprocess.run(
    ["systemctl", "daemon-reload"],
    check=True,
)

print("[CHECK] systemctl daemon-reload PASS")

final = GATEWAY.read_text(encoding="utf-8")

for marker in (
    "SEC_PROFILE_NORMAL",
    "SEC_PROFILE_STRONG",
    "SEC_PROFILE_HIGH",
    "SEC_PROFILE_NAMES",
    "SEC_PROFILE_AES_BITS",
    "SEC_PROFILE_REKEY",
    "lay_security_profile_cau_hinh",
    "tao_session_packet_cho_du",
    "session_security_profile",
    "PROFILE_NEXT=",
    "REKEY_EVERY=",
):
    if marker in final:
        raise RuntimeError(
            f"[STOP] Con adaptive symbol trong rbs_gateway.py: {marker}"
        )

svc_final = SERVICE.read_text(encoding="utf-8")

if "RBS_SECURITY_PROFILE=" in svc_final:
    raise RuntimeError(
        "[STOP] Service van con RBS_SECURITY_PROFILE"
    )

print()
print("[DONE] FIXED AES-128-GCM STAGE B1 PI")
print("[CRYPTO] rBS khong con NORMAL/STRONG/HIGH")
print("[HANDSHAKE] SESSION_START/READY khong con mang profile")
print("[SERVICE] RBS_SECURITY_PROFILE da bo")
print("[IMPORTANT] Script KHONG restart service tu dong.")
print("[NEXT] Upload SU + DU B1, sau do restart rbs-gateway.service.")
