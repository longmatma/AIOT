from pathlib import Path
import ast
import py_compile
import re

ROOT = Path(__file__).resolve().parent
RBS = ROOT / 'rBS_E22' / 'rbs_gateway.py'
SU = ROOT / 'SU' / 'src' / 'main.cpp'
DU = ROOT / 'DU' / 'src' / 'main.cpp'
DUR = ROOT / 'DU' / 'src' / 'nhan_lora.cpp'
DUH = ROOT / 'DU' / 'src' / 'nhan_lora.h'

for p in (RBS, SU, DU, DUR, DUH):
    assert p.exists(), f'MISSING {p}'

py_compile.compile(str(RBS), doraise=True)
rbs = RBS.read_text(encoding='utf-8', errors='replace')
su = SU.read_text(encoding='utf-8', errors='replace')
du = DU.read_text(encoding='utf-8', errors='replace')
dur = DUR.read_text(encoding='utf-8', errors='replace')
duh = DUH.read_text(encoding='utf-8', errors='replace')

checks = {
    'rbs tag': 'V2C5_2_JAM_LEASE_DISTRIBUTION_SIM' in rbs,
    'rbs rf hard off': re.search(r'^V2C5_JAM_RF_ENABLE\s*=\s*False', rbs, re.M) is not None,
    'rbs lease function': 'def _v2c52_jam_lease_mask' in rbs,
    'rbs lease bits encoded': '((int(jam_lease_mask) & 0x03) << 2)' in rbs,
    'rbs transition mask zero': 'if str(transition_phase) != "STEADY":' in rbs,
    'su prepared': '[SU JAM V2C5.2 PREPARED]' in su,
    'su armed': '[SU JAM V2C5.2 ARMED]' in su,
    'su revoke': '[SU JAM V2C5.2 REVOKE]' in su,
    'su rf off': 'SU_JAM52_RF_ENABLE = false' in su,
    'du prepared': '[DU JAM V2C5.2 PREPARED]' in dur,
    'du armed': '[DU JAM V2C5.2 ARMED]' in dur,
    'du revoke': '[DU JAM V2C5.2 REVOKE]' in dur,
    'du beacon timeout': 'DU_JAM52_BEACON_FAILSAFE_MS = 700U' in dur,
    'du rf off': 'DU_JAM52_RF_ENABLE = false' in dur,
    'du public api': 'DU_Jam52_Prepare' in duh and 'DU_Jam52_Stop' in duh,
    'no AI power hook': 'AI_POWER' not in rbs + su + du + dur,
}
for name, ok in checks.items():
    assert ok, f'FAIL static check: {name}'

# Extract and execute only the two pure rBS functions under stubs.
tree = ast.parse(rbs)
want = {'_v2c52_jam_lease_mask', '_v2c1_build_beacon_payload'}
nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in want]
assert {n.name for n in nodes} == want
mod = ast.Module(body=nodes, type_ignores=[])
ast.fix_missing_locations(mod)
ns = {
    'V2C52_LEASE_DISTRIBUTION': True,
    'V2C2_MODE_SINGLE1': 1,
    'V2C2_MODE_SINGLE2': 2,
    'V2C2_MODE_DUAL': 3,
    '_v2c1_pair_has_work': lambda p: bool(p.get('session_id') is not None and not p.get('ended', False)),
    '_v2c1_ack_tuple': lambda p: (0xFFFFFFFF, 0),
}
exec(compile(mod, '<lease-test>', 'exec'), ns)
mask = ns['_v2c52_jam_lease_mask']
build = ns['_v2c1_build_beacon_payload']

def st(p1=True, p2=True):
    return {'pairs': {
        1: {'session_id': 1 if p1 else None, 'ended': False, 'du_ready': p1},
        2: {'session_id': 2 if p2 else None, 'ended': False, 'du_ready': p2},
    }}

assert mask(st(True, False), 1, 'STEADY') == 0x1
assert mask(st(False, True), 2, 'STEADY') == 0x2
assert mask(st(True, True), 3, 'STEADY') == 0x3
assert mask(st(True, True), 3, 'PREPARE') == 0x0
assert mask(st(True, True), 3, 'COMMIT') == 0x0

payload = build(st(True, True), 7, 1, 3, 0, 3)
assert len(payload) == 15
assert payload[-1] == 0xCD, hex(payload[-1])  # MODE=3, MASK=3, GRANT=1

# Safety audit: the newly-added lease state blocks themselves must not call RF TX.
for text, a, b, label, banned in [
    (su, '// V2C5.2 - FRIENDLY-JAM LEASE DISTRIBUTION (SIM ONLY)', 'static void SU_V2B_ResetSync()', 'SU jam block', ['Phat_GoiTin_LoRa(', 'LoRa.beginPacket']),
    (dur, '// V2C5.2 - FRIENDLY-JAM LEASE DISTRIBUTION (SIM ONLY)', '// =====================================================\n// V2C4.7A', 'DU jam block', ['LoRa.beginPacket', 'LoRa.endPacket', 'setTxPower']),
]:
    i = text.index(a); j = text.index(b, i)
    block = text[i:j]
    for x in banned:
        assert x not in block, f'FAIL {label}: contains RF TX primitive {x}'

print('PASS V2C5_2_JAM_LEASE_DISTRIBUTION_SIM')
print('rBS bits3..2 carry pair lease mask; PREPARE/COMMIT=0; SU/DU PREPARED->ARMED->REVOKE; RF_JAM hard OFF; AI not integrated.')
