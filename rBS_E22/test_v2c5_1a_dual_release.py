from pathlib import Path
import py_compile
import re

p = Path(__file__).with_name("rbs_gateway.py")
s = p.read_text(encoding="utf-8", errors="replace")

checks = {
    "tag": "V2C5_1A_DUAL_RELEASE_FIX" in s,
    "fast release constant": re.search(r"^V2C5_DUAL_FAST_RELEASE_S\s*=\s*0\.95\s*$", s, re.M) is not None,
    "hard release constant": re.search(r"^V2C5_VOICE_HARD_RELEASE_S\s*=\s*2\.20\s*$", s, re.M) is not None,
    "last voice timestamp": '"last_voice_rx_at": None' in s,
    "voice lease hook": "_v2c5_expire_stale_voice_sessions(st)" in s,
    "fast reason": "DUAL_OTHER_PAIR_STILL_ACTIVE" in s,
    "hard reason": "LOST_AUDIO_END_FAILSAFE" in s,
    "stale end control": "STALE END_AUDIO -> DU" in s,
    "prepare commit kept": "MODE PREPARE" in s and "MODE COMMIT" in s,
    "RF hard off": re.search(r"^V2C5_JAM_RF_ENABLE\s*=\s*False\b", s, re.M) is not None,
    "jam bits remain zero": "bits3..2 CO Y GIU = 0" in s,
}
failed = [k for k, v in checks.items() if not v]
if failed:
    raise SystemExit("FAIL V2C5_1A: " + ", ".join(failed))

# Guard against accidentally wiring jam policy into the beacon.
start = s.find("def _v2c1_build_beacon_payload")
end = s.find("def _v2c1_mark_acked", start)
if start < 0 or end < 0:
    raise SystemExit("FAIL V2C5_1A: beacon builder not found")
if "jam_policy" in s[start:end]:
    raise SystemExit("FAIL V2C5_1A: jam policy entered beacon builder")

py_compile.compile(str(p), doraise=True)
print("PASS V2C5_1A_DUAL_RELEASE_FIX")
print("DUAL stale voice release added; PREPARE/COMMIT unchanged; RF JAM remains hard OFF.")
