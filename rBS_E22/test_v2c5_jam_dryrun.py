from pathlib import Path
import py_compile

p = Path(__file__).with_name("rbs_gateway.py")
s = p.read_text(encoding="utf-8", errors="replace")

checks = [
    ("tag", "V2C5_JAM_DRYRUN_RBS_ONLY" in s),
    ("schedule7", "V2C1_SCHEDULE_VERSION = 7" in s),
    ("dryrun_on", 'RBS_JAM_DRYRUN' in s),
    ("hard_rf_off", "V2C5_JAM_RF_ENABLE = False" in s),
    ("jam_bits_zero", "bits3..2 CO Y GIU = 0" in s),
    ("ul_map", 'return f"DU{pi}->HO_TRO(SU{pi}->rBS)"' in s),
    ("dl_map", 'return f"SU{pi}->HO_TRO(rBS->DU{pi})"' in s),
    ("transition_block", 'OFF(CHUYEN_CHE_DO)' in s),
    ("control_block", 'OFF(CO_GOI_DIEU_KHIEN_TRONG_BURST)' in s),
    ("rf_log_off", "RF_JAM=OFF" in s),
    ("join_kept", "V2C4.4 JOIN FAILSAFE" in s),
    ("prepare_commit_kept", "MODE PREPARE" in s and "MODE COMMIT" in s),
]
failed = [name for name, ok in checks if not ok]
if failed:
    raise SystemExit("FAIL V2C5_JAM_DRYRUN: " + ", ".join(failed))

# Safety: schedule_ctl must not OR a jam_policy into bits 3..2.
if "jam_policy" in s[s.find("def _v2c1_build_beacon_payload"):s.find("def _v2c1_mark_acked")]:
    raise SystemExit("FAIL V2C5_JAM_DRYRUN: jam_policy unexpectedly entered beacon builder")

py_compile.compile(str(p), doraise=True)
print("PASS V2C5_JAM_DRYRUN_RBS_ONLY")
print("RF JAM hard-locked OFF; beacon JAM bits remain zero.")
print("Dry-run only logs candidate UL/DL helper nodes and blocks transition/control windows.")
