from pathlib import Path
import re

p = Path(__file__).with_name('rbs_gateway.py')
s = p.read_text(encoding='utf-8', errors='replace')
checks = {
    'build tag': 'V2C5_1_JAM_DRYRUN_LOG_FIX' in s,
    'dryrun forced on': re.search(r'^V2C5_JAM_DRYRUN\s*=\s*True\s*$', s, re.M) is not None,
    'rf hard off': re.search(r'^V2C5_JAM_RF_ENABLE\s*=\s*False\b', s, re.M) is not None,
    'candidate log': '[rBS JAM DRYRUN]' in s,
    'startup forced log': 'DRYRUN CANDIDATE LOG = FORCED_ON' in s,
    'beacon jam bits comment': 'bits3..2 CO Y GIU = 0' in s,
    'schedule ctl has no jam OR term': '((int(mode) & 0x03) << 6)' in s and '| ((int(transition_target) & 0x03) << 4)' in s and '| (int(grant_pair) & 0x03)' in s,
}
failed=[k for k,v in checks.items() if not v]
if failed:
    raise SystemExit('FAIL: ' + ', '.join(failed))
print('PASS V2C5_1_JAM_DRYRUN_LOG_FIX')
print('Candidate log forced ON; RF JAM remains hard-locked OFF; beacon JAM bits remain zero.')
