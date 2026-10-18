"""Run all offline tests with legacy archive-writing tests redirected to a copy.

Run from repo root using calibration/arm-validation/.venv/bin/python THIS_FILE.
No network, robot, camera session, Docker or service access is performed.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
MODULES = ROOT/'calibration/arm-calibration/physical_base'
SESSION = ROOT/'calibration/arm-calibration/20261015T061415Z-physical-base'
sys.path.insert(0, str(MODULES))
import survey_refit
import test_analysis
import test_validation

before = survey_refit.file_hashes(p for p in SESSION.rglob('*') if p.is_file())
expected = json.loads((HERE/'archive-before-sha256.json').read_text())
assert before == expected, 'original archive no longer matches pre-refit snapshot'
with tempfile.TemporaryDirectory(dir=os.environ.get('TMPDIR')) as td:
    copied = Path(td)/'private-copy'
    shutil.copytree(SESSION, copied)
    test_analysis.SESSION = copied
    test_validation.D = copied
    suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n)
        for n in ['test_survey_refit', 'test_geometry', 'test_observations', 'test_analysis', 'test_validation'])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
after = survey_refit.file_hashes(p for p in SESSION.rglob('*') if p.is_file())
assert before == after, 'tests changed the original archive'
comparison = json.loads((HERE/'comparison.json').read_text())
verification = {'tests_run': result.testsRun, 'failures': len(result.failures),
    'errors': len(result.errors), 'archive_files_preserved': len(before),
    'raw_manifest_verified': comparison['raw_manifest_verified_count'],
    'scenario_count': len(comparison['scenarios']),
    'archive_matches_pre_refit_snapshot': before == expected,
    'unsafe_legacy_integration_tests_redirected_to_disposable_copy': True,
    'source_code_sha256': survey_refit.file_hashes([MODULES/'survey_refit.py', MODULES/'test_survey_refit.py']),
    'no_production_edits_or_live_access': True}
(HERE/'verification.json').write_text(json.dumps(verification, indent=2))
print(json.dumps(verification, indent=2))
sys.exit(0 if result.wasSuccessful() else 1)
