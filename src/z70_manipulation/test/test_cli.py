from test_executor import fake, ROOT
from z70_manipulation.cli import run, run_stack


def test_cli_dry_run_is_read_only(fake, tmp_path):
    origin, token, events, _ = fake
    result = run(['yellow', '--dry-run', '--origin', origin, '--token-file', str(token), '--repo', str(ROOT), '--log-dir', str(tmp_path / 'cli')])
    assert result['target']['color'] == 'yellow' and result['objects']
    assert (tmp_path / 'cli/front.jpg').exists()
    assert all(e[0] == 'GET' or e[1] == '/api/camera/frame' and e[2] == {} for e in events)
    stack_res = run_stack(['yellow', 'blue', 'purple', '--dry-run', '--origin', origin, '--token-file', str(token), '--repo', str(ROOT), '--log-dir', str(tmp_path / 'stack-cli')])
    assert stack_res['dry_run'] and len(stack_res['tiers']) == 3
    assert stack_res['targets']['yellow'] is not None
    assert (tmp_path / 'stack-cli/front.jpg').exists()
