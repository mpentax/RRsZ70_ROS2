"""Append timestamped evidence; copy private frames before next capture.
Usage: python record.py '{"kind":"decision", ...}'
Timestamps are recording times unless an upstream timestamp is supplied.
"""
import datetime
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
record = json.loads(sys.argv[1])
record['recorded_at'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
with (ROOT / 'log.jsonl').open('a') as f:
    f.write(json.dumps(record, separators=(',', ':')) + '\n')
print(json.dumps(record))
