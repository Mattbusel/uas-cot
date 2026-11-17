"""Validate CoT events (one XML document per line) against the MITRE public schemas.

usage: python tools/validate_cot.py tests/schema/cot-all.xsd <file> [<file> ...]
"""
import sys
from lxml import etree

schema = etree.XMLSchema(etree.parse(sys.argv[1]))
total = bad = 0
for path in sys.argv[2:]:
    for n, line in enumerate(open(path, "rb"), 1):
        line = line.strip()
        if not line:
            continue
        total += 1
        doc = etree.fromstring(line)
        if not schema.validate(doc):
            bad += 1
            print(f"{path}:{n}: {schema.error_log.last_error}")
print(f"{total} events, {bad} invalid")
sys.exit(1 if bad or not total else 0)
