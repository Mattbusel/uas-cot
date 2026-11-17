"""Convert every CoT event (one XML document per line) to TAK Protocol v1
with the snstac/takproto library and back, and check the fields survive.

usage: python tools/takproto_check.py <file> [<file> ...]
"""
import sys
import takproto
from lxml import etree

total = 0
for path in sys.argv[1:]:
    for n, line in enumerate(open(path, "rb"), 1):
        line = line.strip()
        if not line:
            continue
        total += 1
        x = etree.fromstring(line)
        for ver in (takproto.TAKProtoVer.MESH, takproto.TAKProtoVer.STREAM):
            data = takproto.xml2proto(line, ver)
            msg = takproto.parse_proto(data)
            ev = msg.cotEvent
            p = x.find("point")
            assert ev.uid == x.get("uid"), (path, n)
            assert ev.type == x.get("type"), (path, n)
            assert abs(ev.lat - float(p.get("lat"))) < 1e-9 and abs(ev.lon - float(p.get("lon"))) < 1e-9, (path, n)
            assert abs(ev.hae - float(p.get("hae"))) < 1e-9, (path, n)
            c = x.find("detail/contact")
            if c is not None:
                assert ev.detail.contact.callsign == c.get("callsign"), (path, n)
            t = x.find("detail/track")
            if t is not None:
                assert abs(ev.detail.track.course - float(t.get("course"))) < 1e-9, (path, n)
                assert abs(ev.detail.track.speed - float(t.get("speed"))) < 1e-9, (path, n)
print(f"{total} events converted to TAK Protocol v1 (mesh and stream) and checked")
sys.exit(0 if total else 1)
