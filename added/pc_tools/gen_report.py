#!/usr/bin/env python3
import json, base64, os, html
D = r"C:\Users\Niko\AppData\Local\Temp\claude\D--robot\3dcf8889-30c3-4f0b-bf9d-3dea1bba7671\scratchpad\patrol"

def b64(p):
    with open(p,"rb") as f: return base64.b64encode(f.read()).decode()

rows = []
for fn in ("report.json","report2.json"):
    fp = os.path.join(D, fn)
    if os.path.exists(fp):
        rows += json.load(open(fp, encoding="utf-8"))

# label spots
def spot_label(r):
    if "target" in r:
        t = r["target"]; return "Stop A/B (drum-kit & desk corner)"
    return "Traverse pocket"

cards = []
for r in rows:
    img = os.path.join(D, r["img"])
    if not os.path.exists(img): continue
    desc = html.escape(r["desc"])
    lines = "".join("<li>%s</li>" % html.escape(x.strip(" -")) for x in r["desc"].split("\n") if x.strip(" -"))
    pose = r.get("pose")
    posetxt = ("map (%.2f, %.2f, %s°)" % (pose[0], pose[1], pose[2])) if pose else "—"
    drive = r.get("drive","")
    badge = {"SUCCEEDED":"#1a7f37","ABORTED":"#b35900","TIMEOUT":"#b35900"}.get(drive,"#555")
    cards.append(f"""
    <div class="card">
      <img src="data:image/jpeg;base64,{b64(img)}"/>
      <div class="meta">
        <span class="pan">{html.escape(r['pan']).upper()}</span>
        <span class="badge" style="background:{badge}">{html.escape(drive)}</span>
        <span class="pose">{posetxt}</span>
      </div>
      <ul>{lines}</ul>
    </div>""")

tourmap = b64(os.path.join(D,"tourmap.png"))

POI = """
<b>Music / audio corner</b>: electronic drum kit (snare, kick, hi-hat & cymbal pads, chrome frame, black mat), foot pedals (DW / Tama / Zoom / Roland branding seen), drumsticks; a black upright / digital piano with dark fabric draped over it; a piano bench (“König &amp; Meyer, Germany”); a tall chrome pole (likely a mic/boom stand).<br>
<b>Desk / computer area</b>: black office chair(s) on casters; glass-top desk with thin metal hairpin legs; white shelving unit with books (one spine reads “CHINA”, a “100 SANAS” box), papers & electronics; a large wood-slatted tower (PC case or speaker); a purple/black PC-case box; a white cube device (router/modem or audio interface); a monitor.<br>
<b>Window / storage side</b>: window with heavy layered curtains and strong daylight; a metal drying rack with clothes (green/grey/black/red); storage boxes; coats/jackets on a rack; a red &amp; silver tool case; a black fan; a green bottle.<br>
<b>Everywhere</b>: lots of tangled cables, chargers and white power strips on the floor; multiple wall outlets; a closed white door; wooden (parquet) flooring throughout.
"""

htmlout = f"""<!doctype html><html><head><meta charset="utf-8">
<style>
 body{{font-family:system-ui,Segoe UI,Arial;margin:0;background:#0e1116;color:#e6edf3}}
 .wrap{{max-width:1100px;margin:0 auto;padding:24px}}
 h1{{font-size:22px;margin:0 0 4px}} h2{{font-size:16px;color:#9db2c9;margin:26px 0 10px;border-bottom:1px solid #263042;padding-bottom:6px}}
 .sub{{color:#8b98a8;font-size:13px;margin-bottom:16px}}
 .poi{{background:#161b22;border:1px solid #263042;border-radius:10px;padding:16px;font-size:14px;line-height:1.6}}
 .grid{{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:16px;margin-top:8px}}
 .card{{background:#161b22;border:1px solid #263042;border-radius:10px;overflow:hidden}}
 .card img{{width:100%;display:block}}
 .meta{{display:flex;gap:8px;align-items:center;padding:8px 10px;font-size:12px;flex-wrap:wrap}}
 .pan{{font-weight:700;letter-spacing:.5px}}
 .badge{{color:#fff;padding:1px 7px;border-radius:20px;font-size:11px}}
 .pose{{color:#8b98a8;margin-left:auto}}
 .card ul{{margin:0;padding:8px 10px 14px 26px;font-size:12.5px;line-height:1.5;color:#cdd7e2}}
 img.map{{width:100%;border:1px solid #263042;border-radius:10px}}
</style></head><body><div class="wrap">
 <h1>🤖 Robot room patrol — points of interest</h1>
 <div class="sub">Autonomous Nav2 drive + local vision model (<code>qwen3-vl-4b</code> in LM Studio). Camera on the arm, pan-swept right/centre/left at each stop. All processing on-robot / local network — nothing left the LAN.</div>

 <h2>Where the robot went</h2>
 <img class="map" src="data:image/png;base64,{tourmap}"/>
 <div class="sub" style="margin-top:8px">Honest result: the robot is <b>boxed into a small pocket</b> around the drum kit / desk corner. Every goal toward the big open areas (blue ★, ~5&nbsp;m away) <b>aborted</b> — Nav2 couldn't find a clear path through the surrounding clutter (drums, piano, boxes, chairs). So it swept the camera in place instead of physically crossing the room.</div>

 <h2>Consolidated points of interest</h2>
 <div class="poi">{POI}</div>

 <h2>Every frame the robot saw ({len(cards)} images)</h2>
 <div class="grid">{''.join(cards)}</div>

 <div class="sub" style="margin-top:24px">Note: a couple of frames the model labelled “person seated in chair / bare feet” — most likely dark furniture/fabric being over-read, but flagging it in case someone was actually in the room.</div>
</div></body></html>"""

out = os.path.join(D, "room_report.html")
open(out,"w",encoding="utf-8").write(htmlout)
print("wrote", out, os.path.getsize(out), "bytes,", len(cards), "cards")
