import json, base64, os, html, re
D = r"C:\Users\Niko\AppData\Local\Temp\claude\D--robot\3dcf8889-30c3-4f0b-bf9d-3dea1bba7671\scratchpad\scout2"
def b64(p):
    with open(p,"rb") as f: return base64.b64encode(f.read()).decode()

def md2html(md):
    out=[]; inul=False
    for ln in md.splitlines():
        ln=ln.rstrip()
        def inline(s):
            s=html.escape(s)
            s=re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", s)
            s=re.sub(r'"(.+?)"', r"<em>\1</em>", s)
            return s
        if ln.startswith("## "):
            if inul: out.append("</ul>"); inul=False
            out.append("<h2>%s</h2>"%inline(ln[3:]))
        elif ln.startswith("**") and ln.endswith("**") and "*" not in ln[2:-2]:
            if inul: out.append("</ul>"); inul=False
            out.append("<h3>%s</h3>"%inline(ln.strip("*")))
        elif ln.lstrip().startswith(("* ","- ")):
            if not inul: out.append("<ul>"); inul=True
            out.append("<li>%s</li>"%inline(ln.lstrip()[2:]))
        elif not ln.strip():
            if inul: out.append("</ul>"); inul=False
        else:
            if inul: out.append("</ul>"); inul=False
            out.append("<p>%s</p>"%inline(ln))
    if inul: out.append("</ul>")
    return "\n".join(out)

synth = md2html(open(os.path.join(D,"synthesis.md"),encoding="utf-8").read())
V = json.load(open(os.path.join(D,"scout.json"),encoding="utf-8"))
cards=[]
for v in V:
    img=os.path.join(D,v["img"])
    if not os.path.exists(img): continue
    lines="".join("<li>%s</li>"%html.escape(x.strip(" -")) for x in v["desc"].split("\n") if x.strip(" -"))
    pose=v.get("pose"); posetxt=("(%.2f, %.2f)"%(pose[0],pose[1])) if pose else "—"
    cards.append(f"""<div class="card"><img src="data:image/jpeg;base64,{b64(img)}"/>
      <div class="meta"><span class="pan">{html.escape(v['tag'])}</span><span class="pose">pos {posetxt}</span></div>
      <ul>{lines}</ul></div>""")
mapb=b64(os.path.join(D,"map02f.png"))

doc=f"""<!doctype html><html><head><meta charset="utf-8"><style>
 body{{font-family:system-ui,Segoe UI,Arial;margin:0;background:#0e1116;color:#e6edf3}}
 .wrap{{max-width:1100px;margin:0 auto;padding:24px}}
 h1{{font-size:23px;margin:0 0 4px}} h2{{font-size:17px;color:#9db2c9;margin:24px 0 8px;border-bottom:1px solid #263042;padding-bottom:6px}}
 h3{{font-size:14px;color:#c9d6e5;margin:14px 0 4px}}
 .sub{{color:#8b98a8;font-size:13px;margin-bottom:14px}}
 .box{{background:#161b22;border:1px solid #263042;border-radius:10px;padding:6px 20px;font-size:14px;line-height:1.55}}
 .box ul{{margin:4px 0 10px 0;padding-left:20px}} .box li{{margin:2px 0}} .box p{{margin:8px 0}}
 .haz{{border-color:#7a3b12;background:#1e1409}}
 .grid{{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:14px;margin-top:8px}}
 .card{{background:#161b22;border:1px solid #263042;border-radius:10px;overflow:hidden}}
 .card img{{width:100%;display:block}}
 .meta{{display:flex;gap:8px;padding:7px 10px;font-size:12px}} .pan{{font-weight:700;color:#7fd1ff}} .pose{{color:#8b98a8;margin-left:auto}}
 .card ul{{margin:0;padding:6px 10px 12px 24px;font-size:12px;line-height:1.5;color:#cdd7e2}}
 img.map{{width:100%;border:1px solid #263042;border-radius:10px;background:#fff}}
</style></head><body><div class="wrap">
 <h1>🤖🏠 Downstairs room — autonomous scout report</h1>
 <div class="sub">SLAM mapping (slam_toolbox, retuned to stop CPU overload) + custom <b>continuous reactive frontier explorer</b> + local vision (<code>qwen3-vl-4b</code>), synthesized by a local LLM (<code>gemma-4-26b-it</code>). Fully on-device / LAN — nothing left the network. 18 spin-scan cycles, {len(cards)} frames. Roamed dining → living → kitchen.</div>

 <h2>Map built — <code>map_02</code> (~90 m² mapped)</h2>
 <img class="map" src="data:image/png;base64,{mapb}"/>
 <div class="sub" style="margin-top:8px">Blue line = the robot's roam path; orange = look stops. The big radial "sunburst" is the lidar shooting <b>through the floor-to-ceiling windows/openings</b> into adjacent/outdoor space, so the solid central blob is the room it actually drove; the spokes overstate floor area. It stayed cautious but did traverse and reach the kitchen.</div>

 <div class="box">{synth}</div>

 <h2>All {len(cards)} frames the robot saw</h2>
 <div class="grid">{''.join(cards)}</div>
</div></body></html>"""
out=os.path.join(D,"scout_report.html")
open(out,"w",encoding="utf-8").write(doc)
print("wrote",out,os.path.getsize(out),len(cards),"cards")
