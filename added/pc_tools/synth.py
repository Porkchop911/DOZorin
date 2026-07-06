import json, urllib.request, os
D = r"C:\Users\Niko\AppData\Local\Temp\claude\D--robot\3dcf8889-30c3-4f0b-bf9d-3dea1bba7671\scratchpad\scout2"
V = json.load(open(os.path.join(D,"scout.json"), encoding="utf-8"))
obs = "\n".join("[view %s]\n%s" % (v["tag"], v["desc"]) for v in V)

prompt = f"""A mobile robot autonomously spin-scanned a downstairs room, taking camera shots at multiple angles. A vision model produced these raw per-view object lists (they overlap and repeat, and the arm-camera sits low so some items are seen from odd angles):

{obs}

Consolidate this into a clean report. Output in markdown with exactly these sections:
## Room summary
One or two sentences: what kind of room this is and its overall character.
## Inventory by zone
Grouped bullet lists (e.g. Dining, Lounge/living, Greenery/plants, Media & electronics, Other). Merge duplicates across views into single entries. Keep only things actually observed.
## Points of interest
5-8 short highlights a person would care about.
## Hazards & notes
Note anything safety-relevant for the robot (e.g. a staircase was seen) and note the lists came from a low-mounted camera so some IDs are approximate.
Be factual and concise. Do not invent items that were not in the observations."""

body = {
    "model": "gemma-4-26b-a4b-it",
    "temperature": 0.3,
    "max_tokens": 1200,
    "messages": [{"role": "user", "content": prompt}],
}
req = urllib.request.Request("http://127.0.0.1:1234/v1/chat/completions",
    data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
resp = json.load(urllib.request.urlopen(req, timeout=300))
msg = resp["choices"][0]["message"]
out = msg.get("content") or ""
if not out.strip():
    out = msg.get("reasoning_content") or ""
print("FINISH:", resp["choices"][0].get("finish_reason"), "LEN:", len(out))
out = out.replace("<think>", "").replace("</think>", "").strip()
open(os.path.join(D, "synthesis.md"), "w", encoding="utf-8").write(out)
print("=====")
print(out)
