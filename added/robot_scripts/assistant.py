#!/usr/bin/env python3
# Always-on wake-word assistant. Wake word = Czech "Soudruhu".
# Streaming sherpa (tiny) spots the wake word; Whisper base.en transcribes the COMMAND accurately.
# Two-step: say "Soudruhu", pause, then the command. Routes to look.py (vision) / llm_drive.py (motion).
# Usage: assistant.py [--dry] [--seconds N]
import sys, os, time, subprocess, threading
import numpy as np
import sherpa_onnx
HOME=os.path.expanduser('~')
M=os.path.join(HOME,'third_party/sherpa-onnx/sherpa-onnx-streaming-zipformer-en-2023-06-21')
W=os.path.join(HOME,'third_party/sherpa-onnx/sherpa-onnx-whisper-base.en')
args=list(sys.argv[1:])
DRY='--dry' in args; args=[a for a in args if a!='--dry']
SECS=0
if '--seconds' in args:
    i=args.index('--seconds'); SECS=int(args[i+1]); del args[i:i+2]

WAKE_TOKENS=("noble","glad","darkly","global","dawn","mothel","drew","comrade")
JUNK={"you","thank you","thanks","the","bye","so","um","uh","hmm","news networks","force","static","okay","ok"}
def is_wake(text):
    w=text.lower().split()
    if not w or len(w)>4: return False
    if any(tok in text.lower() for tok in WAKE_TOKENS): return True
    if w[0] in ("so","soo") or w[0].startswith(("sud","sod","sou","zud","zud")): return True
    return False

# streaming recognizer -> wake spotting
rec=sherpa_onnx.OnlineRecognizer.from_transducer(
  tokens=os.path.join(M,'tokens.txt'),
  encoder=os.path.join(M,'encoder-epoch-99-avg-1.int8.onnx'),
  decoder=os.path.join(M,'decoder-epoch-99-avg-1.int8.onnx'),
  joiner=os.path.join(M,'joiner-epoch-99-avg-1.int8.onnx'),
  num_threads=2, sample_rate=16000, feature_dim=80, decoding_method='greedy_search',
  enable_endpoint_detection=True, rule1_min_trailing_silence=1.3, rule2_min_trailing_silence=0.8, rule3_min_utterance_length=200)
stream=rec.create_stream()
# offline Whisper -> accurate command transcription
whisper=sherpa_onnx.OfflineRecognizer.from_whisper(
  encoder=os.path.join(W,'base.en-encoder.int8.onnx'),
  decoder=os.path.join(W,'base.en-decoder.int8.onnx'),
  tokens=os.path.join(W,'base.en-tokens.txt'),
  language='en', task='transcribe', num_threads=4)
def transcribe(raw):
    if len(raw)<3200: return ""
    st=whisper.create_stream()
    st.accept_waveform(16000, np.frombuffer(raw,dtype=np.int16).astype(np.float32)/32768.0)
    whisper.decode_stream(st)
    return st.result.text.strip()

VIS=["see","look","what","describe","find","spot","colour","color","there","front","watch"]
def route(text):
    t=text.lower()
    if any(w in t for w in VIS):
        pan="left" if "left" in t else ("right" if "right" in t else "center")
        print("   [-> VISION pan=%s] %s"%(pan,text), flush=True)
        if not DRY: subprocess.run(['python3',os.path.join(HOME,'look.py'),pan,text])
    else:
        print("   [-> DRIVE] %s"%text, flush=True)
        if not DRY: subprocess.run(['python3',os.path.join(HOME,'llm_drive.py'),text])
_busy={"v":False}
def route_async(text):
    if _busy["v"]:
        print("   [busy - ignoring] %s"%text, flush=True); return
    _busy["v"]=True
    def work():
        try: route(text)
        finally: _busy["v"]=False
    threading.Thread(target=work, daemon=True).start()
RESP=os.path.join(HOME,'allzeit.wav')
def play_response():
    try: subprocess.run(['aplay','-q','-D','plughw:0,0',RESP], stderr=subprocess.DEVNULL)
    except Exception: pass

print("Comrade assistant online (Whisper + Allzeit bereit). wake='Soudruhu' dry=%s. Say wake word, wait for 'Allzeit bereit', then command."%DRY, flush=True)
proc=subprocess.Popen(['arecord','-D','plughw:0,0','-f','S16_LE','-r','16000','-c','1','-t','raw','-q'],stdout=subprocess.PIPE)
start=time.time(); armed=False; cmd_buf=b''; capture_from=0; capture_until=0; cmd_started=False; cmd_max=0
try:
    while True:
        if SECS and time.time()-start>SECS: break
        raw=proc.stdout.read(3200)
        if not raw: break
        stream.accept_waveform(16000, np.frombuffer(raw,dtype=np.int16).astype(np.float32)/32768.0)
        while rec.is_ready(stream): rec.decode_stream(stream)
        if armed:
            now=time.time()
            if now>=capture_from:
                if not cmd_started:
                    rec.reset(stream); cmd_started=True; cmd_max=0   # fresh stream for command only
                cmd_buf+=raw
                a=int(np.abs(np.frombuffer(raw,dtype=np.int16)).max())
                if a>cmd_max: cmd_max=a
            # end early once speech happened then went quiet (endpoint); else hard cap
            ended = (cmd_started and cmd_max>2500 and len(cmd_buf)>12000 and rec.is_endpoint(stream)) or now>=capture_until
            if cmd_started and ended:
                armed=False; cmd_started=False
                arr=np.frombuffer(cmd_buf,dtype=np.int16); cmd_buf=b''; rec.reset(stream)
                if cmd_max>2000:
                    command=transcribe(arr.tobytes())
                    cl=command.lower().strip(" .,!?")
                    if cl and cl not in JUNK and len(cl)>2:
                        print("[command] %s"%command, flush=True); route_async(command)
                    else:
                        print("[unclear, ignored: %r]"%command, flush=True)
                else:
                    print("[no speech after wake]", flush=True)
        elif rec.is_endpoint(stream):
            text=rec.get_result(stream).strip().lower(); rec.reset(stream)
            if text and is_wake(text):
                print("[WAKE] Soudruhu! (%r) -> Allzeit bereit!"%text, flush=True)
                threading.Thread(target=play_response, daemon=True).start()
                armed=True; cmd_buf=b''; cmd_started=False; cmd_max=0
                capture_from=time.time()+1.6   # skip audio while the response plays
                capture_until=capture_from+3.3
            elif text:
                print("[ignored] %s"%text, flush=True)
finally:
    proc.terminate()
print("assistant stopped", flush=True)
