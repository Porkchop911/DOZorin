#!/usr/bin/env python3
import os, time, subprocess
import numpy as np, sherpa_onnx
HOME=os.path.expanduser('~')
M=os.path.join(HOME,'third_party/sherpa-onnx/sherpa-onnx-streaming-zipformer-en-2023-06-21')
rec=sherpa_onnx.OnlineRecognizer.from_transducer(
  tokens=os.path.join(M,'tokens.txt'),
  encoder=os.path.join(M,'encoder-epoch-99-avg-1.int8.onnx'),
  decoder=os.path.join(M,'decoder-epoch-99-avg-1.int8.onnx'),
  joiner=os.path.join(M,'joiner-epoch-99-avg-1.int8.onnx'),
  num_threads=2, sample_rate=16000, feature_dim=80, decoding_method='greedy_search',
  enable_endpoint_detection=True, rule1_min_trailing_silence=1.5, rule2_min_trailing_silence=0.8, rule3_min_utterance_length=200)
stream=rec.create_stream()
proc=subprocess.Popen(['arecord','-D','plughw:0,0','-f','S16_LE','-r','16000','-c','1','-t','raw','-q'],stdout=subprocess.PIPE)
print("listening 20s, talk now...", flush=True)
start=time.time(); last=""; tick=0
while time.time()-start<20:
    raw=proc.stdout.read(3200)
    if not raw:
        print("NO DATA from arecord", flush=True); break
    s=np.frombuffer(raw,dtype=np.int16).astype(np.float32)
    amp=int(np.abs(s).max())
    stream.accept_waveform(16000, s/32768.0)
    while rec.is_ready(stream): rec.decode_stream(stream)
    r=rec.get_result(stream).strip()
    ep=rec.is_endpoint(stream)
    tick+=1
    if r!=last or ep:
        print("amp=%d ep=%s partial='%s'"%(amp,ep,r), flush=True); last=r
    elif tick%10==0:
        print("amp=%d (no text)"%amp, flush=True)
    if ep:
        rec.reset(stream); last=""
proc.terminate()
print("done", flush=True)
