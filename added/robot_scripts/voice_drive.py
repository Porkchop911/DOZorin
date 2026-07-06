#!/usr/bin/env python3
import sys, os, subprocess, wave
import numpy as np
import sherpa_onnx
HOME=os.path.expanduser('~')
M=os.path.join(HOME,'third_party/sherpa-onnx/sherpa-onnx-streaming-zipformer-en-2023-06-21')
DUR=int(sys.argv[1]) if len(sys.argv)>1 else 5
WAV='/tmp/voice.wav'
print(f"[recording {DUR}s - speak your command now]", flush=True)
subprocess.run(['arecord','-D','plughw:0,0','-d',str(DUR),'-f','S16_LE','-r','16000','-c','1',WAV],
               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
print("[transcribing locally with sherpa-onnx...]", flush=True)
wf=wave.open(WAV,'rb'); sr=wf.getframerate()
samples=np.frombuffer(wf.readframes(wf.getnframes()), dtype=np.int16).astype(np.float32)/32768.0
wf.close()
rec=sherpa_onnx.OnlineRecognizer.from_transducer(
    tokens=os.path.join(M,'tokens.txt'),
    encoder=os.path.join(M,'encoder-epoch-99-avg-1.int8.onnx'),
    decoder=os.path.join(M,'decoder-epoch-99-avg-1.int8.onnx'),
    joiner=os.path.join(M,'joiner-epoch-99-avg-1.int8.onnx'),
    num_threads=2, sample_rate=16000, feature_dim=80, decoding_method='greedy_search')
s=rec.create_stream(); s.accept_waveform(sr, samples)
s.accept_waveform(sr, np.zeros(int(0.6*sr), dtype=np.float32)); s.input_finished()
while rec.is_ready(s): rec.decode_stream(s)
text=rec.get_result(s).strip().lower()
print(f"[heard]: {text!r}", flush=True)
if not text:
    print("[nothing recognized - speak louder/clearer and retry]"); sys.exit(0)
subprocess.run(['python3', os.path.join(HOME,'llm_drive.py'), text])
