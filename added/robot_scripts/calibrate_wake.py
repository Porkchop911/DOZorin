#!/usr/bin/env python3
# Calibration: print how the English sherpa model transcribes a spoken (Czech) wake word.
import os, sys, time, subprocess
import numpy as np, sherpa_onnx
HOME=os.path.expanduser('~')
M=os.path.join(HOME,'third_party/sherpa-onnx/sherpa-onnx-streaming-zipformer-en-2023-06-21')
SECS=int(sys.argv[1]) if len(sys.argv)>1 else 40
rec=sherpa_onnx.OnlineRecognizer.from_transducer(
  tokens=os.path.join(M,'tokens.txt'),
  encoder=os.path.join(M,'encoder-epoch-99-avg-1.int8.onnx'),
  decoder=os.path.join(M,'decoder-epoch-99-avg-1.int8.onnx'),
  joiner=os.path.join(M,'joiner-epoch-99-avg-1.int8.onnx'),
  num_threads=2, sample_rate=16000, feature_dim=80, decoding_method='greedy_search',
  enable_endpoint_detection=True, rule1_min_trailing_silence=1.2, rule2_min_trailing_silence=0.8, rule3_min_utterance_length=200)
stream=rec.create_stream()
proc=subprocess.Popen(['arecord','-D','plughw:0,0','-f','S16_LE','-r','16000','-c','1','-t','raw','-q'],stdout=subprocess.PIPE)
print("CALIBRATION: say the wake word clearly ~6 times, pause between each. (%ds)"%SECS, flush=True)
start=time.time()
try:
    while time.time()-start<SECS:
        raw=proc.stdout.read(3200)
        if not raw: break
        s=np.frombuffer(raw,dtype=np.int16).astype(np.float32)/32768.0
        stream.accept_waveform(16000,s)
        while rec.is_ready(stream): rec.decode_stream(stream)
        if rec.is_endpoint(stream):
            text=rec.get_result(stream).strip()
            rec.reset(stream)
            if text: print("HEARD: %r"%text, flush=True)
finally:
    proc.terminate()
print("calibration done", flush=True)
