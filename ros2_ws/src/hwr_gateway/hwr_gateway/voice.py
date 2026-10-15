"""Speaker output: sherpa-onnx Piper VITS (male voice) on the Jetson CPU."""
from __future__ import annotations

import io
import os
import queue
import subprocess
import threading
import wave

MODEL_DIR = os.environ.get("HWR_TTS_DIR", "/home/ubuntu/third_party/sherpa-onnx/vits-piper-en_GB-alan-medium")
# PulseAudio owns the USB card on the Jetson; raw plughw access fails with "device busy".
DEVICE = os.environ.get("HWR_AUDIO_DEVICE", "pulse")
_UID = os.getuid() if hasattr(os, "getuid") else 1000
os.environ.setdefault("PULSE_SERVER", f"unix:/run/user/{_UID}/pulse/native")


class Voice:
    def __init__(self, max_pending: int = 3):
        self.pending: queue.Queue[str] = queue.Queue(max_pending)
        self.tts = None
        threading.Thread(target=self._run, daemon=True).start()

    def say(self, text: str) -> bool:
        try:
            self.pending.put_nowait(text)
            return True
        except queue.Full:
            return False

    def _load(self):
        import sherpa_onnx
        name = os.path.basename(MODEL_DIR).removeprefix("vits-piper-")
        vits = sherpa_onnx.OfflineTtsVitsModelConfig(
            model=f"{MODEL_DIR}/{name}.onnx", tokens=f"{MODEL_DIR}/tokens.txt",
            data_dir=f"{MODEL_DIR}/espeak-ng-data")
        return sherpa_onnx.OfflineTts(sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(vits=vits, num_threads=4)))

    def _play_wav(self, data: bytes) -> None:
        subprocess.run(["aplay", "-q", "-D", DEVICE, "-"], input=data, timeout=60, check=False)

    def _speak_en(self, text: str) -> None:
        import numpy as np
        if self.tts is None:
            self.tts = self._load()
        audio = self.tts.generate(text, sid=0, speed=1.0)
        pcm = (np.clip(np.asarray(audio.samples), -1, 1) * 32767).astype("<i2").tobytes()
        buf = io.BytesIO()
        with wave.open(buf, "wb") as out:
            out.setnchannels(1)
            out.setsampwidth(2)
            out.setframerate(audio.sample_rate)
            out.writeframes(pcm)
        self._play_wav(buf.getvalue())

    def _run(self) -> None:
        while True:
            text = self.pending.get()
            try:
                self._speak_en(text)
            except Exception as exc:  # a failed utterance must not kill the speaker thread
                print(f"voice: {exc}", flush=True)
