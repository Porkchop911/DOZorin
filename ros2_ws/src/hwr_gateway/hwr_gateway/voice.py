"""Speaker output: sherpa-onnx VITS on the Jetson CPU, espeak-ng German for the motto."""
from __future__ import annotations

import io
import os
import queue
import re
import subprocess
import threading
import wave

MODEL_DIR = os.environ.get("HWR_TTS_DIR", "/home/ubuntu/third_party/sherpa-onnx/vits-ljs")
# PulseAudio owns the USB card on the Jetson; raw plughw access fails with "device busy".
DEVICE = os.environ.get("HWR_AUDIO_DEVICE", "pulse")
_UID = os.getuid() if hasattr(os, "getuid") else 1000
os.environ.setdefault("PULSE_SERVER", f"unix:/run/user/{_UID}/pulse/native")
MOTTO = re.compile(r"allzeit\s+bereit!?", re.IGNORECASE)


def split_motto(text: str) -> list[tuple[str, str]]:
    """Split text into ("de"|"en", chunk) parts; the English model drops German words."""
    parts, pos = [], 0
    for match in MOTTO.finditer(text):
        if text[pos:match.start()].strip():
            parts.append(("en", text[pos:match.start()].strip()))
        parts.append(("de", "Allzeit bereit!"))
        pos = match.end()
    if text[pos:].strip():
        parts.append(("en", text[pos:].strip()))
    return parts


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
        vits = sherpa_onnx.OfflineTtsVitsModelConfig(
            model=f"{MODEL_DIR}/vits-ljs.onnx", lexicon=f"{MODEL_DIR}/lexicon.txt",
            tokens=f"{MODEL_DIR}/tokens.txt")
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
            for lang, chunk in split_motto(text):
                try:
                    if lang == "de":
                        wav = subprocess.run(["espeak-ng", "-v", "de", "--stdout", chunk],
                                             capture_output=True, timeout=10).stdout
                        self._play_wav(wav)
                    else:
                        self._speak_en(chunk)
                except Exception as exc:  # a failed utterance must not kill the speaker thread
                    print(f"voice: {exc}", flush=True)


if __name__ == "__main__":
    assert split_motto("Allzeit bereit! Hello comrade.") == [("de", "Allzeit bereit!"), ("en", "Hello comrade.")]
    assert split_motto("Ready. allzeit bereit") == [("en", "Ready."), ("de", "Allzeit bereit!")]
    assert split_motto("Just English.") == [("en", "Just English.")]
    print("ok")
