#!/usr/bin/env python3
"""Soudruhu's ears (runs on the Jetson): wake word "Soudruhu" -> "Allzeit bereit!" -> Whisper -> Hermes.

Port of the July ~/assistant.py. Streaming sherpa spots the wake word; Whisper base.en transcribes
the command; the text is posted into the Hermes robot session, which answers through `say`.
Usage: ears.py --session <hermes session id> [--dry]
Env (from ~/hwr/config/hwr.env): HWR_HERMES_URL, HWR_HERMES_KEY.
"""
import argparse
import os
import re
import subprocess
import threading
import time

import numpy as np

HOME = os.path.expanduser("~")
M = os.path.join(HOME, "third_party/sherpa-onnx/sherpa-onnx-streaming-zipformer-en-2023-06-21")
W = os.path.join(HOME, "third_party/sherpa-onnx/sherpa-onnx-whisper-base.en")
RESP = os.path.join(HOME, "allzeit.wav")
os.environ.setdefault("PULSE_SERVER", f"unix:/run/user/{os.getuid()}/pulse/native")

# Calibrated renderings of Czech "Soudruhu" by the English model (July calibrate_wake.py).
# "comrade" removed: Soudruhu says it himself and would wake himself up.
WAKE_TOKENS = ("noble", "glad", "darkly", "global", "dawn", "mothel", "drew")
NONSPEECH = re.compile(r"[\(\[][^)\]]*[\)\]]")  # Whisper tags like "(buzzing)", "[static]"
JUNK = {"you", "thank you", "thanks", "the", "bye", "so", "um", "uh", "hmm", "news networks",
        "force", "static", "okay", "ok"}


def is_wake(text: str) -> bool:
    words = text.lower().split()
    if not words or len(words) > 4:
        return False
    if any(tok in text.lower() for tok in WAKE_TOKENS):
        return True
    return words[0] in ("so", "soo") or words[0].startswith(("sud", "sod", "sou", "zud"))


def spoken_text(text: str) -> str:
    return NONSPEECH.sub(" ", text).strip(" .,!?")


def speaker_busy() -> bool:
    """True while PulseAudio is playing on the USB card, i.e. Soudruhu is talking."""
    out = subprocess.run(["pactl", "list", "short", "sinks"], capture_output=True, text=True).stdout
    return any("usb" in line.lower() and "RUNNING" in line for line in out.splitlines())


def hermes_message(heard: str) -> str:
    return (f"[Heard through your microphone] {heard}\n"
            "Answer out loud with say (one or two sentences, in character), then do what was asked.")


class Ears:
    def __init__(self, session: str, dry: bool):
        import requests
        import sherpa_onnx
        self.requests, self.session, self.dry = requests, session, dry
        self.url = os.environ["HWR_HERMES_URL"].rstrip("/")
        self.key = os.environ["HWR_HERMES_KEY"]
        self.busy = False
        self.pending = None   # one message heard while Hermes was busy; newest wins
        self.rec = sherpa_onnx.OnlineRecognizer.from_transducer(
            tokens=f"{M}/tokens.txt", encoder=f"{M}/encoder-epoch-99-avg-1.int8.onnx",
            decoder=f"{M}/decoder-epoch-99-avg-1.int8.onnx",
            joiner=f"{M}/joiner-epoch-99-avg-1.int8.onnx", num_threads=2, sample_rate=16000,
            feature_dim=80, decoding_method="greedy_search", enable_endpoint_detection=True,
            rule1_min_trailing_silence=1.3, rule2_min_trailing_silence=0.8,
            rule3_min_utterance_length=200)
        self.whisper = sherpa_onnx.OfflineRecognizer.from_whisper(
            encoder=f"{W}/base.en-encoder.int8.onnx", decoder=f"{W}/base.en-decoder.int8.onnx",
            tokens=f"{W}/base.en-tokens.txt", language="en", task="transcribe", num_threads=4)

    def transcribe(self, raw: bytes) -> str:
        if len(raw) < 3200:
            return ""
        st = self.whisper.create_stream()
        st.accept_waveform(16000, np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0)
        self.whisper.decode_stream(st)
        return st.result.text.strip()

    def send(self, heard: str) -> None:
        if self.busy:
            self.pending = heard
            print(f"[busy, queued] {heard}", flush=True)
            return
        self.busy = True

        def work():
            try:
                if self.dry:
                    print(f"[dry] would send: {heard}", flush=True)
                    return
                response = self.requests.post(
                    f"{self.url}/api/sessions/{self.session}/chat",
                    headers={"Authorization": f"Bearer {self.key}"},
                    json={"message": hermes_message(heard)}, timeout=600)
                reply = response.json().get("message", {}).get("content", "") if response.ok else response.text
                print(f"[hermes {response.status_code}] {reply[:200]}", flush=True)
            except Exception as exc:  # a dropped request must not kill the listener
                print(f"[hermes error] {exc}", flush=True)
            finally:
                self.busy = False
                if self.pending:
                    queued, self.pending = self.pending, None
                    self.send(queued)
        threading.Thread(target=work, daemon=True).start()

    def run(self) -> None:
        stream = self.rec.create_stream()
        proc = subprocess.Popen(["arecord", "-D", "pulse", "-f", "S16_LE", "-r", "16000", "-c", "1",
                                 "-t", "raw", "-q"], stdout=subprocess.PIPE)
        print(f"ears online: session={self.session} dry={self.dry}", flush=True)
        armed = cmd_started = False
        cmd_buf, cmd_max, capture_from, capture_until = b"", 0, 0.0, 0.0
        utt_buf = b""
        try:
            while raw := proc.stdout.read(3200):
                stream.accept_waveform(16000, np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0)
                while self.rec.is_ready(stream):
                    self.rec.decode_stream(stream)
                now = time.time()
                if armed:
                    if now >= capture_from:
                        if not cmd_started:
                            self.rec.reset(stream)
                            cmd_started, cmd_max = True, 0
                        cmd_buf += raw
                        cmd_max = max(cmd_max, int(np.abs(np.frombuffer(raw, dtype=np.int16)).max()))
                    ended = (cmd_started and cmd_max > 2500 and len(cmd_buf) > 12000
                             and self.rec.is_endpoint(stream)) or now >= capture_until
                    if cmd_started and ended:
                        armed = cmd_started = False
                        audio, cmd_buf = cmd_buf, b""
                        self.rec.reset(stream)
                        command = self.transcribe(audio) if cmd_max > 2000 else ""
                        clean = spoken_text(command).lower()
                        if clean and clean not in JUNK and len(clean) > 2:
                            print(f"[heard] {command}", flush=True)
                            self.send(command)
                        else:
                            print(f"[unclear, ignored: {command!r}, peak={cmd_max}]", flush=True)
                else:
                    utt_buf = (utt_buf + raw)[-16000 * 2 * 8:]   # current utterance, max 8 s
                if not armed and self.rec.is_endpoint(stream):
                    text = self.rec.get_result(stream).strip().lower()
                    self.rec.reset(stream)
                    utterance, utt_buf = utt_buf, b""
                    if text and is_wake(text) and len(text.split()) > 2 and not speaker_busy():
                        # Name and command in one breath ("Soudruhu, turn right").
                        command = spoken_text(self.transcribe(utterance))
                        print(f"[WAKE+heard] ({text!r}) {command}", flush=True)
                        if len(command.split()) > 1:
                            self.send(command)
                    elif text and is_wake(text) and not speaker_busy():
                        print(f"[WAKE] ({text!r}) -> Allzeit bereit!", flush=True)
                        subprocess.Popen(["aplay", "-q", "-D", "pulse", RESP],
                                         stderr=subprocess.DEVNULL)
                        armed, cmd_buf, cmd_started, cmd_max = True, b"", False, 0
                        capture_from = time.time() + 1.6   # skip our own reply audio
                        capture_until = capture_from + 6.0
        finally:
            proc.terminate()


if __name__ == "__main__":
    assert is_wake("sudru") and is_wake("so who") and not is_wake("comrade")
    assert not is_wake("this is a long sentence about nothing")
    assert "[Heard through your microphone] go left" in hermes_message("go left")
    assert spoken_text("(buzzing)") == "" and spoken_text("So who, turn right. [static]") == "So who, turn right"
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", required=True)
    parser.add_argument("--dry", action="store_true")
    args = parser.parse_args()
    Ears(args.session, args.dry).run()
