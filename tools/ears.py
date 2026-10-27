#!/usr/bin/env python3
"""Soudruhu's ears (runs on the Jetson): wake word "Soudruhu" -> "Allzeit bereit!" -> Whisper -> Hermes.

Port of the July ~/assistant.py. Streaming sherpa spots the wake word; Whisper base.en transcribes
the command. Fixed motion commands ("turn left", "come here", "stop") run as reflexes straight on the
gateway (plan 15, layer L2'); everything else is posted into the Hermes robot session, which answers
through `say`.
Usage: ears.py --session <hermes session id> [--dry]
Env (from ~/hwr/config/hwr.env): HWR_HERMES_URL, HWR_HERMES_KEY, HWR_GATEWAY_TOKEN (or ~/.config/hwr/token).
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
NONSPEECH = re.compile(r"[\(\[][^)\]]*[\)\]]|\*[^*]*\*")  # Whisper tags: "(buzzing)", "[static]", "*sad music*"
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


NUMBERS = {"fifteen": 15, "twenty": 20, "thirty": 30, "forty five": 45, "forty-five": 45,
           "fifty": 50, "sixty": 60, "ninety": 90, "a hundred and eighty": 180,
           "one hundred and eighty": 180, "one eighty": 180, "one": 1, "two": 2, "half a": 0.5, "a": 1}
NUM = r"(\d+(?:\.\d+)?|" + "|".join(sorted(map(re.escape, NUMBERS), key=len, reverse=True)) + ")"
WAKE_PREFIX = re.compile(r"^(so+ ?who|sud\w*|sod\w*|sou\w*|zud\w*|comrade)\b[ ,]*")


def _num(text: str) -> float:
    return float(text) if text[0].isdigit() else float(NUMBERS[text])


def reflex(heard: str):
    """Fixed voice grammar -> (gateway path, payload, spoken ack); None = not a reflex, ask Hermes.

    The whole utterance must match, so "turn left and tell me what you see" still goes to Hermes."""
    text = re.sub(r"[^a-z0-9.\- ]", " ", heard.lower())
    text = WAKE_PREFIX.sub("", re.sub(r"\s+", " ", text).strip(" .")).strip()
    text = re.sub(r"^(please|okay|ok|now) |( please| now)$", "", text).strip(" .")
    if re.fullmatch(r"(stop|halt|freeze|stay)( (now|there|moving|it))?", text):
        return "/stop", None, "Stopping."
    if re.fullmatch(r"turn (around|back)", text):
        return "/motion/turn", {"degrees": 180.0}, "Turning around."
    if re.fullmatch(r"(spin( around)?|do a spin|turn a full circle)", text):
        return "/motion/turn", {"degrees": 360.0}, "Spinning."
    m = re.fullmatch(rf"turn (?:to the )?(left|right)(?: (?:by )?{NUM} degrees?)?", text)
    if m:
        degrees = _num(m.group(2)) if m.group(2) else 90.0
        return ("/motion/turn", {"degrees": degrees if m.group(1) == "left" else -degrees},
                f"Turning {m.group(1)}.")
    m = re.fullmatch(rf"(?:go|move|drive|step|walk) (forward|ahead|left|right)"
                     rf"(?: {NUM} (meters?|metres?|centimeters?|centimetres?|cm|m))?", text)
    if m:
        meters = 0.5
        if m.group(2):
            meters = _num(m.group(2)) / (100.0 if m.group(3).startswith(("cent", "cm")) else 1.0)
        direction = "forward" if m.group(1) == "ahead" else m.group(1)
        return "/motion/move", {"meters": meters, "direction": direction}, f"Moving {direction}."
    if re.fullmatch(r"come (here|to me|over here|closer)", text):
        return "/motion/approach", {"target": "person", "stop_at_m": 0.8}, "Coming, comrade."
    if re.fullmatch(r"(go|move|drive) back(wards?)?|back up|reverse", text):
        return None, None, "I cannot drive backwards. Tell me to turn around."
    return None


def hermes_message(heard: str) -> str:
    return (f"[Heard through your microphone] {heard}\n"
            "Answer out loud with say (one or two sentences, in character), then do what was asked.")


class NoiseFloor:
    """Fan-noise floor as a low percentile of recent idle 0.1 s chunk RMS; speech is a ratio above it.

    shortcut: SPEECH_RATIO=3 is a guess (July peaks: floor ~800, speech ~3100); tune from the
    [heard]/[unclear] log lines, which print rms/floor.
    """
    SPEECH_RATIO = 3.0

    def __init__(self, chunks: int = 100):
        self.recent, self.chunks = [], chunks

    def update(self, rms: float) -> None:
        self.recent = (self.recent + [rms])[-self.chunks:]

    @property
    def floor(self) -> float:
        return float(np.percentile(self.recent, 20)) if len(self.recent) >= 10 else float("inf")

    def is_speech(self, rms: float) -> bool:
        return rms >= self.SPEECH_RATIO * self.floor


def chunk_rms(raw: bytes) -> float:
    return float(np.sqrt(np.mean(np.frombuffer(raw, dtype=np.int16).astype(np.float32) ** 2)))


class Ears:
    def __init__(self, session: str, dry: bool):
        import requests
        import sherpa_onnx
        self.requests, self.session, self.dry = requests, session, dry
        self.url = os.environ["HWR_HERMES_URL"].rstrip("/")
        self.key = os.environ["HWR_HERMES_KEY"]
        self.busy = False
        self.pending = None   # one message heard while Hermes was busy; newest wins
        self.reflexes = []    # reflexes done since Hermes' last turn, prepended to its next message
        self.gateway = os.environ.get("HWR_GATEWAY_URL", "http://127.0.0.1:8090").rstrip("/")
        token_file = os.path.join(HOME, ".config/hwr/token")
        self.gateway_token = os.environ.get("HWR_GATEWAY_TOKEN") or (
            open(token_file).read().strip() if os.path.exists(token_file) else "")
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

    def handle(self, heard: str) -> None:
        match = reflex(heard)
        if match is None:
            self.send(heard)
            return
        path, payload, ack = match
        print(f"[reflex] {heard!r} -> {path} {payload}", flush=True)
        if self.dry:
            return

        def work():
            headers = {"Authorization": f"Bearer {self.gateway_token}"}
            try:
                self.requests.post(f"{self.gateway}/say", json={"text": ack}, headers=headers, timeout=5)
                if path is None:
                    outcome = "explained, no motion"
                else:
                    response = self.requests.post(f"{self.gateway}{path}", json=payload,
                                                  headers=headers, timeout=150)
                    outcome = (response.json().get("outcome", "done") if response.ok
                               else f"refused: {response.text[:120]}")
                    if outcome != "done":
                        self.requests.post(f"{self.gateway}/say", headers=headers, timeout=5,
                                           json={"text": f"I could not: {outcome[:80]}"})
            except Exception as exc:  # a failed reflex must not kill the listener
                outcome = f"error: {exc}"
            print(f"[reflex done] {outcome}", flush=True)
            self.reflexes.append(f"heard {heard!r} -> {path or 'spoken reply'} {payload or ''}: {outcome}")
        threading.Thread(target=work, daemon=True).start()

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
                done, self.reflexes = self.reflexes, []
                notes = ("[Done by your reflexes since your last turn; already handled, no action needed: "
                         + "; ".join(done) + "]\n") if done else ""
                response = self.requests.post(
                    f"{self.url}/api/sessions/{self.session}/chat",
                    headers={"Authorization": f"Bearer {self.key}"},
                    json={"message": notes + hermes_message(heard)}, timeout=600)
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
        noise, cmd_rms = NoiseFloor(), 0.0
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
                            cmd_started, cmd_max, cmd_rms = True, 0, 0.0
                        cmd_buf += raw
                        cmd_max = max(cmd_max, int(np.abs(np.frombuffer(raw, dtype=np.int16)).max()))
                        cmd_rms = max(cmd_rms, chunk_rms(raw))
                    # Fixed peak gate (loud speech) OR well above the measured fan floor (soft speech).
                    voiced = cmd_max > 2500 or noise.is_speech(cmd_rms)
                    ended = (cmd_started and voiced and len(cmd_buf) > 12000
                             and self.rec.is_endpoint(stream)) or now >= capture_until
                    if cmd_started and ended:
                        armed = cmd_started = False
                        audio, cmd_buf = cmd_buf, b""
                        self.rec.reset(stream)
                        level = f"peak={cmd_max} rms={cmd_rms:.0f} floor={noise.floor:.0f}"
                        command = self.transcribe(audio) if cmd_max > 2000 or noise.is_speech(cmd_rms) else ""
                        clean = spoken_text(command).lower()
                        if clean and clean not in JUNK and len(clean) > 2:
                            print(f"[heard] {command} ({level})", flush=True)
                            self.handle(command)
                        else:
                            print(f"[unclear, ignored: {command!r}, {level}]", flush=True)
                else:
                    utt_buf = (utt_buf + raw)[-16000 * 2 * 8:]   # current utterance, max 8 s
                    noise.update(chunk_rms(raw))
                if not armed and self.rec.is_endpoint(stream):
                    text = self.rec.get_result(stream).strip().lower()
                    self.rec.reset(stream)
                    utterance, utt_buf = utt_buf, b""
                    if text and is_wake(text) and len(text.split()) > 2 and not speaker_busy():
                        # Name and command in one breath ("Soudruhu, turn right").
                        command = spoken_text(self.transcribe(utterance))
                        print(f"[WAKE+heard] ({text!r}) {command}", flush=True)
                        if len(command.split()) > 1:
                            self.handle(command)
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
    _noise = NoiseFloor()
    assert not _noise.is_speech(1e9)  # no floor measured yet -> only the fixed peak gate applies
    for _ in range(100):
        _noise.update(200.0)
    assert _noise.is_speech(650.0) and not _noise.is_speech(500.0)
    assert chunk_rms(np.full(1600, 300, dtype=np.int16).tobytes()) == 300.0
    assert not is_wake("this is a long sentence about nothing")
    assert "[Heard through your microphone] go left" in hermes_message("go left")
    assert reflex("So who, turn right.") == ("/motion/turn", {"degrees": -90.0}, "Turning right.")
    assert reflex("Turn left 45 degrees")[1] == {"degrees": 45.0}
    assert reflex("turn to the left by ninety degrees")[1] == {"degrees": 90.0}
    assert reflex("turn around please")[1] == {"degrees": 180.0}
    assert reflex("Move forward 30 centimeters.")[1] == {"meters": 0.3, "direction": "forward"}
    assert reflex("go ahead")[1] == {"meters": 0.5, "direction": "forward"}
    assert reflex("move left half a meter")[1] == {"meters": 0.5, "direction": "left"}
    assert reflex("Come here!")[0] == "/motion/approach"
    assert reflex("Stop.")[0] == "/stop" and reflex("go back")[0] is None
    assert reflex("turn left and tell me what you see") is None and reflex("what do you see") is None
    assert spoken_text("(buzzing)") == "" and spoken_text("So who, turn right. [static]") == "So who, turn right"
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", required=True)
    parser.add_argument("--dry", action="store_true")
    args = parser.parse_args()
    Ears(args.session, args.dry).run()
