import json

import pytest
import requests

from hwr_core.navigation import SemanticPreflightClient


class Response:
    def __init__(self, *, data=None, content=b"jpeg", ok=True, status_code=200):
        self._data = data
        self.content = content
        self.ok = ok
        self.status_code = status_code
        self.text = "response"

    def json(self):
        return self._data


def _context(clearance=1.0):
    return {
        "localization": {"healthy": True},
        "camera_fresh": True,
        "camera_forward": True,
        "current_goal": None,
        "drive_active": False,
        "hard_clearance_m": {"forward": clearance},
        "clearance_limit_m": 1.5,
        "trapped_start": False,
    }


def _visual(*, recommendation="sensor_preflight", hazards=None):
    return {
        "visibility_sufficient": True,
        "doorway_visible": False,
        "person_visible": False,
        "person_in_corridor": False,
        "glass_boundary_visible": False,
        "glass_boundary_in_corridor": False,
        "glass_boundary_evidence": "",
        "hazards": [] if hazards is None else hazards,
        "visual_corridor": "clear open floor lane",
        "recommendation": recommendation,
        "rationale": "complete proposed corridor is visible",
        "doorway_evidence": "",
        "person_evidence": "",
    }


def test_semantic_client_combines_visual_and_metric_evidence(monkeypatch):
    context = _context()

    def fake_get(url, **_kwargs):
        return Response(data=context) if url.endswith("/navigation/context") else Response()

    monkeypatch.setattr(requests, "get", fake_get)
    monkeypatch.setattr(requests, "post", lambda *_args, **_kwargs: Response(
        data={"choices": [{"message": {"content": json.dumps(_visual())}}]}))
    client = SemanticPreflightClient(
        "http://gateway", "token", "http://vlm/v1", "vision")
    allowed, report = client.assess(0.20)
    assert allowed
    assert report["metric_allowed"]
    assert report["assessment"]["safe_to_translate"]


def test_semantic_client_cannot_override_tight_metric_clearance(monkeypatch):
    context = _context(clearance=0.25)
    monkeypatch.setattr(requests, "get", lambda url, **_kwargs: (
        Response(data=context) if url.endswith("/navigation/context") else Response()))
    monkeypatch.setattr(requests, "post", lambda *_args, **_kwargs: Response(
        data={"choices": [{"message": {"content": json.dumps(_visual())}}]}))
    client = SemanticPreflightClient(
        "http://gateway", "token", "http://vlm/v1", "vision", margin=0.10)
    allowed, report = client.assess(0.20)
    assert not allowed
    assert not report["metric_allowed"]
    assert "insufficient forward clearance" in report["reasons"]


def test_semantic_client_retries_then_fails_closed(monkeypatch):
    context = _context()
    monkeypatch.setattr(requests, "get", lambda url, **_kwargs: (
        Response(data=context) if url.endswith("/navigation/context") else Response()))

    def unavailable(*_args, **_kwargs):
        raise requests.ConnectionError("offline")

    monkeypatch.setattr(requests, "post", unavailable)
    client = SemanticPreflightClient(
        "http://gateway", "token", "http://vlm/v1", "vision", retries=1)
    with pytest.raises(RuntimeError, match="2 attempts"):
        client.assess(0.10)


def test_semantic_client_repairs_one_schema_omission(monkeypatch):
    context = _context()
    monkeypatch.setattr(requests, "get", lambda url, **_kwargs: (
        Response(data=context) if url.endswith("/navigation/context") else Response()))
    incomplete = _visual()
    incomplete.pop("rationale")
    replies = iter((incomplete, _visual()))
    payloads = []

    def reply(*_args, **kwargs):
        payloads.append(kwargs["json"])
        return Response(data={"choices": [{"message": {
            "content": json.dumps(next(replies))}}]})

    monkeypatch.setattr(requests, "post", reply)
    client = SemanticPreflightClient(
        "http://gateway", "token", "http://vlm/v1", "vision")
    allowed, report = client.assess(0.10)
    assert allowed
    assert report["vlm_attempts"] == 2
    assert len(payloads[1]["messages"]) == 3
    assert "Never omit rationale" in payloads[1]["messages"][-1]["content"]


def test_semantic_client_fails_after_one_bad_schema_repair(monkeypatch):
    context = _context()
    monkeypatch.setattr(requests, "get", lambda url, **_kwargs: (
        Response(data=context) if url.endswith("/navigation/context") else Response()))
    incomplete = _visual()
    incomplete.pop("rationale")
    monkeypatch.setattr(requests, "post", lambda *_args, **_kwargs: Response(
        data={"choices": [{"message": {"content": json.dumps(incomplete)}}]}))
    client = SemanticPreflightClient(
        "http://gateway", "token", "http://vlm/v1", "vision")
    with pytest.raises(ValueError, match="after one schema repair"):
        client.assess(0.10)
