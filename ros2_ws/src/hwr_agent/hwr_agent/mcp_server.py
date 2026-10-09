"""Optional MCP exposure of the same bounded gateway skills."""
from __future__ import annotations

from .skills import GatewaySkills


def main():
    try:
        from mcp.server.fastmcp import FastMCP
    except ImportError as exc:
        raise SystemExit("install MCP support: python3 -m pip install mcp") from exc
    skills = GatewaySkills()
    server = FastMCP("hwr")
    server.tool()(skills.goto_place)
    server.tool()(skills.goto)
    server.tool()(skills.explore)
    server.tool()(skills.look)
    server.tool()(skills.describe_view)
    server.tool()(skills.assess_scene)
    server.tool()(skills.patrol)
    server.tool()(skills.arm_pose)
    server.tool()(skills.pick_test)
    server.tool()(skills.where_am_i)
    server.run()
