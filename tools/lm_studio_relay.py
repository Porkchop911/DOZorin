"""Small process-scoped TCP relay for robot-to-PC LM Studio access."""
from __future__ import annotations

import argparse
import asyncio


async def _pipe(reader: asyncio.StreamReader,
                writer: asyncio.StreamWriter) -> None:
    try:
        while chunk := await reader.read(64 * 1024):
            writer.write(chunk)
            await writer.drain()
    except (ConnectionError, asyncio.CancelledError):
        pass
    finally:
        writer.close()


async def _serve(args: argparse.Namespace) -> None:
    async def handle(client_reader: asyncio.StreamReader,
                     client_writer: asyncio.StreamWriter) -> None:
        peer = client_writer.get_extra_info("peername")
        peer_host = peer[0] if peer else None
        if args.allowed_client and peer_host != args.allowed_client:
            client_writer.close()
            await client_writer.wait_closed()
            return
        try:
            upstream_reader, upstream_writer = await asyncio.open_connection(
                args.target_host, args.target_port)
        except OSError:
            client_writer.close()
            await client_writer.wait_closed()
            return
        await asyncio.gather(
            _pipe(client_reader, upstream_writer),
            _pipe(upstream_reader, client_writer),
        )

    server = await asyncio.start_server(
        handle, args.listen_host, args.listen_port)
    async with server:
        await server.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=12340)
    parser.add_argument("--target-host", default="127.0.0.1")
    parser.add_argument("--target-port", type=int, default=1234)
    parser.add_argument("--allowed-client")
    args = parser.parse_args()
    asyncio.run(_serve(args))


if __name__ == "__main__":
    main()
