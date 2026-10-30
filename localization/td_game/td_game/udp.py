"""Non-blocking UDP endpoints for the GameController and team messages (LOC-08). Standard library only.

`GameControllerLink`:
- receives control messages (broadcast, CONTROL_PORT);
- sends status messages back to the GameController that sent them (unicast, STATUS_PORT), as the protocol
  requires.
`TeamLink`: broadcasts and receives team messages on TEAM_PORT_BASE + team number.

Both are poll-based (`poll()` returns everything waiting) so a ROS timer or test loop can drive them without
threads. Ports and the broadcast address are parameters so tests can use loopback and free ports.

Not tested on a real field network yet. Interface choice, firewall rules and the GameController's
"limited broadcast" option are deployment concerns (see the GameController README, "Network Communication").
"""

from __future__ import annotations

import socket

from td_game.gc_data import (
    CONTROL_PORT,
    STATUS_PORT,
    TEAM_PORT_BASE,
    ControlData,
    GCFormatError,
    StatusData,
    build_status,
    parse_control,
)
from td_game.team_msg import MAX_PAYLOAD


def _udp_socket(bind_port: int, bind_host: str = "") -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((bind_host, bind_port))
    s.setblocking(False)
    return s


class GameControllerLink:
    def __init__(self, control_port: int = CONTROL_PORT, status_port: int = STATUS_PORT, bind_host: str = ""):
        self.sock = _udp_socket(control_port, bind_host)
        self.status_port = status_port
        self.gc_address: str | None = None  # learned from the last valid control message
        self.rejected = 0

    def poll(self) -> list[ControlData]:
        """All valid control messages waiting, oldest first. Invalid packets are counted and dropped."""
        out = []
        while True:
            try:
                data, (host, _port) = self.sock.recvfrom(2048)
            except (BlockingIOError, InterruptedError):
                return out
            try:
                out.append(parse_control(data))
                self.gc_address = host
            except GCFormatError:
                self.rejected += 1

    def send_status(self, status: StatusData) -> bool:
        """Unicast our status to the GameController (0.5–2 Hz required). False until one is known."""
        if self.gc_address is None:
            return False
        self.sock.sendto(build_status(status), (self.gc_address, self.status_port))
        return True

    def close(self) -> None:
        self.sock.close()


class TeamLink:
    def __init__(
        self,
        team_number: int,
        broadcast: str = "255.255.255.255",
        port: int | None = None,
        bind_host: str = "",
        send_port: int | None = None,
    ):
        self.port = TEAM_PORT_BASE + team_number if port is None else port
        # On the field, robots send and receive on the same port. Tests that run two endpoints on one machine
        # point send_port at the other endpoint.
        self.send_port = self.port if send_port is None else send_port
        self.broadcast = broadcast
        self.sock = _udp_socket(self.port, bind_host)
        self.sent = 0

    def send(self, payload: bytes) -> None:
        if len(payload) > MAX_PAYLOAD:
            raise ValueError(f"team message payload {len(payload)} B exceeds {MAX_PAYLOAD} B")
        self.sock.sendto(payload, (self.broadcast, self.send_port))
        self.sent += 1

    def poll(self) -> list[tuple[bytes, str]]:
        """(payload, sender host) for all waiting messages. Includes our own broadcasts; filter by player."""
        out = []
        while True:
            try:
                data, (host, _port) = self.sock.recvfrom(2048)
            except (BlockingIOError, InterruptedError):
                return out
            out.append((data, host))

    def close(self) -> None:
        self.sock.close()
