#!/usr/bin/env python3
"""
gantry_link.py - The wire protocol between a remote client and the gantry
server that owns the controller's serial port.

Shared verbatim by both ends. It is deliberately pure: it turns objects into
lines and lines back into objects, and performs no I/O of its own beyond the
LineReader framing helper, which is given a recv callable. That makes the whole
protocol testable without a socket, a Pi, or Bluetooth.

WHY A COMMAND PROTOCOL AND NOT A BYTE TUNNEL
  Two properties of the controller make a transparent serial tunnel impossible:
    * DB9 pin 7 is a hardware RESET driven by the host RTS line, and neither
      Bluetooth RFCOMM nor TCP carries RS-232 control lines. A remote client
      could never reset the board.
    * Reply framing is timing-based -- a reply is "complete" once no byte has
      arrived for REPLY_QUIET_S. Bluetooth aggregates and jitters packets, so
      that logic only works next to the UART.
  Both therefore execute on the server. This protocol carries whole commands
  and whole replies.

FORMAT
  One request or response per line, '\\n' terminated, ASCII.

    -> HELLO  <id> <proto> <client name>
    -> CMD    <id> <wait-ms> <controller command>
    -> RESET  <id>
    -> BANNER <id>
    -> ARM    <id> <acknowledgement text>
    -> DISARM <id>
    -> STOP   <id>
    -> PING   <id>

    <- REP <id> OK  [payload]
    <- REP <id> ERR <code> <message>
    <- EVT <name> [payload]

  Request ids are client-assigned and echoed back, so a priority STOP may be
  answered before a command that was sent earlier.

ESCAPING
  Controller replies contain CR and LF (the reset banner is multi-line), so the
  free-text field of every line is escaped: backslash -> '\\\\', CR -> '\\r',
  LF -> '\\n'. Only the trailing field needs it, which keeps the common case
  readable in a log:

      CMD 7 200 VELX 5000
      REP 7 OK
      REP 8 OK X+1000
      REP 9 ERR DISARMED MOVRX needs an armed session

  Base64 was considered and rejected: it makes logs unreadable, and the logs
  are how this gets debugged when it misbehaves in a lab at the far end of a
  Bluetooth link.
"""

from __future__ import annotations

from dataclasses import dataclass, field

#: Bumped whenever the meaning of a line changes. A mismatch is a hard failure,
#: never a warning: a client that misunderstands this protocol must not be
#: allowed to move metal.
PROTOCOL_VERSION = 1

#: Refuse absurd lines rather than buffering forever. The longest legitimate
#: line is a reset banner, a few hundred bytes.
MAX_LINE_BYTES = 8192

# ------------------------------------------------------------- reason codes
# Machine-readable, so the client can say precisely what happened rather than
# showing a sentence it cannot reason about.
ERR_BUSY = "BUSY"  # another client already owns the controller
ERR_DISARMED = "DISARMED"  # session has not been armed; see ARM
ERR_REFUSED_DESTRUCTIVE = "REFUSED_DESTRUCTIVE"  # RUN/NEW/SAVE/CONT
ERR_REFUSED_HOME = "REFUSED_HOME"  # no home switches exist on this machine
ERR_NO_PORT = "NO_PORT"  # the server has no working serial port
ERR_BAD_REQUEST = "BAD_REQUEST"  # malformed or out-of-range arguments
ERR_PROTO = "PROTO"  # protocol version mismatch or bad framing
ERR_ABORTED = "ABORTED"  # discarded because a priority STOP overtook it
ERR_TIMEOUT = "TIMEOUT"  # the controller did not answer in time
ERR_INTERNAL = "INTERNAL"  # bug on the server; see its log

# ------------------------------------------------------------------- events
EVT_STOPPED = "stopped"  # the server issued STOPALL on its own initiative
EVT_WATCHDOG = "watchdog"  # heartbeat lapsed; payload says what was done
EVT_GOODBYE = "goodbye"  # the server is shutting down

# --------------------------------------------------------------------- verbs
#: verb -> (number of fixed space-separated arguments, takes trailing free text)
#:
#: The trailing field is "rest of line", which is what keeps a command such as
#: "VELX 5000" readable instead of forcing spaces to be escaped as well.
_VERB_SHAPES: dict[str, tuple[int, bool]] = {
    "HELLO": (1, True),  # <proto> <client name>
    "CMD": (1, True),  # <wait-ms> <controller command>
    "RESET": (0, False),
    "BANNER": (0, False),  # the start-up banner the server actually captured
    "ARM": (0, True),  # <acknowledgement text>
    "DISARM": (0, False),
    "STOP": (0, False),
    "PING": (0, False),
}

VERBS = tuple(_VERB_SHAPES)

#: Verbs the server honours before HELLO has established the protocol version.
#: STOP is here deliberately: stopping must never depend on a handshake.
PRE_HELLO_VERBS = ("HELLO", "STOP")


#: The exact phrase ARM must carry. A fixed sentence rather than a free-text
#: "yes" makes arming a deliberate act that a stray keystroke cannot produce,
#: and it states what the operator is actually confirming. The server compares
#: it verbatim and logs it.
ARM_ACKNOWLEDGEMENT = "workspace clear; power cutoff reachable"

#: Request id used for a response that answers no particular request -- a line
#: that could not be parsed, or a connection refused before any request.
NO_REQUEST_ID = 0


class ProtocolError(Exception):
    """A line could not be understood. Always fatal for that line, and for the
    connection when it happens during the handshake."""


# ------------------------------------------------------------------ escaping
def escape(text: str) -> str:
    """Make free text safe to put at the end of a protocol line."""
    return text.replace("\\", "\\\\").replace("\r", "\\r").replace("\n", "\\n")


def unescape(text: str) -> str:
    """Inverse of escape(). Left-to-right so that '\\\\n' is a literal
    backslash followed by 'n', not a newline."""
    out: list[str] = []
    i = 0
    while i < len(text):
        ch = text[i]
        if ch == "\\" and i + 1 < len(text):
            nxt = text[i + 1]
            if nxt == "\\":
                out.append("\\")
                i += 2
                continue
            if nxt == "r":
                out.append("\r")
                i += 2
                continue
            if nxt == "n":
                out.append("\n")
                i += 2
                continue
        out.append(ch)
        i += 1
    return "".join(out)


# ------------------------------------------------------------------ messages
@dataclass(frozen=True)
class Request:
    """One client -> server line."""

    verb: str
    req_id: int
    args: tuple[str, ...] = ()

    def encode(self) -> str:
        return encode_request(self)

    # Convenience accessors, so callers do not index a tuple by hand.
    @property
    def text(self) -> str:
        """The trailing free-text argument, or '' if the verb has none."""
        fixed, has_text = _VERB_SHAPES[self.verb]
        return self.args[fixed] if has_text and len(self.args) > fixed else ""


@dataclass(frozen=True)
class Response:
    """One server -> client line answering a request."""

    req_id: int
    ok: bool
    payload: str = ""
    code: str = ""

    def encode(self) -> str:
        return encode_response(self)


@dataclass(frozen=True)
class Event:
    """One unsolicited server -> client line."""

    name: str
    payload: str = ""

    def encode(self) -> str:
        return encode_event(self)


# ------------------------------------------------------------------ encoding
def _check_id(req_id: int) -> int:
    if not isinstance(req_id, int) or isinstance(req_id, bool):
        raise ProtocolError(f"request id must be an int, got {req_id!r}")
    if not 0 <= req_id <= 2**31 - 1:
        raise ProtocolError(f"request id {req_id} out of range")
    return req_id


def encode_request(request: Request) -> str:
    verb = request.verb.upper()
    if verb not in _VERB_SHAPES:
        raise ProtocolError(f"unknown verb {request.verb!r}")
    fixed, has_text = _VERB_SHAPES[verb]
    expected = fixed + (1 if has_text else 0)
    if len(request.args) != expected:
        raise ProtocolError(f"{verb} takes {expected} argument(s), got {len(request.args)}")
    parts = [verb, str(_check_id(request.req_id))]
    for arg in request.args[:fixed]:
        if " " in arg or "\n" in arg or "\r" in arg:
            raise ProtocolError(f"{verb}: fixed argument {arg!r} may not contain whitespace")
        parts.append(arg)
    if has_text:
        parts.append(escape(request.args[fixed]))
    return " ".join(parts)


def encode_response(response: Response) -> str:
    parts = ["REP", str(_check_id(response.req_id))]
    if response.ok:
        parts.append("OK")
        if response.payload:
            parts.append(escape(response.payload))
    else:
        code = response.code or ERR_INTERNAL
        if " " in code:
            raise ProtocolError(f"reason code {code!r} may not contain a space")
        parts += ["ERR", code]
        if response.payload:
            parts.append(escape(response.payload))
    return " ".join(parts)


def encode_event(event: Event) -> str:
    if " " in event.name or not event.name:
        raise ProtocolError(f"event name {event.name!r} must be one non-empty word")
    parts = ["EVT", event.name]
    if event.payload:
        parts.append(escape(event.payload))
    return " ".join(parts)


# ------------------------------------------------------------------ decoding
def decode_request(line: str) -> Request:
    """Parse one client -> server line. Raises ProtocolError on anything
    malformed -- the server answers ERR BAD_REQUEST rather than guessing."""
    stripped = line.strip()
    if not stripped:
        raise ProtocolError("empty line")
    head, _, rest = stripped.partition(" ")
    verb = head.upper()
    if verb not in _VERB_SHAPES:
        raise ProtocolError(f"unknown verb {head!r}")
    fixed, has_text = _VERB_SHAPES[verb]

    # The line is: id, then `fixed` more words, then optionally the rest.
    want_words = 1 + fixed
    pieces = rest.split(" ")
    if pieces == [""] or len(pieces) < want_words:
        raise ProtocolError(f"{verb}: expected at least {want_words} argument(s)")
    try:
        req_id = int(pieces[0])
    except ValueError as e:
        raise ProtocolError(f"{verb}: request id {pieces[0]!r} is not an integer") from e
    _check_id(req_id)

    args = tuple(pieces[1:want_words])
    if has_text:
        trailing = " ".join(pieces[want_words:])
        args += (unescape(trailing),)
    elif len(pieces) > want_words:
        extra = " ".join(pieces[want_words:])
        raise ProtocolError(f"{verb}: takes no trailing text, got {extra!r}")
    return Request(verb, req_id, args)


def decode_response(line: str) -> Response:
    """Parse one server -> client REP line."""
    stripped = line.strip()
    pieces = stripped.split(" ")
    if not pieces or pieces[0].upper() != "REP":
        raise ProtocolError(f"not a response line: {line!r}")
    if len(pieces) < 3:
        raise ProtocolError(f"truncated response: {line!r}")
    try:
        req_id = int(pieces[1])
    except ValueError as e:
        raise ProtocolError(f"response id {pieces[1]!r} is not an integer") from e
    status = pieces[2].upper()
    if status == "OK":
        return Response(req_id, True, unescape(" ".join(pieces[3:])))
    if status == "ERR":
        if len(pieces) < 4:
            raise ProtocolError(f"ERR response without a reason code: {line!r}")
        return Response(req_id, False, unescape(" ".join(pieces[4:])), pieces[3])
    raise ProtocolError(f"response status must be OK or ERR, got {pieces[2]!r}")


def decode_event(line: str) -> Event:
    """Parse one server -> client EVT line."""
    pieces = line.strip().split(" ")
    if not pieces or pieces[0].upper() != "EVT" or len(pieces) < 2:
        raise ProtocolError(f"not an event line: {line!r}")
    return Event(pieces[1], unescape(" ".join(pieces[2:])))


def is_event(line: str) -> bool:
    """True if this line is unsolicited and carries no request id."""
    return line.strip()[:4].upper() == "EVT "


# ------------------------------------------------------------------- framing
@dataclass
class LineReader:
    """Turns a stream of chunks into whole lines.

    Given any object with `recv(n) -> bytes` (a socket, or a fake in a test).
    Kept here rather than in the client or the server so that both ends frame
    identically -- a framing mismatch between the two would be a miserable bug
    to chase across a Bluetooth link.
    """

    recv: object  # callable(int) -> bytes
    max_line: int = MAX_LINE_BYTES
    _buf: bytearray = field(default_factory=bytearray, repr=False)
    _eof: bool = False

    def pending(self) -> bool:
        """True if a complete line is already buffered, with no read needed."""
        return b"\n" in self._buf

    def feed(self, chunk: bytes) -> None:
        """Add bytes received elsewhere (used by tests and by a select loop)."""
        if not chunk:
            self._eof = True
            return
        self._buf.extend(chunk)
        if len(self._buf) > self.max_line:
            raise ProtocolError(f"line exceeded {self.max_line} bytes; dropping the connection")

    def take(self) -> str | None:
        """Pop one buffered line, or None if no complete line is buffered."""
        index = self._buf.find(b"\n")
        if index < 0:
            return None
        line = bytes(self._buf[:index])
        del self._buf[: index + 1]
        return line.decode("latin-1", "replace").rstrip("\r")

    def read_line(self, chunk_size: int = 512) -> str | None:
        """Return the next whole line, reading as needed. None at end of
        stream. Blocking behaviour is entirely the underlying object's.

        ⚠️ An unterminated remainder at end of stream is DISCARDED, never
        returned. A line that was cut off mid-transmission can decode into a
        perfectly valid but different message -- "CMD 5 200 MOVRX" truncated to
        "CMD 5 200 MOVR" is still a parseable command. On a machine with no
        limit switches and no encoders, executing a truncated command is
        exactly the silent-failure class that hurts here. If the newline did
        not arrive, the sender did not finish, and we do not guess.
        """
        while True:
            line = self.take()
            if line is not None:
                return line
            if self._eof:
                self._buf.clear()  # drop any truncated remainder, see above
                return None
            self.feed(self.recv(chunk_size))  # type: ignore[operator]


def encode_line(message: Request | Response | Event) -> bytes:
    """Encode any message and terminate it, ready for the wire."""
    return (message.encode() + "\n").encode("latin-1")
