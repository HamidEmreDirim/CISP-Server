# terminal_pty.py (GÜNCEL)
import os, pty, select, threading, fcntl, termios, struct, errno, signal
from flask import request
from flask_socketio import SocketIO

_sessions = {}      # sid → fd
_pids      = {}      # sid → child pid
_lock      = threading.Lock()

# ---------- Yardımcı ----------
def _close_session(sid, socketio: SocketIO, send_msg=True):
    """FD ve PID'yi tek seferlik güvenli kapat."""
    with _lock:
        fd  = _sessions.pop(sid, None)
        pid = _pids.pop(sid,  None)

    if fd is not None:
        try:
            os.close(fd)
        except OSError:
            pass

    if pid is not None:
        try:
            os.kill(pid, signal.SIGHUP)
        except OSError:
            pass

    if send_msg:
        socketio.emit("terminalOutput", "\r\n[Session closed]\r\n", to=sid)

# ---------- Kayıt ----------
def register_terminal_events(socketio: SocketIO):
    @socketio.on("startTerminal")
    def _start():
        if request.sid in _sessions:   # zaten açık
            return
        pid, fd = pty.fork()
        if pid == 0:                   # Child → bash
            os.execvp("bash", ["bash"])

        with _lock:
            _sessions[request.sid] = fd
            _pids[request.sid]      = pid

        th = threading.Thread(
            target=_reader, args=(socketio, request.sid, fd), daemon=True
        )
        th.start()

    def _reader(socketio: SocketIO, sid: str, fd: int):
        try:
            while True:
                r, _, _ = select.select([fd], [], [], 0.1)
                if r:
                    try:
                        data = os.read(fd, 1024)
                    except OSError as e:
                        # FD kapanmış; sessizce çık
                        if e.errno in (errno.EIO, errno.EBADF):
                            break
                        raise
                    if not data:
                        break
                    socketio.emit("terminalOutput",
                                  data.decode("utf-8", "ignore"), to=sid)
        finally:
            _close_session(sid, socketio, send_msg=False)

    @socketio.on("terminalInput")
    def _input(data):
        fd = _sessions.get(request.sid)
        if fd:
            try:
                os.write(fd, data.encode())
            except OSError:
                _close_session(request.sid, socketio)

    @socketio.on("terminalResize")
    def _resize(size):
        fd = _sessions.get(request.sid)
        if not fd:
            return
        rows, cols = size.get("rows", 24), size.get("cols", 80)
        try:
            fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))
        except OSError:
            _close_session(request.sid, socketio)

    @socketio.on("disconnect")
    def _disc():
        _close_session(request.sid, socketio, send_msg=False)

