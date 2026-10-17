"""Match the installed HTTP server's TCP reuse semantics during preflight."""
import socket


def assert_port_available(port):
    with socket.socket() as sock:
        # A stopped server's TIME_WAIT connections must not reject the next run.
        # An actively listening server still prevents this bind.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('127.0.0.1', port))
