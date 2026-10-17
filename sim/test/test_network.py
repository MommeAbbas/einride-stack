import socket
import pytest
from sim.network import assert_port_available


def test_active_listener_is_rejected():
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        server.bind(('127.0.0.1',0));server.listen()
        with pytest.raises(OSError):assert_port_available(server.getsockname()[1])


def test_recently_closed_server_can_restart():
    server=socket.socket();server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    server.bind(('127.0.0.1',0));port=server.getsockname()[1];server.listen()
    client=socket.create_connection(('127.0.0.1',port));connection,_=server.accept()
    connection.close();client.recv(1);client.close();server.close()
    # Reproduce the old preflight's false EADDRINUSE while TCP is in TIME_WAIT.
    with socket.socket() as old_probe:
        with pytest.raises(OSError):old_probe.bind(('127.0.0.1',port))
    assert_port_available(port)
