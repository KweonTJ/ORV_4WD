import os
import pty
import pytest
from orv_4wd.transport import SerialTransport
from orv_4wd.protocol import frame, Kind


def test_real_serial_transport_over_pty_and_disconnect():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    serial = SerialTransport(path)
    try:
        with pytest.raises(BlockingIOError):
            SerialTransport(path)
        packet = frame(Kind.STOP, 123)
        serial.write(packet)
        assert os.read(master, 100) == packet
        os.write(master, packet)
        import select
        assert select.select([serial.fd], [], [], 0.5)[0]
        assert serial.read() == packet
        os.close(master); master = None
        with pytest.raises(OSError): serial.read()
    finally:
        serial.close()
        if master is not None: os.close(master)
        os.close(slave)
