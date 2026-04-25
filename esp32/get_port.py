import serial.tools.list_ports
import sys

ports = list(serial.tools.list_ports.comports())
if ports:
    sys.stdout.write(ports[0].device)
else:
    sys.stdout.write("socket://192.168.1.117:23")
