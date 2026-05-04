Import("env")
import serial.tools.list_ports

print("\n---------------------------------------------------------")
print("[ROBOT] [AMR 2.0] SCANNING FOR CONNECTION METHODS...")

# Quét tất cả các cổng USB/COM đang cắm vào máy tính
ports = list(serial.tools.list_ports.comports())

if len(ports) == 0:
    print("[WARN]  No USB cable found. Falling back to WIFI (OTA) / Telnet...")
    env.Replace(UPLOAD_PROTOCOL="espota")
    env.Replace(UPLOAD_PORT="192.168.1.98")
    env.Replace(MONITOR_PORT="socket://192.168.1.98:23")
else:
    com_port = ports[0].device
    print(f"[OK] USB cable detected at {com_port}. Setting Serial upload/monitor mode!")
    env.Replace(UPLOAD_PROTOCOL="esptool")
    env.Replace(MONITOR_PORT=com_port)

print("---------------------------------------------------------\n")
