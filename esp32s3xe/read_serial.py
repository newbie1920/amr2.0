import sys
import time
import serial

try:
    print("Opening COM4 at 115200...")
    ser = serial.Serial('COM4', 115200, timeout=1)
    
    print("Resetting ESP32 via RTS/DTR...")
    ser.dtr = False
    ser.rts = True
    time.sleep(0.1)
    ser.dtr = False
    ser.rts = False
    
    print("Listening to data...")
    start_time = time.time()
    
    while time.time() - start_time < 15:
        data = ser.read_all()
        if len(data) > 0:
            sys.stdout.write(data.decode(errors='ignore'))
            sys.stdout.flush()
        time.sleep(0.1)
        
    print("\n[END] 15s monitor over.")
    ser.close()
except Exception as e:
    print("Error: " + str(e))
