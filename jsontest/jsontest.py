import serial
import json

ser = serial.Serial()
ser.baudrate = 38400
ser.port = "COM9"
ser.open()

dataSend = {}
dataSend["led"] = True

#ser.write(json.dumps(dataSend).encode("utf-8"))
#dataRead = ser.readline().decode("utf-8")

while True:
    cmd = input("LED on or off: ")
    if cmd == "on":
        dataSend["led"] = True
    elif cmd == "off":
        dataSend["led"] = False
    else:
        print("error")
    ser.write(json.dumps(dataSend).encode("utf-8"))
    dataRead = ser.readline().decode("utf-8")
    print(json.loads(dataRead))