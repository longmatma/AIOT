GPS LIVE LOG PATCH - rBS_E22_update

Muc tieu:
- Khong can cam day Serial vao tung SU/DU de xem GPS.
- SU1/SU2/DU1/DU2 van gui packet GPS LoRa 0x17/0x18 nhu cu.
- rBS in ngay GPS nhan duoc tren terminal.
- Khong thay doi packet 44 byte, CSV, scheduler, LoRa hay Digital Twin.

Node IDs:
SU1 = 0x01
DU1 = 0x02
rBS = 0x03
SU2 = 0x04
DU2 = 0x05

Log mau:
[rBS GPS LIVE] SU1 | GPS_PERIODIC | REF=12 | FIX | LAT=20.9803020 | LON=105.7952470 | ALT=9.90m | SPEED=0.08m/s | SAT=5 | HDOP=1.20 | AGE=420ms | RSSI=-69.0dBm | SNR=8.8dB

Neu GPS chua fix:
[rBS GPS LIVE] SU2 | GPS_PERIODIC | REF=13 | NO_FIX | LAT=0.0000000 | LON=0.0000000 | ...

File da sua:
rbs_gateway.py -> ham _v2c1_handle_telemetry()
