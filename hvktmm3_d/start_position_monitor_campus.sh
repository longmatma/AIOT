#!/bin/bash
set -e
cd /home/pi5/rBS_AIOT
PY="/home/pi5/rBS_AIOT/.venv/bin/python3"
[ -x "$PY" ] || PY="python3"
export RBS_GPS_CSV="/home/pi5/rBS_AIOT/rbs_lien_ket_dinh_ky_v11.csv"
export RBS_SU_DU_CHANNEL_CSV="/home/pi5/rBS_AIOT/rbs_kenh_su_du.csv"
export RBS_CAMPUS_BG="/home/pi5/rBS_AIOT/campus_live_background.png"
export RBS_CAMPUS_MAP_CONFIG="/home/pi5/rBS_AIOT/campus_live_map_config.json"
export RBS_UI_FULLSCREEN="${RBS_UI_FULLSCREEN:-1}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-wayland}"
export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/1000/bus}"
exec "$PY" /home/pi5/rBS_AIOT/position_monitor_campus.py
