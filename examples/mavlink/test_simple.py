from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:127.0.0.1:14541')
master.wait_heartbeat()

while True:
    # msg = master.recv_match(blocking=True)
    msg = master.recv_match(type=['HEARTBEAT', 'LOCAL_POSITION_NED'], blocking=True)
    if not msg:
        continue

    msg_type = msg.get_type()
    

    # Пропускаем служебные пустые
    if msg_type == "BAD_DATA":
        continue

    print(f"\n=== {msg_type} ===")
    print(msg.to_dict())