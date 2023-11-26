import grpc
import DroneService_pb2
import DroneService_pb2_grpc
from concurrent import futures
from pymavlink import mavutil
import time

class DroneService(DroneService_pb2_grpc.DroneServiceServicer):
    def __init__(self):
        self.heartbeat = None
        self.detailed_data = None
    def send_command_long(self, command_type, *args):
        command_mapping = {
            "SendArmDisarm": mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            "SendNavTakeoff": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            "SendReturnToLaunch": mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        }
        method_name = command_type.split("Send")[1]

        command = command_mapping[command_type]
        master.mav.command_long_send(master.target_system, master.target_component, command, 0, *args)

        msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        return DroneService_pb2.LongAnswer(result=msg.result)

    def SendArmDisarm(self, request, context):
        return self.send_command_long("SendArmDisarm", request.arm, 0, 0, 0, 0, 0, 0)

    def SendNavTakeoff(self, request, context):
        return self.send_command_long("SendNavTakeoff", request.pitch, 0, 0, request.yaw, request.latitude, request.longitude, request.altitude)

    def SendReturnToLaunch(self, request, context):
        return self.send_command_long("SendReturnToLaunch", 0, 0, 0, 0, 0, 0, 0)
    
    def SendChangeFlightMode(self, request, context):
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, request.mode)
        msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        return DroneService_pb2.LongAnswer(result=msg.result)
    def SendFlightPlan(self, request, context):
        flightplan = request.flightPlan
        master.mav.mission_count_send(master.target_system, master.target_component, len(flightplan)+2)
        master.mav.mission_item_send(master.target_system, master.target_component, 0, 0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 0, 0, 0) 
        
        counter = 1     
        for geocode in flightplan:
            latitude = geocode.latitude
            longitude = geocode.longitude
            altitude = geocode.altitude
            master.mav.mission_item_send(master.target_system, master.target_component, 
                             counter,     # Waypoint Number
                             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                             0,     # Is Current Waypoint
                             1,     # Autocontinue to next WP
                             request.holdTime,  # Hold Time (s)
                             request.acceptRadius,     # Radius for accept within range (meters)
                             0,     # Orbit to circle (meters).  Positive clockwise, Negative counter-clockwise
                             0,     # Yaw Orientation [o to 360 degrees]
                             latitude,     # local: x position, global: latitude
                             longitude,     # local: y position, global: longitude
                             altitude)    # local: z position, global: altitude   (meters)
            counter = counter + 1
            
        master.mav.mission_item_send(master.target_system, master.target_component, counter,
                             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                             mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                             0, 0, 0, 0, 0, 0, 0, 0, 0)
        
        msg = master.recv_match(type="MISSION_ACK", blocking=True)
        self.UpdateDetailedData()
        return DroneService_pb2.LongAnswer(result=msg.type)
    def GetHeartbeat(self, request, context):
        self.UpdateHeartbeat()
        return self.heartbeat
    def UpdateHeartbeat(self):
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        self.heartbeat = DroneService_pb2.Heartbeat(
            type=msg.type,
            autopilot=msg.autopilot,
            baseMode=msg.base_mode,
            customMode=msg.custom_mode,
            systemStatus=msg.system_status,
            mavlinkVersion=msg.mavlink_version
        )
    def GetDetailedData(self, request, context):
        self.UpdateDetailedData()
        return self.detailed_data
    def UpdateDetailedData(self):
        systemStatus = master.recv_match(type='SYS_STATUS', blocking=True)
        missionCurrent = master.recv_match(type='MISSION_CURRENT', blocking=True)
        master.waypoint_request_list_send()
        missionCount = master.recv_match(type='MISSION_COUNT', blocking=True)
        position = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        attitude = master.recv_match(type='ATTITUDE', blocking=True)

        # TODO: Проверить, верен ли этот блок кода (P.S. вроде верен)
        mission_count = missionCount.count
        if (mission_count > 0):
            mission_count = mission_count - 1

        self.detailed_data = DroneService_pb2.DetailedData(
            voltageBattery=systemStatus.voltage_battery,
            batteryRemaining=systemStatus.battery_remaining,
            latitude=position.lat,
            longitude=position.lon,
            absoluteAltitude=position.alt,
            relativeAltitude=position.relative_alt,
            heading=position.hdg,
            roll=attitude.roll,
            pitch=attitude.pitch,
            yaw=attitude.yaw,
            rollSpeed=attitude.rollspeed,
            pitchSpeed=attitude.pitchspeed,
            yawSpeed=attitude.yawspeed,
            currentMission=missionCurrent.seq,
            missionCount=mission_count,
            vx=position.vx,
            vy=position.vy,
            vz=position.vz
        )
    def SendStartMission(self, request, context):
        master.mav.command_long_send(master.target_system, master.target_component,
                             mavutil.mavlink.MAV_CMD_MISSION_START, 0, 1, self.detailed_data.missionCount, 0, 0, 0, 0, 0)
        msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        return DroneService_pb2.LongAnswer(result=msg.result)

    def SendClearAllMissions(self, request, context):
        master.mav.mission_clear_all_send(master.target_system, master.target_component)
        msg = master.recv_match(type="MISSION_ACK", blocking=True)
        return DroneService_pb2.LongAnswer(result=msg.type)
    
def server_start(port):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    DroneService_pb2_grpc.add_DroneServiceServicer_to_server(DroneService(), server)
    server.add_insecure_port('[::]:{0}'.format(port))
    server.start()
    print("Сервер запущен. Порт: {0}.".format(port))
    try:
        while True:
            time.sleep(86400)
    except KeyboardInterrupt:
        server.stop(0)

SERIAL_PORT = 'COM3'
BAUDRATE = 57600
#Альтернативный вариант подключения через интернет:
host = 'localhost'
port = 5760
master = mavutil.mavlink_connection('tcp:{0}:{1}'.format(host, port))

print("Подключаюсь к полётнику...")
#master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUDRATE)
master.wait_heartbeat()

print("Подключение установлено!")
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
print("Запускаю сервер...") 
server_start(1604)