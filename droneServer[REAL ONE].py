import os
import sys
import math
import json
import asyncio
import logging
import aiofiles
from grpc import aio
import DroneService_pb2
import DroneService_pb2_grpc
from pymavlink import mavutil


async def write_settings(settings: DroneService_pb2.DroneSettings) -> bool:
    try:
        async with aiofiles.open(DEFAULT_SETTINGS_FILE, 'wb') as file:
            await file.write(settings.SerializeToString())
        return True
    except Exception as e:
        print(f"Error writing settings: {e}")
        return False


async def get_valid_callsign() -> str:
    while True:
        callsign = input("Мой позывной: ")
        if callsign.strip():
            return callsign
        else:
            print("Позывной не может быть пустым. Пожалуйста, введите корректный позывной.")

async def read_settings() -> DroneService_pb2.DroneSettings:
    try:
        if not os.path.exists(DEFAULT_SETTINGS_FILE):
            callsign = await get_valid_callsign()
            settings = DroneService_pb2.DroneSettings(callsign=callsign, equipments=[])
            await write_settings(settings)
            return settings
        async with aiofiles.open(DEFAULT_SETTINGS_FILE, 'rb') as file:
            data = await file.read()
        settings = DroneService_pb2.DroneSettings()
        settings.ParseFromString(data)
        return settings
    except Exception as e:
        print(f"Error reading settings: {e}")
        return None

async def get_drone_settings():
    settings = await read_settings()
    if not settings:
        callsign = await get_valid_callsign()
        settings = DroneService_pb2.DroneSettings(callsign=callsign, equipments=[])
        await write_settings(settings)

    return settings


class DroneService(DroneService_pb2_grpc.DroneServiceServicer):
    def __init__(self, settings):
        self.heartbeat = None
        self.detailed_data = None
        self.loop = asyncio.get_running_loop()
        self.update_task = self.loop.create_task(self.update_data())
        self.settings = settings
        self.flight_mode = 0

    async def SendArmDisarm(self, request, context):
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, request.arm, 0, 0, 0, 0, 0, 0)
        msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        return DroneService_pb2.LongAnswer(result=msg.result)

    async def SendNavTakeoff(self, request, context):
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                     0, request.pitch, 0, 0, request.yaw, request.latitude, request.longitude,
                                     request.altitude)
        msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        return DroneService_pb2.LongAnswer(result=msg.result)

    async def SendReturnToLaunch(self, request, context):
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
        msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        return DroneService_pb2.LongAnswer(result=msg.result)

    async def SendChangeFlightMode(self, request, context):
        master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, request.mode)
        msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        return DroneService_pb2.LongAnswer(result=msg.result)

    async def SendFlightPlan(self, request, context):
        flightplan = request.flightPlan
        master.mav.mission_count_send(master.target_system, master.target_component, len(flightplan) + 3)
        master.mav.mission_item_send(master.target_system, master.target_component, 0, 0,
                                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        firstGeocode = flightplan[0]

        master.mav.mission_item_send(master.target_system, master.target_component,
                                     1,  # Waypoint Number
                                     0,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                     1,  # Is Current Waypoint
                                     1,  # Should Autocontinue to next wp
                                     0,  # Hold Time (ms)
                                     0,  # Radius for accept within range (meters)
                                     0,
                                     # LOITER Command: Orbit to circle (meters).  Positive clockwise, Negative counter-clockwise
                                     0,  # NAV/LOITER Command: Yaw Orientation [o to 360 degrees]
                                     firstGeocode.latitude,  # local: x position, global: latitude
                                     firstGeocode.longitude,  # local: y position, global: longitude
                                     firstGeocode.altitude)  # local: z position, global: altitude   (meters)

        counter = 2

        for geocode in flightplan:
            latitude = geocode.latitude
            longitude = geocode.longitude
            altitude = geocode.altitude
            master.mav.mission_item_send(master.target_system, master.target_component,
                                         counter,  # Waypoint Number
                                         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                         0,  # Is Current Waypoint
                                         1,  # Autocontinue to next WP
                                         request.holdTime,  # Hold Time (s)
                                         request.acceptRadius,  # Radius for accept within range (meters)
                                         0,  # Orbit to circle (meters).  Positive clockwise, Negative counter-clockwise
                                         0,  # Yaw Orientation [o to 360 degrees]
                                         latitude,  # local: x position, global: latitude
                                         longitude,  # local: y position, global: longitude
                                         altitude)  # local: z position, global: altitude   (meters)
            counter = counter + 1

        master.mav.mission_item_send(master.target_system, master.target_component, counter,
                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                     mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                     0, 0, 0, 0, 0, 0, 0, 0, 0)

        msg = master.recv_match(type="MISSION_ACK", blocking=True)
        await self.UpdateDetailedData()
        return DroneService_pb2.LongAnswer(result=msg.type)

    async def update_data(self):
        while True:
            await self.UpdateHeartbeat()
            await self.UpdateDetailedData()
            await asyncio.sleep(0.5)

    async def GetHeartbeat(self, request, context):
        return self.heartbeat

    async def UpdateHeartbeat(self):
        msg = master.recv_match(type='HEARTBEAT', blocking=True)

        type = msg.type
        if msg.type >= 17:
            type = 17

        self.heartbeat = DroneService_pb2.Heartbeat(
            type=type,
            autopilot=msg.autopilot,
            baseMode=msg.base_mode,
            systemStatus=msg.system_status,
            mavlinkVersion=msg.mavlink_version
        )
        self.flight_mode = msg.custom_mode

    async def GetDetailedData(self, request, context):
        return self.detailed_data

    async def SetName(self, request, context):
        self.settings.callsign = request.droneName
        success = await write_settings(self.settings)
        return DroneService_pb2.SettingsOperationResponse(success=success)

    async def UpdateEquipment(self, request, context):
        del self.settings.equipments[:]

        for equipment in request.equipments:
            self.settings.equipments.append(equipment)

        success = await write_settings(self.settings)
        return DroneService_pb2.SettingsOperationResponse(success=success)

    async def AddNewEquipment(self, request, context):
        try:
            self.settings.equipments.append(request.equipment)
            await write_settings(self.settings)
            return DroneService_pb2.SettingsOperationResponse(success=True)
        except AttributeError as e:
            return DroneService_pb2.SettingsOperationResponse(success=False)

    async def GetSettings(self, request, context):
        return self.settings

    async def UpdateDetailedData(self):
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

        is_armed = master.motors_armed() in mavutil.mavlink.enums['MAV_MODE_FLAG']

        self.detailed_data = DroneService_pb2.DetailedData(
            armed=is_armed,
            flightMode=self.flight_mode,
            voltageBattery=systemStatus.voltage_battery / 1000,
            batteryRemaining=systemStatus.battery_remaining,
            latitude=position.lat / 10000000,
            longitude=position.lon / 10000000,
            absoluteAltitude=position.alt / 1000,
            relativeAltitude=position.relative_alt / 1000,
            heading=position.hdg / 100,
            roll=math.degrees(attitude.roll),
            pitch=math.degrees(attitude.pitch),
            yaw=attitude.yaw,
            currentMission=missionCurrent.seq,
            missionCount=mission_count,
        )

    async def SendStartMission(self, request, context):
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_MISSION_START, 0, 1, self.detailed_data.missionCount, 0, 0,
                                     0, 0, 0)
        msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        return DroneService_pb2.LongAnswer(result=msg.result)

    async def SendClearAllMissions(self, request, context):
        master.mav.mission_clear_all_send(master.target_system, master.target_component)
        msg = master.recv_match(type="MISSION_ACK", blocking=True)
        return DroneService_pb2.LongAnswer(result=msg.type)


async def server_start(port):
    server = aio.server()
    DroneService_pb2_grpc.add_DroneServiceServicer_to_server(DroneService(await get_drone_settings()), server)
    server.add_insecure_port('[::]:{0}'.format(port))
    await server.start()
    print("Сервер запущен. Порт: {0}.".format(port))
    await server.wait_for_termination()


DEFAULT_SETTINGS_FILE = "drone_settings.bin"
SERIAL_PORT = 'COM3'
BAUDRATE = 57600
#Альтернативный вариант подключения через интернет:
#host = '192.168.1.54'
#port = '1450'
#port = sys.argv[1]
#out_port = int(sys.argv[2])
if len(sys.argv) >= 4:
    DEFAULT_SETTINGS_FILE = sys.argv[3]

#port = 5760
out_port = 1604

#master = mavutil.mavlink_connection('udp:{0}:{1}'.format(host, port))

print("Подключение к полётному контроллеру...")
master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUDRATE)
master.wait_heartbeat()

print("Подключение установлено!")
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                    2, 1)
print("Запускаю сервер...")
logging.basicConfig(level=logging.INFO)

asyncio.run(server_start(out_port))
