#!/usr/bin/env python3
# mission_mavsdk.py
# Simple autonomous mission for PX4 SITL (Gazebo). Uses MAVSDK Python.

import asyncio
import csv
import os
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

LOG_CSV = "mission_telemetry.csv"

async def connect_drone(address="udp://:14540"):
    drone = System()
    await drone.connect(system_address=address)
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to drone!")
            break
    return drone

async def write_log_header():
    os.makedirs("logs", exist_ok=True)
    with open(os.path.join("logs", LOG_CSV), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_s", "lat", "lon", "rel_alt_m", "vx", "vy", "vz"])

async def telemetry_logger(drone):
    async for pos in drone.telemetry.position_velocity_ned():
        # position_velocity_ned provides velocity + position in NED; might not exist on older MAVSDKs
        try:
            lat = pos.position.latitude_deg
            lon = pos.position.longitude_deg
            alt = pos.position.relative_altitude_m
        except:
            # fallback to telemetry.position()
            async for p in drone.telemetry.position():
                lat = p.latitude
                lon = p.longitude
                alt = p.relative_altitude_m
                break
        # velocities - fallback if not present
        vx = getattr(pos.velocity, "north_m_s", 0.0) if hasattr(pos, "velocity") else 0.0
        vy = getattr(pos.velocity, "east_m_s", 0.0) if hasattr(pos, "velocity") else 0.0
        vz = getattr(pos.velocity, "down_m_s", 0.0) if hasattr(pos, "velocity") else 0.0

        ts = asyncio.get_event_loop().time()
        with open(os.path.join("logs", LOG_CSV), "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([f"{ts:.3f}", lat, lon, alt, vx, vy, vz])
        # print minimal telemetry
        print(f"telemetry: lat={lat:.6f}, lon={lon:.6f}, rel_alt={alt:.2f} m")
        await asyncio.sleep(0.5)

async def run_mission():
    # Connect
    drone = await connect_drone()

    # Wait until healthy to arm (helps avoid preflight fails)
    print("Waiting for health good state (global/local)...")
    async for health in drone.telemetry.health_all_ok():
        if health:
            print("Drone reports healthy.")
            break
        else:
            print("Not healthy yet, waiting 1s...")
            await asyncio.sleep(1)

    # Arm & takeoff
    print("Arming...")
    await drone.action.arm()
    print("Taking off to 5m")
    await drone.action.takeoff()
    await asyncio.sleep(6)  # let it reach altitude

    # Create mission items (local lat/lon from SITL home)
    print("Uploading mission waypoints...")
    # Example coordinates - PX4 SITL default home coords are used, these may not be meaningful in sim.
    items = [
        MissionItem(47.3977419, 8.5455937, 5, 5, True, float("nan"), float("nan"), MissionItem.CameraAction.NONE),
        MissionItem(47.3977419, 8.5465937, 5, 5, True, float("nan"), float("nan"), MissionItem.CameraAction.NONE),
        MissionItem(47.3967419, 8.5465937, 5, 5, True, float("nan"), float("nan"), MissionItem.CameraAction.NONE),
    ]
    mission_plan = MissionPlan(items)
    await drone.mission.clear_mission()
    await drone.mission.upload_mission(mission_plan)
    print("Mission uploaded. Starting mission...")
    await drone.mission.start_mission()

    # start telemetry logging in background
    telemetry_task = asyncio.create_task(telemetry_logger(drone))

    # monitor mission progress
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")
        if mission_progress.current == mission_progress.total:
            print("Mission completed.")
            break
        await asyncio.sleep(0.5)

    # Land
    print("Landing...")
    await drone.action.land()
    await asyncio.sleep(6)

    telemetry_task.cancel()
    print("Mission finished. Logs are in ./logs/mission_telemetry.csv")

if __name__ == "__main__":
    asyncio.run(write_log_header())
    asyncio.run(run_mission())

