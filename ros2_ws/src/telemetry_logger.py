#!/usr/bin/env python3
import asyncio, csv, os
from mavsdk import System

LOG_CSV = "telemetry.csv"

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    os.makedirs("logs", exist_ok=True)
    with open(os.path.join("logs", LOG_CSV), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ts", "lat", "lon", "alt"])
    async for pos in drone.telemetry.position():
        with open(os.path.join("logs", LOG_CSV), "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([asyncio.get_event_loop().time(), pos.latitude, pos.longitude, pos.relative_altitude_m])
        print(pos)
        await asyncio.sleep(0.5)

if __name__=="__main__":
    asyncio.run(run())

