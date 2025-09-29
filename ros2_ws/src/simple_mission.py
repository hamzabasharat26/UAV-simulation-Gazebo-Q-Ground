import asyncio
from mavsdk import System

async def run():
    # Connect to PX4 SITL (default UDP port)
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Drone discovered")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("✅ Global position OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(8)

    print("-- Flying forward (finish line)")
    # Move forward in local NED (North, East, Down) frame
    await drone.action.set_velocity_body(2.0, 0.0, 0.0, 0.0)  # 2 m/s forward
    await asyncio.sleep(10)  # fly forward for 10 sec

    print("-- Stopping at finish line")
    await drone.action.hold()
    await asyncio.sleep(2)

    print("-- Landing")
    await drone.action.land()
    await asyncio.sleep(8)

    print("✅ Mission complete")

if __name__ == "__main__":
    asyncio.run(run())

