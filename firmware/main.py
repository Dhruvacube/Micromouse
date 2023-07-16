import heapq
from collections import deque, namedtuple
from math import sqrt
from micropython import const
import machine
import gc

import uasyncio as asyncio
from machine import Pin, I2C

from libraries import fusion_async as fusion  # Using async version
from libraries import imu as _imu

imu = _imu.MPU6050(I2C(0, sda=Pin(21), scl=Pin(22)))

machine.freq(240000000)  # 240 MHz

MAZE_WIDTH = const(16)
MAZE_HEIGHT = const(16)


async def read_coro():
    await asyncio.sleep_ms(20)  # Plenty of time for mag to be ready
    return imu.accel.xyz, imu.gyro.xyz


fuse = fusion.Fusion(read_coro)


async def mem_manage():  # Necessary for long term stability
    while True:
        await asyncio.sleep_ms(100)
        gc.collect()
        gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())


asyncio.create_task(mem_manage())
asyncio.run(fuse.start())


def getIMUData():
    _IMUData = namedtuple("IMUData", "HEADING PICHT ROLL")
    return _IMUData(fuse.heading, fuse.pitch, fuse.roll)
