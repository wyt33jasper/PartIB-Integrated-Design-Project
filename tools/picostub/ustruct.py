"""Host stand-in for MicroPython's ustruct, which is CPython's struct."""

from struct import *          # noqa: F401,F403
from struct import calcsize, pack, pack_into, unpack, unpack_from  # noqa: F401
