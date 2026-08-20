"""Small geo-math helpers for Shadow Mode's follow-point calculation. Not a
navigation algorithm -- that's still entirely ArduPilot's own GUIDED-mode
controller (see rover_control.py's goto()). All this does is compute WHERE
the follow point is; the Cube does everything about how to actually get
there.
"""
import math


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = math.sin(d_lat / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(d_lon / 2) ** 2
    return r * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def destination_point(lat, lon, bearing_deg, distance_m):
    """Point `distance_m` away from (lat, lon) in direction `bearing_deg`
    (0=north, 90=east, clockwise). Standard spherical-earth offset formula."""
    r = 6371000.0
    p1 = math.radians(lat)
    l1 = math.radians(lon)
    brng = math.radians(bearing_deg)
    d_r = distance_m / r

    p2 = math.asin(math.sin(p1) * math.cos(d_r) + math.cos(p1) * math.sin(d_r) * math.cos(brng))
    l2 = l1 + math.atan2(
        math.sin(brng) * math.sin(d_r) * math.cos(p1),
        math.cos(d_r) - math.sin(p1) * math.sin(p2),
    )
    return math.degrees(p2), math.degrees(l2)
