#!/usr/bin/env python

from utils import constrain_value
from math import cos,radians

class Waypoint(object):
    """
        lat     : latitude in degree
        lng     : longitude in degree
        alt     : altitude in meter
        rel_alt : relative altitude in meter from home
    """
    LOCATION_SCALING_FACTOR = 111318.84502145034

    def __init__(self, lat=None, lng=None, alt=None, frame=0,seq=None, rel_alt=0, mavdict=None):
        self.lat = lat
        self.lng = lng
        self.alt = alt

        self.frame = frame
        self.seq = seq

        self.rel_alt = rel_alt

        if mavdict:
            self.frame = mavdict["frame"]
            self.lat = mavdict["x"]
            self.lng = mavdict["y"]
            self.seq = mavdict["seq"]

            if self.frame == 3:
                self.rel_alt = mavdict["z"]
            elif self.frame == 0:
                self.alt = mavdict["z"]


    def __str__(self):
        print_values = [(self.lat,"lat"), (self.lng,"lng"), (self.rel_alt,"rel_alt"), (self.alt,"alt"),(self.frame,"frame")]

        return "\n".join( "%s : %6.10f"%(key,val) for val,key in print_values if val ) + "\n"+"-"*20

    """
        return x,y position relate other
    """
    def location_diff(self, other):
        if self.frame == 0:
            z = self.alt - other.alt
        elif self.frame ==3:
            z = self.rel_alt

        return (
                (self.lat - other.lat) * self.LOCATION_SCALING_FACTOR,
                (self.lng - other.lng) * self.LOCATION_SCALING_FACTOR * self.longitude_scale(other),
                z,
        )

    def offset_from_meter_lat(self,x):
        '''
        Calculate Lat from X,Y with respect to Home Position
        Assume x,y = 0,0 is Home Location
        '''
        lat_offset_from_x  = x / self.LOCATION_SCALING_FACTOR
        return lat_offset_from_x

    def offset_from_meter_long(self,y):
        '''
        Calculate Long from X,Y with respect to Home Position
        Assume x,y = 0,0 is Home Location
        '''
        long_offset_from_y = y / self.LOCATION_SCALING_FACTOR
        return long_offset_from_y

    @staticmethod
    def longitude_scale(loc):
        scale = cos(radians(loc.lat))
        return constrain_value(scale, 0.01, 1.0)

    #     Vector2f location_diff(const struct Location &loc1, const struct Location &loc2)
    # {
    #     return Vector2f((loc2.lat - loc1.lat) * LOCATION_SCALING_FACTOR,
    #                     (loc2.lng - loc1.lng) * LOCATION_SCALING_FACTOR * longitude_scale(loc1));
    # }