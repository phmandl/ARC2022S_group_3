#!/usr/bin/env python3
import numpy as np
import rospy

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

def wp_vis_msg(xy, ts, rgba=[255.0, 0.0, 0.0, 1.0], scale=0.3, dur=rospy.Duration(5.0), frame='map', log=False):
    """
    """
    if log:
        print(
            f"Publishing waypoint x = {xy[0]}, y = {xy[1]}, r = {rgba[0]}, g = {rgba[1]}, b = {rgba[2]}")
    marker = Marker()
    marker.header.stamp = ts
    marker.header.frame_id = frame
    marker.type = marker.SPHERE
    marker.pose.position.x = xy[0]
    marker.pose.position.y = xy[1]
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]
    marker.lifetime = dur
    
    return marker


def wp_map_pt_vis_msg(path, ts, rgba=[255.0, 0.0, 0.0, 1.0], scale=0.1, 
                       dur=None, frame='map'):
    """
    """
    marker_array = MarkerArray()
    for idx, ps in enumerate(path):
        marker = Marker()
        marker.header.stamp = ts
        marker.header.frame_id = frame
        marker.id = idx
        marker.type = marker.SPHERE
        marker.pose.position.x = ps[0]
        marker.pose.position.y = ps[1]
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        pt = Point()
        pt.x = marker.pose.position.x
        pt.y = marker.pose.position.y
        marker.points.append(pt)
        if dur:
            marker.lifetime = dur
        marker_array.markers.append(marker)

    return marker_array


def wp_map_line_vis_msg(path, ts, ident=0, rgba=[255.0, 0.0, 0.0, 1.0], rgbas=None, scale=0.1, 
                       dur=None, frame='map'):
    """
    """
    marker = Marker()
    marker.header.stamp = ts
    marker.header.frame_id = frame
    marker.id = ident
    marker.type = marker.LINE_STRIP
    marker.scale.x = scale
    if dur:
        marker.lifetime = dur
    for idx, ps in enumerate(path):
        pt = Point()
        pt.x = ps[0]
        pt.y = ps[1]
        marker.points.append(pt)
        color = ColorRGBA()
        if rgbas:
            color.r = rgbas[idx][0]
            color.g = rgbas[idx][1]
            color.b = rgbas[idx][2]
            color.a = rgbas[idx][3]
        else:
            color.r = rgba[0] / 255
            color.g = rgba[1] / 255
            color.b = rgba[2] / 255
            color.a = rgba[3]
        marker.colors.append(color)

    return marker


def wp_map_line_vis_with_start_msg(start, path, ts, ident=0, rgba=[255.0, 0.0, 0.0, 1.0], 
                                   rgbas=None, scale=0.1, dur=None, frame='map'):
    """
    """
    path = np.concatenate(([start], path))
    return wp_map_line_vis_msg(path, ts, ident=ident, rgba=rgba, rgbas=rgbas, scale=scale, 
                       dur=dur, frame=frame)



def wp_map_line_with_vel_vis_msg(path, vel, ts, ident=0, rgba_min=[255.0, 0.0, 0.0, 1.0], 
                                 rgba_mid=[255.0, 255.0, 0.0, 1.0], rgba_max=[0.0, 255.0, 0.0, 1.0], 
                                 vel_range=None, scale=0.1, dur=None, frame='map', wrap=False):
    """
    """
    if not vel_range:
        vmin = np.min(vel)
        vmax = np.max(vel)
        vel_range = [vmin, np.mean([vmin, vmax]), vmax]
    if wrap:
        path = np.concatenate((path, [path[0]]))
        vel = np.concatenate((vel, [vel[0]]))
    rgbas = []
    for idx, ps in enumerate(path):
        r = np.interp(vel[idx], vel_range, [rgba_min[0] / 255, rgba_mid[0] / 255, rgba_max[0] / 255])
        g = np.interp(vel[idx], vel_range, [rgba_min[1] / 255, rgba_mid[1] / 255, rgba_max[1] / 255])
        b = np.interp(vel[idx], vel_range, [rgba_min[2] / 255, rgba_mid[2] / 255, rgba_max[2] / 255])
        a = np.interp(vel[idx], vel_range, [rgba_min[3], rgba_mid[3], rgba_max[3]])
        rgbas.append([r, g, b, a])
    marker = wp_map_line_vis_msg(path, ts, ident=ident, rgbas=rgbas, scale=scale, dur=dur, frame=frame)
    return marker