#!/usr/bin/env python3
"""Dev tool (not installed by CMake): height of base_link above ground at the stand pose.

That height is the z offset of base_footprint_joint (base_footprint -> base_link), so that
base_footprint sits on the ground when the hexapod stands. The value is the
lowest vertex of the tibia CAD mesh (what RViz draws / the real foot); the collision foot
sphere height (what Gazebo touches the floor with) is printed as a consistency check. Re-run after changing the stand
pose in config/rest_pose.yaml, the tibia collision box, or any geometry in
mechanical_params.xacro, then paste the result into stand.base_height in rest_pose.yaml.

Geometry is read from mechanical_params.xacro (numeric properties) and meshes/tibia.stl, so
there is no second copy of the CAD numbers. Leg 1 is used; the result is identical for all
legs (coxa rotates about the vertical axis).

    python3 stand_height.py [--femur RAD] [--tibia RAD]   # defaults: stand pose from yaml
"""
import argparse
import os
import re
import struct

import numpy as np
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def props():
    txt = open(os.path.join(PKG, 'urdf', 'mechanical_params.xacro')).read()
    txt = re.sub(r'<!--.*?-->', '', txt, flags=re.S)  # ignore commented-out properties
    out = {}
    for name, val in re.findall(r'<xacro:property\s+name="(\w+)"\s+value="([^"]*)"', txt):
        try:
            out[name] = np.array([float(x) for x in val.split()])
        except ValueError:
            pass  # ${...} expressions / package:// paths
    return out


def load_stl(fn):
    b = open(fn, 'rb').read()
    if b[:5] == b'solid' and b'facet' in b[:400]:
        return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', b), dtype=float)
    n = struct.unpack('<I', b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype(
        [('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].reshape(-1, 3).astype(float)


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def T(R, p):
    t = np.eye(4)
    t[:3, :3] = R
    t[:3, 3] = p
    return t


def apply(Tm, pts):
    return (Tm @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]


def main():
    rest = yaml.safe_load(open(os.path.join(PKG, 'config', 'rest_pose.yaml')))
    ap = argparse.ArgumentParser()
    ap.add_argument('--femur', type=float, default=rest['stand']['femur'])
    ap.add_argument('--tibia', type=float, default=rest['stand']['tibia'])
    a = ap.parse_args()

    P = props()
    q1 = rest['stand'].get('coxa', rest['coxa'][0])
    T1 = T(Rz(P['leg1_yaw'][0]), P['leg1_xyz']) @ T(Rz(q1), [0, 0, 0])
    T2 = T1 @ T(np.eye(3), P['j2_xyz']) @ T(Ry(a.femur), [0, 0, 0])
    T3 = T2 @ T(Rz(P['j3_rpy'][2]), P['j3_xyz']) @ T(Ry(a.tibia), [0, 0, 0])

    # collision foot sphere (what Gazebo contacts the ground with)
    centre = apply(T3, [P['tibia_foot_xyz']])[0]
    sphere_low = centre[2] - P['tibia_foot_radius'][0]

    # visual mesh (CAD geometry)
    mesh = load_stl(os.path.join(PKG, 'meshes', 'tibia.stl')) * 0.001
    mesh = (Rz(P['tibia_mesh_rpy'][2]) @ mesh.T).T + P['tibia_mesh_xyz']
    mesh_low = apply(T3, mesh)[:, 2].min()

    print(f'stand pose: femur={a.femur} rad, tibia={a.tibia} rad')
    print(f'  base_link height, tibia mesh lowest vertex   : {-mesh_low:.4f} m  <- CAD / RViz / real foot')
    print(f'  base_link height, collision foot sphere      : {-sphere_low:.4f} m  <- Gazebo contact')
    print(f'  Gazebo vs CAD mismatch                       : {(mesh_low - sphere_low) * 1000:.1f} mm (should be ~0)')
    print(f'set stand.base_height: {-mesh_low:.4f}   in config/rest_pose.yaml')


if __name__ == '__main__':
    main()
