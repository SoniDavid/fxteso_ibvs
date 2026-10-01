"""The F450 airframe as every plant sees it, derived from one SDF.

One implementation, because three plants must integrate the same body: gz_sim.launch.py writes
the derived F450_base that DART flies under plant:=gazebo and plant:=px4, and uav_dynamics
integrates its composite mass and inertia under plant:=analytic. Loaded through importlib, as
camera_presets.py is.
"""
import math
import os
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

PKG = 'quad_description'
GRAVITY = 9.81
ROTOR_JOINTS = ('rotor_front_left_joint', 'rotor_rear_left_joint',
                'rotor_rear_right_joint', 'rotor_front_right_joint')


def base_sdf():
    """models/F450_base/model.sdf as shipped."""
    path = os.path.join(get_package_share_directory(PKG), 'models', 'F450_base', 'model.sdf')
    with open(path) as fh:
        return fh.read()


def _sub_once(text, pattern, repl, what):
    out, n = re.subn(pattern, repl, text, count=1, flags=re.DOTALL)
    if n != 1:
        raise RuntimeError('airframe.py could not find %s in F450_base/model.sdf.' % what)
    return out


def rewrite_mass(sdf, quad_mass):
    """base_link absorbs the change and its inertia scales with mass at fixed geometry - an
    approximation until the real airframe's inertia is measured. The rotors are unchanged."""
    total = sum(float(v) for v in re.findall(r'<mass>([^<]*)</mass>', sdf))
    if abs(quad_mass - total) < 1e-6:
        return sdf
    m = re.search(r"<link name='base_link'>.*?</inertial>", sdf, re.DOTALL)
    if m is None:
        raise RuntimeError('F450_base/model.sdf has no base_link <inertial>; cannot apply '
                           'quad_mass:=%g.' % quad_mass)
    blk = m.group(0)
    base = float(re.search(r'<mass>([^<]*)</mass>', blk).group(1))
    if quad_mass <= total - base:
        raise RuntimeError('quad_mass:=%g is not heavier than the rotors alone.' % quad_mass)
    blk = _sub_once(blk, r'<mass>[^<]*</mass>',
                    '<mass>%.6f</mass>' % (quad_mass - (total - base)), 'base <mass>')
    for tag in ('ixx', 'iyy', 'izz'):
        old = float(re.search(r'<%s>([^<]*)</%s>' % (tag, tag), blk).group(1))
        blk = _sub_once(blk, r'<%s>[^<]*</%s>' % (tag, tag),
                        '<%s>%.9g</%s>' % (tag, old * quad_mass / total, tag), tag)
    return sdf[:m.start()] + blk + sdf[m.end():]


def weld_rotors(sdf):
    """Rotor joints revolute -> fixed. Under DART a joint velocity command is a SERVO constraint,
    so a spinning rotor pushes rotor inertia x delta-omega into the airframe; the wrench-driven
    plant models no rotor, so its rotors carry mass and inertia but must not spin."""
    for name in ROTOR_JOINTS:
        sdf = _sub_once(
            sdf, r"<joint name='%s' type='revolute'>(.*?)\s*<axis>.*?</axis>" % name,
            r"<joint name='%s' type='fixed'>\1" % name, 'joint %s' % name)
    return sdf


def strip_rotor_spin(sdf):
    """Drop F450's four JointController plugins; with welded rotors they have nothing to drive."""
    out, n = re.subn(r'\s*<plugin filename="gz-sim-joint-controller-system".*?</plugin>', '',
                     sdf, flags=re.DOTALL)
    if n != 4:
        raise RuntimeError('F450/model.sdf has %d JointController plugins, expected 4.' % n)
    return out


def _pose(elem):
    """(translation, rotation matrix) of an SDF <pose> child, identity if absent."""
    p = elem.find('pose')
    v = [float(x) for x in p.text.split()] if p is not None else [0.0] * 6
    r, pi, y = v[3:]
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(pi), math.sin(pi),
                              math.cos(y), math.sin(y))
    rot = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
           [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
           [-sp, cp * sr, cp * cr]]
    return v[:3], rot


def _mat(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def composite(sdf):
    """Total mass, centre of mass and inertia about it, in the model frame (FLU), summed over
    every link's <inertial>. Link poses are relative to the model, as in F450_base."""
    root = ET.fromstring(sdf)
    mass, moment, bodies = 0.0, [0.0, 0.0, 0.0], []
    for link in root.iter('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        lt, lr = _pose(link)
        it, ir = _pose(inertial)
        c = [lt[i] + sum(lr[i][k] * it[k] for k in range(3)) for i in range(3)]
        rot = _mat(lr, ir)
        m = float(inertial.find('mass').text)
        t = inertial.find('inertia')
        g = {k: float(t.find(k).text) if t.find(k) is not None else 0.0
             for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')}
        local = [[g['ixx'], g['ixy'], g['ixz']], [g['ixy'], g['iyy'], g['iyz']],
                 [g['ixz'], g['iyz'], g['izz']]]
        bodies.append((m, c, _mat(_mat(rot, local), _transpose(rot))))
        mass += m
        moment = [moment[i] + m * c[i] for i in range(3)]
    com = [x / mass for x in moment]
    J = [[0.0] * 3 for _ in range(3)]
    for m, c, I in bodies:
        d = [c[i] - com[i] for i in range(3)]
        dd = sum(x * x for x in d)
        for i in range(3):
            for j in range(3):
                J[i][j] += I[i][j] + m * ((dd if i == j else 0.0) - d[i] * d[j])
    return {'mass': mass, 'com': com, 'J': J}


def derive(quad_mass):
    """The composite body DART flies for quad_mass; what uav_dynamics must integrate too."""
    return composite(rewrite_mass(base_sdf(), quad_mass))


ROTOR_LINKS = ('rotor_front_left', 'rotor_rear_left', 'rotor_rear_right', 'rotor_front_right')
# Spin sense from above, as F450/model.sdf's JointControllers declare it.
ROTOR_SPIN = {'rotor_front_left': -1.0, 'rotor_rear_left': 1.0,
              'rotor_rear_right': -1.0, 'rotor_front_right': 1.0}


def _link_block(sdf, name):
    m = re.search(r"<link name='%s'>.*?</link>" % name, sdf, re.DOTALL)
    if m is None:
        raise RuntimeError('F450_base/model.sdf has no link %s.' % name)
    return m


def strip_rotor_visuals(sdf):
    """Welded rotor links keep their inertia but lose their meshes; rotor_visuals() draws them."""
    for name in ROTOR_LINKS:
        m = _link_block(sdf, name)
        blk, n = re.subn(r'\s*<visual name=.*?</visual>', '', m.group(0), flags=re.DOTALL)
        if n != 1:
            raise RuntimeError('link %s has %d visuals, expected 1.' % (name, n))
        sdf = sdf[:m.start()] + blk + sdf[m.end():]
    return sdf


def rotor_visuals(sdf, spawn):
    """Visual-only rotor models for the world, plus the parameters gz_pose_broadcaster poses them
    with. Each carries its link's own <visual>, so the mesh has one source."""
    models, params = [], {'rotor_models': [], 'rotor_offsets': [], 'rotor_signs': []}
    for name in ROTOR_LINKS:
        blk = _link_block(sdf, name).group(0)
        hub = [float(x) for x in re.search(r'<pose>([^<]*)</pose>', blk).group(1).split()[:3]]
        visual = re.search(r'<visual name=.*?</visual>', blk, re.DOTALL).group(0)
        model = 'F450_' + name
        models.append(
            "<model name='%s'>\n  <pose>%.6f %.6f %.6f 0 0 0</pose>\n"
            "  <link name='link'>\n    <gravity>0</gravity>\n"
            "    <inertial><mass>0.001</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy>"
            "<izz>1e-7</izz></inertia></inertial>\n    %s\n  </link>\n</model>"
            % (model, spawn[0] + hub[0], spawn[1] + hub[1], spawn[2] + hub[2], visual))
        params['rotor_models'].append(model)
        params['rotor_offsets'] += hub
        params['rotor_signs'].append(ROTOR_SPIN[name])
    return '\n'.join(models), params
