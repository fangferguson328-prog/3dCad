"""Constructed planar half-circle; no user logs, Qt, SDK, or robot connection.
Mirrors X-direction formulas in mainwindow.cpp:4244-4266 and
openglwidget.cpp:2168-2189, compared with old flat-weld fixed-X policy.
"""
import math


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def normalize(v):
    length = math.sqrt(dot(v, v))
    assert length > 1e-12
    return tuple(x/length for x in v)


def projected_chord(a, b, normal):
    chord = normalize(sub(b, a))
    return normalize(tuple(chord[i] - dot(chord, normal)*normal[i] for i in range(3)))


def direction_angle(v):
    return math.degrees(math.atan2(v[1], v[0]))


def report(name, values):
    print(name)
    for point, v in zip(('start', 'middle', 'end'), values):
        print(f'  {point:6s}: X={tuple(round(x, 6) for x in v)}, atan2={direction_angle(v):8.3f} deg')


# Camera-to-base rotation is identity for this constructed example.
# Circle center=(0,0,1) m, radius=0.1 m, traversal counterclockwise.
p0, p1, p2 = (.1,0.,1.), (0.,.1,1.), (-.1,0.,1.)
normal = (0.,0.,-1.)
tangent0 = projected_chord(p0,p1,normal)  # mainwindow.cpp:4244
tangent1 = projected_chord(p0,p2,normal)  # mainwindow.cpp:4247
tangentMid = projected_chord(p1,p2,normal)  # mainwindow.cpp:4250
old_flat = [normalize((-1.,-1.,0.))]*3
current_2d = [tangent0, tangentMid, tangent1]  # mainwindow.cpp:4266
current_3d = [tangent0, tangent1, tangentMid]  # openglwidget.cpp:2189
true_tangents = [(0.,1.,0.), (-1.,0.,0.), (0.,-1.,0.)]

assert current_2d[1] == current_3d[2]
assert current_2d[2] == current_3d[1]
assert abs(direction_angle(current_2d[1]) - (-135.)) < 1e-9
assert abs(direction_angle(current_2d[2]) - 180.) < 1e-9
print('CONSTRUCTED EXAMPLE; NOT USER LOGS; tool Z=(0,0,1) for all methods.')
for name, values in [('Old flat-weld fixed X', old_flat),
                     ('Current 2D ARC array', current_2d),
                     ('Current 3D ARC array', current_3d),
                     ('Exact circular tangents', true_tangents)]:
    report(name, values)
print('PASS: 2D middle/end correspondence differs from the formulas used by 3D.')


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def current_orientation(z, x):
    """Scalar transcription of common.cpp:495-541, not a replacement solver."""
    z, x = normalize(z), normalize(x)
    y = normalize(cross(z, x))
    pitch = math.asin(-x[2])
    roll = 0.0 if abs(pitch) >= math.pi/2 else math.atan2(y[2], z[2])
    yaw = math.atan2(x[1], x[0])
    if yaw > 0:
        yaw -= 2*math.pi
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def reconstruct_z(euler):
    """Third column of Rz*Ry*Rx, the convention used by composeTransformMatrix."""
    a, b, c = map(math.radians, euler)
    return (math.cos(c)*math.sin(b)*math.cos(a)+math.sin(c)*math.sin(a),
            math.sin(c)*math.sin(b)*math.cos(a)-math.cos(c)*math.sin(a),
            math.cos(b)*math.cos(a))


def angle_between(a, b):
    return math.degrees(math.acos(max(-1., min(1., dot(normalize(a), normalize(b))))))


yaws = [current_orientation((0., 0., 1.),
                           (math.cos(math.radians(a)), math.sin(math.radians(a)), 0.))[2]
        for a in (-1., 0., 1.)]
assert abs(yaws[2]-yaws[0]+358.) < 1e-9
print('Yaw boundary: physical [-1,0,1] deg -> output', yaws)

for label, z, x, expected_error in [
    ('Nonorthogonal input', (0., 0., 1.), (1., 0., 1.), 45.),
    ('Exact gimbal branch', (0., -1., 0.), (0., 0., -1.), 90.),
]:
    euler = current_orientation(z, x)
    actual_z = reconstruct_z(euler)
    error = angle_between(z, actual_z)
    assert abs(error-expected_error) < 1e-8
    print(f'{label}: euler={euler}, reconstructed Z={actual_z}, Z error={error:.6f} deg')

print('PASS means the described defects were reproduced, not that they were fixed.')
print('No application build, controller simulation, inverse kinematics or hardware operation was performed.')
