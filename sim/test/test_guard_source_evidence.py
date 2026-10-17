import numpy as np
from geometry_msgs.msg import Transform
from guard_source_evidence import inside_polygon, transform_points


def test_recorded_contact_and_nearby_point_differ():
    points=np.array([[.12596,.1,0.],[.2,0.,0.],[-.12596,.1,0.]])
    footprint=[[.1325,.125],[.1325,-.125],[-.1325,-.125],[-.1325,.125]]
    assert inside_polygon(points,footprint).tolist()==[True,False,True]


def test_camera_points_rotate_and_translate_to_base():
    t=Transform();t.rotation.z=np.sin(np.pi/4);t.rotation.w=np.cos(np.pi/4);t.translation.z=.167
    result=transform_points(np.array([[1.,0.,0.]]),t)
    assert np.allclose(result,[[0.,1.,.167]])
