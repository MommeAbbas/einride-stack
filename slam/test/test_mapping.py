import json
import numpy as np
import cv2
import pytest
from slam.landmarks import detect_tags
from slam.snapshot import sanitise_ranges,load_snapshot,fuse_landmark


def test_invalid_bins_never_become_clearing_rays():
    clean,count=sanitise_ranges([float('nan'),float('inf'),-float('inf'),0.,.01,.02,2.,12.,13.],.02,12.)
    assert clean==[0.,0.,0.,0.,0.,.02,2.,12.,0.] and count==6


def test_final_mapping_pose_is_not_replaced_by_origin(tmp_path):
    (tmp_path/'map.yaml').write_text('image: map.pgm\n')
    data=dict(format_version=1,map_yaml='map.yaml',landmarks={str(i):{} for i in range(8)},final_pose=[2.,-1.,.8],final_pose_covariance=np.eye(6).ravel().tolist())
    path=tmp_path/'slam_snapshot.json';path.write_text(json.dumps(data))
    loaded=load_snapshot(path)
    assert loaded['final_pose']==[2.,-1.,.8] and loaded['final_pose_covariance']==data['final_pose_covariance']
    data['final_pose'][0]=float('nan');path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='Nonfinite'):load_snapshot(path)


def test_calibrated_tag_identity_and_depth():
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tag=cv2.aruco.drawMarker(dictionary,3,100);image=np.full((400,400),255,np.uint8);image[150:250,150:250]=tag
    observations=detect_tags(image,np.array([[400.,0,200],[0,400,200],[0,0,1]]),np.zeros(5),.1,2.)
    assert len(observations)==1 and observations[0][0]==3
    assert abs(observations[0][1][2]-.4)<.02


def test_dock_fusion_replaces_bad_seed_but_rejects_confirmed_outlier():
    p=dict(tag_sigma=.08,tag_observation_interval=.5,tag_outlier_gate=.3,tag_min_observations=3,covariance_intersection_weight=.5)
    seed=fuse_landmark(None,[20.,0.,.1],0.,1.,False,p)
    near=fuse_landmark(seed,[1.,2.,.1],np.pi-.02,2.,True,p)
    assert near['count']==1 and np.allclose(near['xy'],[1,2])
    repeat=fuse_landmark(near,[1.,2.,.1],-np.pi+.02,3.,True,p)
    assert abs(abs(repeat['heading'])-np.pi)<1e-6
    assert np.allclose(repeat['cov'],near['cov']) # Correlated repeats cannot invent confidence.
    repeat['count']=3
    assert fuse_landmark(repeat,[20.,0.,.1],0.,4.,True,p) is None


def test_range_incidence_uncertainty_and_partial_snapshot(tmp_path):
    from slam.snapshot import tag_covariance,landmark_valid
    p=dict(tag_sigma=.08,tag_yaw_sigma=.12,tag_reference_range=1.,tag_min_cosine=.1,
           tag_min_observations=3,tag_geometry_covariance=True,dock_max_position_variance=.01,dock_max_yaw_variance=.04)
    near,yaw=tag_covariance(.9,0.,p)
    far,far_yaw=tag_covariance(2.5,np.pi/4,p)
    assert landmark_valid(dict(count=3,cov=near,yaw_var=yaw),p)
    assert not landmark_valid(dict(count=3,cov=far,yaw_var=far_yaw),p)
    (tmp_path/'map.yaml').write_text('image: map.pgm\n')
    snapshot=dict(format_version=1,map_yaml='map.yaml',landmarks={},partial=True,final_pose=[0.,0.,0.],final_pose_covariance=[0.]*36)
    path=tmp_path/'snapshot.json';path.write_text(json.dumps(snapshot))
    assert load_snapshot(path)['partial']


def test_real_slam_node_initializes_live_dock_topics():
    import rclpy
    from slam.slam_node import SlamNode
    rclpy.init();node=SlamNode()
    try:
        assert set(p.topic_name for p in node.live_docks.values())=={f'/slam/live_docks/dock_{i}' for i in range(8)}
    finally:node.destroy_node();rclpy.shutdown()


def test_camera_waits_for_capture_time_tf_instead_of_dropping_early_frame():
    import rclpy
    from rclpy.task import Future
    from sensor_msgs.msg import Image,CameraInfo
    from geometry_msgs.msg import TransformStamped
    from slam.slam_node import SlamNode
    rclpy.init();node=SlamNode()
    try:
        f=Future();node.buffer.wait_for_transform_async=lambda *a:f
        node.camera=CameraInfo();frames=[];node.process_image=lambda m,tr:frames.append(m.header.stamp.sec)
        m=Image();m.header.frame_id='camera';m.header.stamp.sec=7
        assert node.image_wait_timer.is_canceled()
        node.on_image(m);assert node.pending_image is not None and not frames
        assert not node.image_wait_timer.is_canceled()
        f.set_result(TransformStamped());node.finish_image_wait()
        assert frames==[7] and node.pending_image is None and node.dropped_tf==0
        assert node.image_wait_timer.is_canceled()
        f2=Future();node.buffer.wait_for_transform_async=lambda *a:f2
        m.header.stamp.sec=8;node.on_image(m)
        node.now=lambda:node.pending_image[2]+node.p('tag_tf_wait_timeout')+1
        node.finish_image_wait();assert node.pending_image is None and node.dropped_tf==1 and f2.cancelled()
        assert node.image_wait_timer.is_canceled()
    finally:node.destroy_node();rclpy.shutdown()
