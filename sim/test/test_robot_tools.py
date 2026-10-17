from pathlib import Path



def test_competition_camera_driver_configuration_preserves_clock_and_alignment():
    import yaml
    directory=Path(__file__).resolve().parents[1]/'config'
    mapping=yaml.safe_load((directory/'oak_hardware.yaml').read_text())['/oak']['ros__parameters']
    mission=yaml.safe_load((directory/'oak_competition.yaml').read_text())['/oak']['ros__parameters']
    assert mission['camera']==mapping['camera']
    for stream in ('rgb','stereo'):
        assert [mission[stream][key] for key in ('i_width','i_height','i_fps')]==[640,360,5.]
        assert mission[stream]['i_get_base_device_timestamp'] is False
        assert mission[stream]['i_update_ros_base_time_on_ros_msg'] is True
        assert mission[stream]['i_max_q_size']==1
        assert [mapping[stream][key] for key in ('i_width','i_height')]==[1280,720]
    assert mission['stereo']['i_align_depth'] is True
    assert mission['rgb']['i_isp_num']/mission['rgb']['i_isp_den']==1/3
