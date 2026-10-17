from pathlib import Path
import yaml
import pytest
from sim.hardware import service_environment, wall_clock_config


def test_service_config_is_data_not_shell(tmp_path):
    file = tmp_path / 'service'
    file.write_text('export ROS_DOMAIN_ID="42" # domain\nFASTDDS_BUILTIN_TRANSPORTS=UDPv4\nSECRET=$(false)\n')
    assert service_environment(file) == {'ROS_DOMAIN_ID': '42', 'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'}
    file.write_text('ROS_DOMAIN_ID=42\nFASTDDS_BUILTIN_TRANSPORTS=DEFAULT\n')
    with pytest.raises(ValueError, match='UDPv4'):
        service_environment(file)


def test_all_nested_costmaps_use_wall_clock(tmp_path):
    source = tmp_path / 'source.yaml'; target = tmp_path / 'target.yaml'
    source.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'use_sim_time': True}},
        'global_costmap': {'global_costmap': {'ros__parameters': {'use_sim_time': True}}}}))
    wall_clock_config(source, target, {'mission_enabled': False})
    result = yaml.safe_load(target.read_text())
    assert result['/**']['ros__parameters'] == {'use_sim_time': False, 'mission_enabled': False}
    assert result['global_costmap']['global_costmap']['ros__parameters']['use_sim_time'] is False


@pytest.mark.parametrize('profile', ['map','dock','full','competition','competition_lite'])
@pytest.mark.parametrize('saved_map', [False,True])
def test_hardware_profiles(tmp_path,monkeypatch,profile,saved_map):
    import importlib.util,json
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch_ros.actions import Node
    from launch.actions import RegisterEventHandler
    share=lambda p:Path(get_package_share_directory(p))
    spec=importlib.util.spec_from_file_location('hardware_launch',share('sim')/'launch/hardware.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    captured=[]
    def capture(**kwargs):
        captured.append(kwargs);return Node(**kwargs)
    monkeypatch.setattr(module,'Node',capture)
    context=LaunchContext();service=tmp_path/'service';service.write_text('ROS_DOMAIN_ID=93\nFASTDDS_BUILTIN_TRANSPORTS=UDPv4\n')
    snapshot=''
    if saved_map:
        snapshot=str(tmp_path/'snapshot.json');(tmp_path/'map.yaml').write_text('{}')
        Path(snapshot).write_text(json.dumps({'format_version':1,'map_yaml':str(tmp_path/'map.yaml'),'landmarks':{str(i):{} for i in range(8)},'final_pose':[0.,0.,0.],'final_pose_covariance':[0.]*36}))
    monkeypatch.setenv('AUTONOMY_SAGA_TOKEN','test-only')
    context.launch_configurations.update(config=str(share('sim')/'config/hardware.yaml'),profile=profile,
        use_sim_time='false',run_dir=str(tmp_path),service_config=str(service),snapshot=snapshot,save_snapshot=str(tmp_path/'save.json'),
        saga_url='http://localhost:8000',detector_backend='onnx',detector_config='',odometry_config='',detector_python='/usr/bin/python3',foxglove='false',particles_per_robot='80',tag_search='coverage',coverage_config='')
    if profile in ('full','competition','competition_lite') and not saved_map:
        with pytest.raises(RuntimeError,match='completed map snapshot'):module.start(context)
        return
    actions=module.start(context)
    executables={n['executable'] for n in captured}
    full=profile in ('full','competition');mission=full or profile=='competition_lite'
    assert ('perception_perception_node' in executables)==full
    assert ('planning_planning_node' in executables)==mission
    assert ('planning_coverage_node' in executables)==(not saved_map and profile!='dock')
    assert ('explore' in executables)==(not saved_map and profile!='dock')
    assert 'control_navigation_relay' in executables
    assert all(n['parameters'][-1]['use_sim_time'] is False for n in captured)
    assert ('control_direct_command_node' in executables)!=mission
    assert ('amcl' in executables)==saved_map
    assert ('slam_toolbox' in {n.get('name') for n in captured})!=saved_map
    assert sum(n['executable']=='slam_wheel_odometry_node' for n in captured)==1
    for n in captured:
        if n['package']=='perception':assert n['respawn'] and n['respawn_delay']==3.
        if n['executable']=='lifecycle_manager':
            assert n['parameters'][0]['bond_timeout']==20.
            assert n['parameters'][0]['attempt_respawn_reconnection']
    # Targeted handlers only; optional nodes have no shutdown handler.
    assert sum(isinstance(a,RegisterEventHandler) for a in actions)==15
    import xml.etree.ElementTree as ET
    assert all(n.get('goal_checker_id')=='goal_checker' for n in ET.parse(tmp_path/'navigation.xml').iter('FollowPath'))
    control=yaml.safe_load((tmp_path/'control.yaml').read_text())
    if mission:assert control['/**']['ros__parameters']['coordinator_rate']==5.
    assert control['collision_monitor']['ros__parameters']['cmd_vel_out_topic']=='/cmd_vel'
    assert control['collision_monitor']['ros__parameters']['source_timeout']==1.
    assert 'collision_validation' not in control
    assert 'control_guard_deadband' not in executables
    assert 'control_escape_node' in executables
    planning=yaml.safe_load((tmp_path/'planning.yaml').read_text())
    global_costmap=planning['global_costmap']['global_costmap']['ros__parameters']
    local_costmap=control['local_costmap']['local_costmap']['ros__parameters']
    assert local_costmap['footprint_padding']==(.01 if saved_map else .06)
    assert global_costmap['footprint_padding']==(.01 if saved_map else .06)
    freshness=planning['/**']['ros__parameters']['costmap_stale_timeout']
    # Healthy publishers must refresh before admission rejects the map as stale.
    assert 1/global_costmap['publish_frequency'] < freshness
    assert 1/global_costmap['update_frequency'] < freshness
    assert control['controller_server']['ros__parameters']['controller_frequency']==10.
    assert control['controller_server']['ros__parameters']['FollowPath']['model_dt']==.1
    assert control['controller_server']['ros__parameters']['FollowPath']['batch_size']==1000
    assert planning['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']['inflation_radius']>=.17
    if not full:
        assert control['collision_monitor']['ros__parameters']['observation_sources']==['scan']
        assert planning['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['observation_sources']=='scan'


def test_lidar_source_height_bounds_do_not_use_jazzy_zero_default():
    from pathlib import Path
    import yaml
    timing=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/hardware_timing.yaml').read_text())
    for area,name in [('control','local_costmap'),('planning','global_costmap')]:
        scan=timing[area][name][name]['ros__parameters']['obstacle_layer']['scan']
        assert scan['min_obstacle_height']<=.167<=scan['max_obstacle_height']
