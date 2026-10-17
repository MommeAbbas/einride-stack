"""Hardware launch configuration only: no simulator imports, truth, or service changes."""
from pathlib import Path
import shlex
import yaml


def service_environment(path):
    values = {}
    for line in Path(path).expanduser().read_text().splitlines():
        fields = shlex.split(line, comments=True)
        if fields and fields[0] == 'export':
            fields = fields[1:]
        if len(fields) != 1 or '=' not in fields[0]:
            continue
        key, value = fields[0].split('=', 1)
        if key in ('ROS_DOMAIN_ID', 'FASTDDS_BUILTIN_TRANSPORTS'):
            values[key] = value
    try:
        domain = int(values['ROS_DOMAIN_ID'])
    except (KeyError, ValueError) as error:
        raise ValueError('Service config must contain a numeric ROS_DOMAIN_ID') from error
    if values.get('FASTDDS_BUILTIN_TRANSPORTS') != 'UDPv4':
        raise ValueError('Base service must already use FASTDDS_BUILTIN_TRANSPORTS=UDPv4; update/restart it before launching')
    if domain < 0:
        raise ValueError('Negative ROS_DOMAIN_ID')
    return values


def wall_clock_config(source, destination, overrides=None, overlay=None):
    data = yaml.safe_load(Path(source).read_text())
    def merge(target, patch):
        for key, value in patch.items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                merge(target[key], value)
            else:
                target[key] = value
    if overlay is not None:
        merge(data, yaml.safe_load(Path(overlay).read_text()))
    def visit(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key == 'use_sim_time':
                    value[key] = False
                else:
                    visit(item)
    visit(data)
    data['/**']['ros__parameters'].update(overrides or {}, use_sim_time=False)
    Path(destination).write_text(yaml.safe_dump(data))
    return str(destination)


def merge_parameters(target, patch):
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            merge_parameters(target[key], value)
        else:
            target[key] = value
    return target


def profile_configurations(share, directory, profile, sim_time, snapshot, save, saga_url, tag_search="coverage"):
    """Build inspectable per-node YAMLs; no processes or middleware side effects."""
    import math
    import ast
    if profile not in ('map','dock','full','competition','competition_lite'):
        raise ValueError('unknown hardware profile')
    profile={'competition':'full'}.get(profile,profile)
    mission=profile in ('full','competition_lite')
    timing=yaml.safe_load((share('sim')/'config/hardware_timing.yaml').read_text())
    configs={}
    for area in ('control','planning','perception','slam'):
        data=yaml.safe_load((share(area)/('config/'+area+'.yaml')).read_text())
        if area=='control':
            if mission:data['/**']['ros__parameters']['coordinator_rate']=timing['mission_coordinator_rate']
            merge_parameters(data,yaml.safe_load((share('control')/'config/hardware_motion.yaml').read_text()))
        merge_parameters(data,timing.get(area,{}))
        if not snapshot and area in ('control','planning'):
            # Mapping must not plan inside the hard Stop envelope. Keep docking
            # profiles' nominal footprint: their final goal intentionally nears a wall.
            name='local_costmap' if area=='control' else 'global_costmap'
            data[name][name]['ros__parameters']['footprint_padding']=timing['mapping_footprint_padding']
        if area=='planning':
            data['/**']['ros__parameters'].update(explore=False, mission_enabled=True,
                saga_url=saga_url,opennav_docking=True,risk_check=False,look=False,observation_curves=False,dock_use_predictions=False)
            costmap=data['global_costmap']['global_costmap']['ros__parameters']
            footprint=ast.literal_eval(costmap['footprint']);pad=costmap['footprint_padding']
            radius=max(math.hypot(abs(x)+pad,abs(y)+pad) for x,y in footprint)
            layer=costmap.setdefault('inflation_layer',{})
            layer['inflation_radius']=max(radius,layer.get('inflation_radius',0.0))
        if area=='control':
            tree='navigate_to_pose_w_replanning_and_recovery.xml' if not mission else 'navigate_w_replanning_time.xml'
            # With two checkers, Nav2 cannot resolve an empty FollowPath checker ID.
            import xml.etree.ElementTree as ET
            bt=ET.parse(share('nav2_bt_navigator')/'behavior_trees'/tree)
            for follow in bt.iter('FollowPath'):follow.set('goal_checker_id','goal_checker')
            navigation_tree=directory/'navigation.xml';bt.write(navigation_tree,encoding='unicode')
            data['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(navigation_tree)
            data['/**']['ros__parameters']['require_predictions']=False
            data['/**']['ros__parameters']['hardware_mapping_passthrough']=mission and not snapshot
            data['controller_server']['ros__parameters']['FollowPath']['PredictionCritic'].update(enabled=False,require_predictions=False)
        if profile!='full' and area in ('control','planning'):
            name='local_costmap' if area=='control' else 'global_costmap'
            layer=data[name][name]['ros__parameters']['obstacle_layer']
            layer['observation_sources']='scan';layer.pop('depth',None);layer.pop('depth_clearing',None)
            if area=='control':
                guard=data['collision_monitor']['ros__parameters']
                guard['observation_sources']=['scan'];guard.pop('depth',None)
        if area=='control':
            data['collision_monitor']['ros__parameters'].pop('Slow',None)
            data['/**']['ros__parameters']['opennav_docking']=True
        if area=='perception':
            data['/**']['ros__parameters'].update(detector_source='external',external_detector_topic='/sim/detections')
        if area=='slam':data['/**']['ros__parameters'].update(save_snapshot=save,load_snapshot=snapshot,
            mapping_managed=not snapshot and profile!='dock',tag_geometry_covariance=True)
        def clock(value):
            if isinstance(value,dict):
                for key,item in value.items():
                    if key=='use_sim_time':value[key]=sim_time
                    else:clock(item)
        clock(data);data['/**']['ros__parameters']['use_sim_time']=sim_time
        file=directory/(area+'.yaml');file.write_text(yaml.safe_dump(data));configs[area]=str(file)
    backend=yaml.safe_load((share('slam')/('config/localisation.yaml' if snapshot else 'config/mapping.yaml')).read_text())
    if not snapshot:merge_parameters(backend,timing['slam'])
    clock(backend)
    if snapshot:
        from slam.snapshot import load_snapshot
        backend['map_server']['ros__parameters']['yaml_filename']=load_snapshot(snapshot)['map_yaml']
    file=directory/'slam_backend.yaml';file.write_text(yaml.safe_dump(backend));configs['slam_backend']=str(file)
    coverage=yaml.safe_load((share('planning')/'config/coverage.yaml').read_text())
    c=coverage['/**']['ros__parameters'];slam=yaml.safe_load(Path(configs['slam']).read_text())['/**']['ros__parameters']
    c.update(use_sim_time=sim_time,tag_search=tag_search,coverage_validator_rate=slam['tag_rate'],
        coverage_sighting_interval=slam['tag_observation_interval'],coverage_required_sightings=slam['tag_min_observations'],
        coverage_max_position_variance=slam['dock_max_position_variance'],coverage_max_yaw_variance=slam['dock_max_yaw_variance'])
    file=directory/'coverage.yaml';file.write_text(yaml.safe_dump(coverage));configs['coverage']=str(file)
    docking=yaml.safe_load((share('control')/'config/docking.yaml').read_text())
    dp=docking['docking_server']['ros__parameters'];dp['use_sim_time']=sim_time
    dp['dock_plugins']=[f'dock_{i}' for i in range(8)]
    for i in range(8):
        dp[f'dock_{i}']=dict(docking['dock_template']['ros__parameters'],detection_topic=f'/slam/live_docks/dock_{i}',
            external_detection_translation_x=-slam['dock_standoff'],staging_x_offset=-(slam['approach_standoff']-slam['dock_standoff']))
    del docking['dock_template']
    file=directory/'docking.yaml';file.write_text(yaml.safe_dump(docking));configs['docking']=str(file)
    return configs,timing['lifecycle']
