"""Verify actual run artifacts, not generated trajectories or manually fabricated metrics."""
import argparse
import csv
import json
from pathlib import Path
import math
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from interfaces.msg import RobotTruthArray


def check(directory):
    directory=Path(directory)
    result=json.loads((directory/'result.json').read_text())
    params=yaml.safe_load((directory/'params.yaml').read_text())['/**']['ros__parameters']
    assert result['traffic_count']==3, 'Full Phase 1 check needs three traffic robots'
    events=list(csv.DictReader((directory/'events.csv').open()))
    for truck in ('2','3','4'):
        completed=[r for r in events if r['kind']=='route' and r['truck_or_robot']==truck and r['action_or_severity']=='unloading_complete']
        assert completed, 'Truck '+truck+' did not complete a Saga route'
        dwells=[r for r in events if r['kind']=='route' and r['truck_or_robot']==truck and r['action_or_severity'] in ('loading_complete','unloading_complete')]
        assert all(float(r['dwell_sim_s'])>=params['dwell_seconds'] and float(r['dwell_wall_s'])>=params['wall_lock_seconds'] for r in dwells)
    forced=[r for r in events if r['kind']=='collision' and r['forced_test']=='True']
    assert forced, 'No labelled forced collision event'
    metadata=yaml.safe_load((directory/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
    counts={t['topic_metadata']['name']:t['message_count'] for t in metadata['topics_with_message_count']}
    assert counts.get('/cmd_vel',0)==0, 'Phase 1 must not command the ego robot'
    for topic in ['/sim/truth','/sim/collisions','/sim/routes','/sim/detections','/clock','/scan']:
        assert counts.get(topic,0)>0, topic+' missing from bag'
    for robot in range(1,4):
        assert counts.get(f'/traffic_{robot}/odom',0)>0
        assert counts.get(f'/traffic_{robot}/cmd_vel',0)>0
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory/'bag'),storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('cdr','cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/sim/truth']))
    first={}; max_distance={i:0. for i in range(4)}; complete_samples=0
    while reader.has_next():
        _,data,_=reader.read_next(); message=deserialize_message(data,RobotTruthArray)
        if len(message.robots)==4:
            complete_samples+=1
        for robot in message.robots:
            xy=(robot.pose.position.x,robot.pose.position.y)
            first.setdefault(robot.robot_id,xy)
            max_distance[robot.robot_id]=max(max_distance[robot.robot_id],math.dist(first[robot.robot_id],xy))
    assert complete_samples>0
    assert all(max_distance[i]>params['robot_length'] for i in range(1,4)), max_distance
    metrics=list(csv.DictReader((directory/'metrics.csv').open()))
    assert metrics and int(metrics[-1]['truth_count'])==4
    assert int(metrics[-1]['detector_frames'])>0
    rates=[]
    for line in (directory/'scan_hz.log').read_text().splitlines():
        if line.startswith('average rate:'):
            rates.append(float(line.split(':')[1]))
    assert rates, 'Missing required ros2 topic hz /scan measurement'
    print('PASS: three traffic trucks completed Saga routes with both dwell clocks; four true poses;')
    print('      forced collision logged; detector frames, metrics CSV and readable bag recorded; no ego commands.')
    print(f'/scan={rates[-1]:.3f} Hz; scan-based RTF={rates[-1]/9.89:.3f}; clock/wall RTF={result["real_time_factor"]:.3f}')
    print('Maximum displacement [m]: '+str(max_distance))


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('run_directory')
    check(parser.parse_args().run_directory)
