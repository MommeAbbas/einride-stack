"""Generate a private arena copy and sensor-free traffic; upstream assets are read-only."""
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
import yaml
from .geometry import rectangle,overlap
from .grid import Grid


def pose(element):
    return [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]


def compose(a, b):
    c, s = math.cos(a[5]), math.sin(a[5])
    return [a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], a[2]+b[2], 0, 0, a[5]+b[5]]


def mesh_polygon(path, scale):
    """Conservative projected convex hull of COLLADA vertices for static oracle map."""
    tree = ET.parse(path)
    ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    vertices=[]
    for mesh in tree.findall('.//c:geometry/c:mesh', ns):
        refs={v.attrib['source'][1:] for v in mesh.findall('c:vertices/c:input', ns)
              if v.attrib.get('semantic') == 'POSITION'}
        for source in mesh.findall('c:source', ns):
            if source.attrib['id'] in refs:
                vertices.extend(np.fromstring(source.find('c:float_array',ns).text, sep=' ').reshape(-1,3))
    unit=tree.find('c:asset/c:unit',ns)
    meter=float(unit.get('meter','1')) if unit is not None else 1.
    if tree.findtext('c:asset/c:up_axis','Z_UP',ns)!='Z_UP':
        raise ValueError('Oracle mesh importer requires Z_UP assets')
    for tag in ('matrix','translate','rotate','scale'):
        if tree.findall('.//c:visual_scene//c:'+tag,ns):
            raise ValueError('Oracle mesh importer requires baked scene transforms')
    points=np.asarray(vertices)*scale*meter
    # These vendored props are Z_UP with identity scene scale/rotation, as measured upstream.
    points=np.unique(points[:,:2],axis=0)
    return points[ConvexHull(points).vertices]


def static_geometry(world, assets):
    polygons, docks = [], {}
    items=[(model,pose(model),model.attrib['name']) for model in world.findall('model')
           if model.attrib['name'] != 'arena_floor']
    for inc in world.findall('include'):
        uri=inc.findtext('uri').removeprefix('model://')
        if uri == 'einride_mini_truck':
            continue
        model=ET.parse(assets/uri/'model.sdf').getroot().find('model')
        items.append((model,pose(inc),inc.findtext('name')))
    for model, base, name in items:
        if name.startswith('dock_'):
            docks[name[-1].upper()] = base
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                p=compose(compose(base,pose(link)),pose(collision))
                geometry=collision.find('geometry')
                if geometry.find('box') is not None:
                    size=[float(v) for v in geometry.findtext('box/size').split()]
                    poly=rectangle(p[0],p[1],p[5],size[0],size[1])
                elif geometry.find('mesh') is not None:
                    mesh=geometry.find('mesh')
                    uri=mesh.findtext('uri').removeprefix('model://')
                    scale=np.fromstring(mesh.findtext('scale','1 1 1'),sep=' ')
                    local=mesh_polygon(assets/uri,scale)
                    c,s=math.cos(p[5]),math.sin(p[5])
                    poly=local@np.array([[c,s],[-s,c]])+[p[0],p[1]]
                else:
                    raise ValueError('Unsupported static collision geometry: '+name)
                polygons.append(poly.tolist())
    return polygons,docks


def generate(gazebo_share, description_share, output, params):
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    assets=Path(gazebo_share)/'worlds'
    tree=ET.parse(assets/'arena.sdf'); world=tree.getroot().find('world')
    polygons,docks=static_geometry(world,assets)
    if params.get('map_source')=='snapshot':
        # Restart the physical simulation where mapping ended. AMCL separately
        # receives the ESTIMATED map pose; no truth is fed to a production node.
        handover=Path(params['map_snapshot']).resolve().parent/'sim_handover.json'
        initial=json.loads(handover.read_text())['world_pose']
        for inc in world.findall('include'):
            if inc.findtext('uri')=='model://einride_mini_truck':
                element=inc.find('pose')
                if element is None:element=ET.SubElement(inc,'pose')
                element.text=' '.join(map(str,initial))
    for dock,p in docks.items():
        p[0]+=params['dock_standoff']*math.cos(p[5])
        p[1]+=params['dock_standoff']*math.sin(p[5])
    count=int(params['traffic_count'])
    if count not in range(4):
        raise ValueError('traffic_count must be 0..3')
    model_file=Path(description_share)/'models/einride_mini_truck/model.sdf'
    source=ET.parse(model_file).getroot().find('model')
    # Simulation setup only: reject overlaps before physics creates an unevaluable
    # contact before the first /cmd_vel. Reuse the evaluation footprint geometry.
    ego=next(i for i in world.findall('include') if i.findtext('uri')=='model://einride_mini_truck')
    initial=pose(ego)
    spawns=[[initial[0],initial[1],initial[5]]]+[params['traffic_spawns'][3*i:3*i+3] for i in range(count)]
    footprints=[rectangle(*p,params['collision_footprint_length'],params['collision_footprint_width']) for p in spawns]
    for i,a in enumerate(footprints):
        for j,b in enumerate(footprints[:i]):
            if overlap(a,b):raise ValueError(f'Initial robot footprint overlap: robots {j} and {i}; choose separated traffic_spawns')
    bridge=[]
    for i in range(1,count+1):
        model=deepcopy(source); model.set('name',f'traffic_{i}')
        x,y,a=params['traffic_spawns'][3*(i-1):3*i]
        ET.SubElement(model,'pose').text=f'{x} {y} 0.01 0 0 {a}' # Same spawn clearance as upstream.
        for link in model.findall('link'):
            for sensor in link.findall('sensor'):
                link.remove(sensor)
        for plugin in model.findall('plugin'):
            if not plugin.get('name','').endswith('::DiffDrive'):
                model.remove(plugin); continue
            for key,value in {'topic':f'/traffic_{i}/cmd_vel','odom_topic':f'/traffic_{i}/odom',
                              'tf_topic':f'/traffic_{i}/pose','frame_id':f'traffic_{i}/odom',
                              'child_frame_id':f'traffic_{i}/base_footprint'}.items():
                plugin.find(key).text=value
        world.append(model)
        for topic,ros_type,gz_type,direction in [('cmd_vel','geometry_msgs/msg/Twist','gz.msgs.Twist','ROS_TO_GZ'),
                                                ('odom','nav_msgs/msg/Odometry','gz.msgs.Odometry','GZ_TO_ROS')]:
            bridge.append(dict(ros_topic_name=f'/traffic_{i}/{topic}',gz_topic_name=f'/traffic_{i}/{topic}',
                               ros_type_name=ros_type,gz_type_name=gz_type,direction=direction))
    if params.get('ego_camera_width',0):
        # Private evaluation world only: preserve robot geometry, topics and FOV.
        model=deepcopy(source)
        element=model.find('pose')
        if element is None:element=ET.SubElement(model,'pose')
        element.text=' '.join(map(str,initial))
        for sensor in model.findall('.//sensor'):
            if sensor.get('name') in ('oak_rgb','oak_stereo'):
                sensor.find('camera/image/width').text=str(params['ego_camera_width'])
                sensor.find('camera/image/height').text=str(params['ego_camera_height'])
                sensor.find('update_rate').text=str(params['ego_camera_rate'])
        world.remove(ego);world.append(model)
    ET.indent(tree); tree.write(output/'arena.sdf',encoding='unicode',xml_declaration=True)
    (output/'bridge.yaml').write_text(yaml.safe_dump(bridge))
    (output/'geometry.json').write_text(json.dumps({'polygons':polygons,'docks':docks}))
    grid=Grid.from_polygons(polygons,params['map_resolution'],params['map_extent'])
    np.savez_compressed(output/'map.npz',occupied=grid.occupied,resolution=grid.resolution,origin=grid.origin)
    return grid,docks
