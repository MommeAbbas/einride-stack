#!/usr/bin/env python3
"""Isolated domain-95 regression: actual DockMap delivery for IDs 32 and 33."""
import os,sys,time,json,subprocess
from pathlib import Path
out=Path(os.environ.get('AUTONOMY_DISCOVERY_PROBE_OUTPUT','/tmp/autonomy_discovery_probe'));out.mkdir(parents=True,exist_ok=True)
if len(sys.argv)>1:
 import rclpy
 from rclpy.parameter import Parameter
 from rclpy.qos import QoSProfile,DurabilityPolicy
 from interfaces.msg import DockMap
 rclpy.init();n=rclpy.create_node('discovery_'+sys.argv[1],parameter_overrides=[Parameter('use_sim_time',value=True)])
 received=[];q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 if sys.argv[1]=='pub':p=n.create_publisher(DockMap,'/probe/docks',q)
 else:n.create_subscription(DockMap,'/probe/docks',lambda m:received.append(m.ready),q)
 end=time.monotonic()+8
 while time.monotonic()<end:
  if sys.argv[1]=='pub':p.publish(DockMap(ready=True,map_revision=1))
  rclpy.spin_once(n,timeout_sec=.1)
 print(json.dumps({'role':sys.argv[1],'received':len(received)}),flush=True)
 n.destroy_node();rclpy.shutdown();sys.exit()
results={}
for fixed in (False,True):
 processes=[]
 for role,ident in [('pub',32),('sub',33)]:
  peers=''
  if fixed:
   peers='<builtin><initialPeersList>'+''.join(f'<locator><udpv4><address>127.0.0.1</address><port>{7400+250*95+10+2*i}</port></udpv4></locator>' for i in (32,33))+'</initialPeersList></builtin>'
  xml=out/f'{fixed}_{role}.xml'
  if fixed:
   from sim.discovery import write_discovery_profile
   write_discovery_profile(xml,95,64)
   xml.write_text(xml.read_text().replace('</rtps>',f'<participantID>{ident}</participantID></rtps>'))
  else:
   xml.write_text(f'<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><participant profile_name="probe" is_default_profile="true"><rtps><participantID>{ident}</participantID></rtps></participant></profiles>')
  env=os.environ.copy();env.update(ROS_DOMAIN_ID='95',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',FASTRTPS_DEFAULT_PROFILES_FILE=str(xml),FASTDDS_DEFAULT_PROFILES_FILE=str(xml))
  processes.append(subprocess.Popen([sys.executable,__file__,role],env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True))
 outputs=[p.communicate(timeout=15)[0] for p in processes]
 results[str(fixed)]=outputs;print(fixed,outputs,flush=True)
(out/'result.json').write_text(json.dumps(results,indent=2)+'\n')

assert json.loads(results["False"][1].strip())["received"]==0
assert json.loads(results["True"][1].strip())["received"]>0
