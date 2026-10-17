"""Recorded integration evidence, including checks that duration alone cannot prove.

Uses existing configured thresholds. Simulator truth is evaluation-only. Risk pairs
are observational before/after values, not a causal or same-path ablation.
"""
import math
from area_check import messages,seconds
from interfaces.msg import CoordinatorRequest


def audit(directory,planning):
    last_risk=None;last_truth=None;distance=0.;max_speed=0.;first=None;last=0.
    reasons={};requests={};looks={};pairs=[];pending=[];risk_waits=[];wait_start=None
    event_states=set();null_hold=False;drive_goals=set();guard_stops=0;resume_after_look=0
    latest_goal=None;last_confirmed=None;violations=[];cmd_guard=None
    guard_start=None;guard_holds=[];guard_wait_start=None;guard_waits=[]
    for topic,m,t in messages(directory,['/planning/risk','/planning/behaviour',
            '/control/request','/control/status','/perception/health','/planning/saga_state',
            '/sim/truth','/cmd_vel','/control/collision_state']):
        if first is None:first=t
        last=t
        if topic=='/planning/risk':
            last_risk=m.data
            # Planning publishes the request before its risk within the same tick.
            # Use that tick's risk, not the previous tick's value.
            for item in looks.values():
                if item['before'] is None and t>=item['start']:item['before']=m.data
            for item in pending:
                pairs.append({**item,'after':m.data,'after_time':t})
            pending=[]
        elif topic=='/planning/saga_state':
            event_states.add(m.event_state);null_hold|=m.available and m.event_state=='running' and m.next_dock<0
        elif topic=='/planning/behaviour':
            reason=m.data.split(';')[0];reasons[reason]=reasons.get(reason,0)+1
            guard_wait=reason.startswith('WAIT: Collision guard')
            if guard_wait and guard_wait_start is None:guard_wait_start=t
            if not guard_wait and guard_wait_start is not None:
                guard_waits.append(t-guard_wait_start);guard_wait_start=None
            risk_wait=reason.startswith('WAIT: Risk:')
            if risk_wait and wait_start is None:wait_start=t
            if not risk_wait and wait_start is not None:
                risk_waits.append(t-wait_start);wait_start=None
        elif topic=='/control/request':
            latest_goal=m.goal_id
            if m.request_id not in requests:
                requests[m.request_id]=m
                if m.mode==m.LOOK:looks[m.request_id]={'before':None,'start':t,'goal':m.goal_id,'completion':None,'confirmed':False}
                if m.mode==m.DRIVE:
                    drive_goals.add(m.goal_id)
                    if last_confirmed is not None and last_confirmed[0]==m.goal_id:
                        resume_after_look+=1;last_confirmed=None
        elif topic=='/control/status' and m.mode==CoordinatorRequest.LOOK and m.status==m.SUCCEEDED:
            if m.request_id in looks:looks[m.request_id]['completion']=seconds(m.completion_stamp)
        elif topic=='/perception/health' and m.has_camera_observation:
            receipt=m.last_camera_observation
            if receipt.processed and receipt.negative_evidence_applied:
                capture=seconds(receipt.header.stamp);prediction=seconds(receipt.predictions_after_frame)
                for ident,item in looks.items():
                    if not item['confirmed'] and item['completion'] is not None and item['completion']<capture<prediction and requests[ident].map_revision==receipt.map_revision:
                        item['confirmed']=True
                        if item['goal']==latest_goal:
                            last_confirmed=(item['goal'],t)
                            if item['before'] is not None:pending.append({'request_id':ident,'before':item['before'],'completion':item['completion'],'capture':capture})
        elif topic=='/sim/truth':
            ego=next(r for r in m.robots if r.robot_id==0);xy=(ego.pose.position.x,ego.pose.position.y)
            if last_truth is not None:distance+=math.dist(xy,last_truth)
            last_truth=xy
        elif topic=='/control/collision_state':
            cmd_guard=m.action_type
            guard_stops+=m.action_type==m.STOP
            if cmd_guard==m.STOP and guard_start is None:guard_start=t
            elif cmd_guard!=m.STOP and guard_start is not None:
                guard_holds.append(t-guard_start);guard_start=None
        elif topic=='/cmd_vel':
            max_speed=max(max_speed,abs(m.linear.x))
            if cmd_guard==1 and (m.linear.x!=0. or m.angular.z!=0.):violations.append(t)
    if wait_start is not None:risk_waits.append(last-wait_start)
    if guard_start is not None:guard_holds.append(last-guard_start)
    if guard_wait_start is not None:guard_waits.append(last-guard_wait_start)
    return dict(distance_travelled_m=distance,max_command_speed_mps=max_speed,
        drive_goals=sorted(drive_goals),event_states=sorted(event_states),running_null_goal_seen=null_hold,
        behaviour_samples=reasons,confirmed_looks=sum(i['confirmed'] for i in looks.values()),
        drive_resumptions_after_confirmed_look=resume_after_look,risk_before_after_look=pairs,
        risk_decreased_after_look=sum(p['after']<p['before'] for p in pairs),
        longest_continuous_risk_wait_s=max(risk_waits,default=0.),
        configured_deadlock_timeout_s=planning['deadlock_timeout'],guard_stop_samples=guard_stops,
        longest_guard_stop_s=max(guard_holds,default=0.),
        longest_continuous_guard_wait_s=max(guard_waits,default=0.),
        nonzero_commands_after_guard_stop=violations,
        limitations='Before/after risks may use refreshed paths. LOOK interruptions can split a risk wait; review repeated cycles and route progress separately.')
