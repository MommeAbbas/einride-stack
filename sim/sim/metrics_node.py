"""Write measured metrics; unavailable area outputs remain blank, never fabricated zeros."""
import csv
from collections import deque
from pathlib import Path
import math
import numpy as np
from scipy.optimize import linear_sum_assignment
import rclpy.time
from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import Float64
from interfaces.msg import RobotTruthArray, CollisionEvent, SimRouteEvent, Predictions, CoordinatorStatus, PerceptionHealth
from vision_msgs.msg import Detection2DArray
from .history import PoseHistory
from .detector_node import pose_matrix
from .detector import transform
from .metrics import interpolate_points, mixture_moments, ellipse_hits, look_confirmed
from .ros_support import SimNode, seconds, run


class MetricsNode(SimNode):
    def __init__(self):
        super().__init__('sim_metrics')
        self.directory=Path(self.p('run_dir')); self.directory.mkdir(parents=True,exist_ok=True)
        self.history=PoseHistory(self.p('metric_history_seconds'))
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.routes={}; self.targets={}; self.contacts=[]; self.minimum=math.inf
        self.look_success=set();self.look_failed=set();self.look_confirmed_ids=set();self.pending_looks={}
        self.waiting=None; self.looks=None; self.look_ids=set(); self.mode=None; self.last_tick=None
        self.frame_receipts=deque();self.split_errors={k:[] for k in ('detection_frame','coasting')};self.split_hits={k:[] for k in self.split_errors}
        self.errors=[]; self.hits=[]; self.future_errors={1:[],2:[],3:[]}; self.pending=[]; self.queued_predictions=[]
        self.dock_results=[]; self.planning=[]; self.truth_count=0; self.detector_frames=0; self.detections=0
        self.invalid_predictions=0; self.missing_prediction_tf=0
        self.fields=['sim_time','traffic_count','route_seed','routes_ego','routes_fleet','score_ego','score_fleet',
                     'collisions_ego','collisions_fleet','minor_collisions','major_collisions','forced_collisions',
                     'minimum_separation_m','waiting_time_s','looks','looks_succeeded','looks_failed','looks_confirmed','perception_position_rmse_m',
                     'calibration_samples','inside_1sigma','expected_1sigma','inside_2sigma','expected_2sigma',
                     'prediction_rmse_1s_m','prediction_rmse_2s_m','prediction_rmse_3s_m',
                     'prediction_samples_1s','prediction_samples_2s','prediction_samples_3s',
                     'dock_guess_accuracy','dock_guess_samples','planning_time_ms','truth_count',
                     'detector_frames','detections','invalid_predictions','missing_prediction_tf']
        self.fields += [f'{kind}_{metric}' for kind in self.split_errors for metric in ('samples','position_rmse_m','inside_1sigma','inside_2sigma')]
        self.file=(self.directory/'metrics.csv').open('w',newline=''); self.writer=csv.DictWriter(self.file,fieldnames=self.fields); self.writer.writeheader()
        self.audit_file=(self.directory/'events.csv').open('w',newline='')
        self.audit=csv.writer(self.audit_file); self.audit.writerow(['sim_time','kind','truck_or_robot','route_or_peer','action_or_severity','dwell_sim_s','dwell_wall_s','relative_speed_mps','forced_test'])
        self.create_subscription(RobotTruthArray,'/sim/truth',self.truth,self.p('qos_depth'))
        self.create_subscription(CollisionEvent,'/sim/collisions',self.collision,self.p('qos_depth'))
        self.create_subscription(SimRouteEvent,'/sim/routes',self.route,self.p('qos_depth'))
        self.create_subscription(Float64,'/sim/minimum_separation',self.distance,self.p('qos_depth'))
        self.create_subscription(CoordinatorStatus,'/control/status',self.control,self.p('qos_depth'))
        self.create_subscription(PerceptionHealth,'/perception/health',self.health,self.p('qos_depth'))
        self.create_subscription(Predictions,'/perception/predictions',self.prediction,self.p('qos_depth'))
        self.create_subscription(Float64,'/planning/time_ms',lambda m:self.planning.append(m.data),self.p('qos_depth'))
        self.create_subscription(Detection2DArray,'/sim/detections',self.detection,self.p('qos_depth'))
        self.create_timer(1/self.p('metrics_rate'),self.tick)

    def truth(self,message):
        self.truth_count=len(message.robots)
        now=seconds(message.header.stamp)
        self.history.add(now,{r.robot_id:pose_matrix(r.pose) for r in message.robots})
        waiting=[]
        for prediction in self.queued_predictions:
            if seconds(prediction.header.stamp)+self.p('metric_match_tolerance')<=now:
                self.evaluate_prediction(prediction)
            else:
                waiting.append(prediction)
        self.queued_predictions=waiting
        pending=[]
        for stamp,robot_id,mean,horizon in self.pending:
            if stamp>now:
                pending.append((stamp,robot_id,mean,horizon)); continue
            poses=self.history.at(stamp,self.p('metric_match_tolerance'))
            if poses and robot_id in poses:
                self.future_errors[horizon].append(float(np.sum((poses[robot_id][:2,3]-mean)**2)))
        self.pending=pending

    def collision(self,message):
        if any(e.event_id==message.event_id for e in self.contacts):
            return
        self.contacts.append(message)
        self.audit.writerow([seconds(message.header.stamp),'collision',message.first_robot,message.second_robot,
                             message.severity,'','',message.relative_speed if message.speed_valid else '',message.forced_test]); self.audit_file.flush()

    def route(self,message):
        self.routes[message.truck_id]=message.completed_routes
        self.targets[message.truck_id-1]=message.source_dock if message.status in ('loading_pending','loading_in_progress') else message.destination_dock
        if message.action!='snapshot':
            self.audit.writerow([seconds(message.header.stamp),'route',message.truck_id,message.route_id,
                                 message.action,message.dwell_sim,message.dwell_wall,'','']); self.audit_file.flush()

    def distance(self,message):
        self.minimum=min(self.minimum,message.data)

    def control(self,message):
        self.mode=message.mode
        if message.reason.startswith(('ENCOUNTER WAIT','ENCOUNTER REPLAN','ENCOUNTER READY','Collision guard STOP')) or (message.reason.startswith('ENCOUNTER BACKUP') and message.stopped):self.mode=0
        if self.waiting is None:
            self.waiting=0.; self.looks=0
        if message.mode==2 and message.status in (message.ACKNOWLEDGED,message.ACTIVE,message.SUCCEEDED) and message.request_id not in self.look_ids:
            self.look_ids.add(message.request_id); self.looks+=1
        if message.mode==2 and message.terminal:
            if message.status==message.SUCCEEDED:
                self.look_success.add(message.request_id)
                if message.request_id not in self.look_confirmed_ids:self.pending_looks[message.request_id]=(seconds(message.completion_stamp),message.map_revision)
            elif message.status in (message.FAILED,message.REJECTED,message.CANCELED):self.look_failed.add(message.request_id)

    def health(self,message):
        if not message.has_camera_observation:return
        receipt=message.last_camera_observation
        capture=seconds(receipt.header.stamp)
        if receipt.processed and (not self.frame_receipts or self.frame_receipts[-1][0]<capture):
            self.frame_receipts.append((capture,receipt.detection_count))
            while self.frame_receipts and capture-self.frame_receipts[0][0]>self.p('metric_history_seconds'):self.frame_receipts.popleft()
        for ident,(completion,revision) in list(self.pending_looks.items()):
            if revision==receipt.map_revision and look_confirmed(completion,seconds(receipt.header.stamp),receipt.processed,receipt.negative_evidence_applied,seconds(receipt.predictions_after_frame)):
                self.look_confirmed_ids.add(ident);del self.pending_looks[ident]

    def detection(self,message):
        self.detector_frames+=1; self.detections+=len(message.detections)

    def prediction(self,message):
        self.queued_predictions.append(message)

    def evaluate_prediction(self,message):
        stamp=seconds(message.header.stamp)
        poses=self.history.at(stamp,self.p('metric_match_tolerance'))
        if not poses or not any(key!=0 for key in poses):
            return
        # Evaluation-only map->world alignment from true ego pose and estimated map->base.
        try:
            tf=self.tf.lookup_transform(message.header.frame_id,'base_footprint',rclpy.time.Time.from_msg(message.header.stamp))
            t,q=tf.transform.translation,tf.transform.rotation
            world_map=poses[0]@np.linalg.inv(transform([t.x,t.y,t.z],[q.x,q.y,q.z,q.w]))
        except (TransformException,KeyError):
            self.missing_prediction_tf+=1; return
        rotation=world_map[:2,:2]; offset=world_map[:2,3]
        tracks=[]
        for robot in message.robots:
            groups=[*robot.groups,robot.leftover]
            parsed=[]
            for g in groups:
                points=[(seconds(p.stamp),rotation@np.array([p.x,p.y])+offset,
                         rotation@np.array(p.covariance).reshape(2,2)@rotation.T) for p in g.points]
                parsed.append((g.probability,g.destination_dock,points))
            current=[]
            for weight,_,points in parsed:
                moment=interpolate_points(points,stamp)
                if moment is None:
                    break
                current.append((weight,*moment))
            moments=mixture_moments(current,self.p('metric_probability_tolerance')) if len(current)==len(parsed) else None
            if moments is None:
                self.invalid_predictions+=1; continue
            tracks.append((parsed,*moments))
        ids=sorted(i for i in poses if i!=0)
        if not tracks:
            return
        costs=np.array([[np.linalg.norm(mean-poses[i][:2,3]) for i in ids] for _,mean,_ in tracks])
        rows,cols=linear_sum_assignment(costs)
        receipt=next((r for r in reversed(self.frame_receipts) if r[0]<=stamp),None)
        kind='detection_frame' if receipt and stamp-receipt[0]<=self.p('metric_detection_frame_max_age') and receipt[1]>0 else 'coasting'
        for row,col in zip(rows,cols):
            parsed,mean,cov=tracks[row]; robot_id=ids[col]; truth=poses[robot_id][:2,3]
            self.errors.append(float(np.sum((truth-mean)**2)))
            self.hits.append(ellipse_hits(truth,mean,cov,self.p('metric_covariance_epsilon'))[:2])
            self.split_errors[kind].append(self.errors[-1]);self.split_hits[kind].append(self.hits[-1])
            weight,dock,_=max(parsed,key=lambda g:g[0])
            if dock>=0 and robot_id in self.targets:
                self.dock_results.append(int(dock==self.targets[robot_id]))
            for horizon in (1,2,3):
                components=[]
                for weight,_,points in parsed:
                    moment=interpolate_points(points,stamp+horizon)
                    if moment is None:
                        break
                    components.append((weight,*moment))
                future=mixture_moments(components,self.p('metric_probability_tolerance')) if len(components)==len(parsed) else None
                if future:
                    self.pending.append((stamp+horizon,robot_id,future[0],horizon))

    def tick(self):
        now=self.now()
        if self.last_tick is not None and self.mode==0:
            self.waiting+=max(0,now-self.last_tick)
        self.last_tick=now
        valid=[e for e in self.contacts if not e.forced_test]
        ego=[e for e in valid if 0 in (e.first_robot,e.second_robot)]
        penalty=lambda events:sum(self.p('major_penalty') if e.severity==e.MAJOR else self.p('minor_penalty') for e in events)
        rmse=lambda values:math.sqrt(float(np.mean(values))) if values else ''
        row=dict(sim_time=now,traffic_count=self.p('traffic_count'),route_seed=self.p('route_seed'),
            routes_ego=self.routes.get(1,''),routes_fleet=sum(self.routes.values()) if self.routes else '',
            score_ego=self.routes[1]*self.p('delivery_points')-penalty(ego) if 1 in self.routes else '',
            score_fleet=sum(self.routes.values())*self.p('delivery_points')-2*penalty(valid) if self.routes else '',
            collisions_ego=len(ego),collisions_fleet=len(valid),minor_collisions=sum(e.severity==e.MINOR for e in valid),
            major_collisions=sum(e.severity==e.MAJOR for e in valid),forced_collisions=len(self.contacts)-len(valid),
            minimum_separation_m=self.minimum if math.isfinite(self.minimum) else '',waiting_time_s=self.waiting,
            looks=self.looks,looks_succeeded=len(self.look_success),looks_failed=len(self.look_failed),looks_confirmed=len(self.look_confirmed_ids),perception_position_rmse_m=rmse(self.errors),calibration_samples=len(self.hits),
            inside_1sigma=float(np.mean([h[0] for h in self.hits])) if self.hits else '',expected_1sigma=1-math.exp(-.5),
            inside_2sigma=float(np.mean([h[1] for h in self.hits])) if self.hits else '',expected_2sigma=1-math.exp(-2),
            dock_guess_accuracy=float(np.mean(self.dock_results)) if self.dock_results else '',dock_guess_samples=len(self.dock_results),
            planning_time_ms=float(np.mean(self.planning)) if self.planning else '',truth_count=self.truth_count,
            detector_frames=self.detector_frames,detections=self.detections,invalid_predictions=self.invalid_predictions,
            missing_prediction_tf=self.missing_prediction_tf)
        for h in (1,2,3):
            row[f'prediction_rmse_{h}s_m']=rmse(self.future_errors[h]); row[f'prediction_samples_{h}s']=len(self.future_errors[h])
        for kind,errors in self.split_errors.items():
            hits=self.split_hits[kind];row[kind+'_samples']=len(errors);row[kind+'_position_rmse_m']=rmse(errors)
            for i,label in enumerate(('1sigma','2sigma')):row[kind+'_inside_'+label]=float(np.mean([h[i] for h in hits])) if hits else ''
        self.writer.writerow(row); self.file.flush()

    def destroy_node(self):
        self.tick(); self.file.close(); self.audit_file.close()
        return super().destroy_node()


def main():
    run(MetricsNode)
