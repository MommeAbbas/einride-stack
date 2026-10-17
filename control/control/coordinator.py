"""Pure request validation and stop/turn state machine; no mission decisions."""
import math


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


def validate(mode,frame,expires,now,goal,revision,path_revision,speed):
    if mode not in (0,1,2):return 'unknown mode'
    if expires<=now:return 'expired lease'
    if mode and frame!='map':return 'invalid frame'
    if mode==1 and (not goal or revision!=path_revision or not math.isfinite(speed) or speed<0):return 'invalid DRIVE'
    return ''


def turn_command(error,stopped,turning,elapsed,p):
    if elapsed>p['look_timeout_limit']:return 0.,'FAILED'
    if not turning and not stopped:return 0.,'STOPPING'
    if abs(error)<p['heading_tolerance']:return 0.,'SUCCEEDED' if stopped else 'STOPPING'
    return max(-p['look_turn_rate'],min(p['look_turn_rate'],p['look_gain']*error)),'ACTIVE'
