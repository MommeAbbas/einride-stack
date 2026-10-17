"""Reactive tree conditions, pure decisions. Control always retains raw-sensor guard priority."""
from dataclasses import dataclass


@dataclass
class Situation:
    guard: bool=False
    map_ready: bool=False
    event_running: bool=False
    next_dock: int=-1
    at_dock: bool=False
    valid: bool=False
    risk: float=1.
    look_useful: bool=False
    dock_free: bool=False
    final_approach: bool=False


def decide(s,tolerance,look_enabled):
    if s.guard: return 'WAIT','Collision guard'
    if not s.map_ready: return 'WAIT','Map: exploration required'
    if not s.event_running: return 'WAIT','Event: hold'
    if s.next_dock<0: return 'WAIT','Mission: loading/unloading hold'
    if s.at_dock: return 'WAIT','Mission: wait for organiser'
    if not s.valid: return 'WAIT','Safe to proceed: invalid inputs'
    if s.risk>tolerance:
        if look_enabled and s.look_useful: return 'LOOK','Improve understanding'
        return 'WAIT','Risk: wait/yield and replan'
    if s.final_approach and not s.dock_free: return 'WAIT','Dock free: occupied or unknown'
    return 'DRIVE','Final docking' if s.final_approach else 'Navigate to approach point'
