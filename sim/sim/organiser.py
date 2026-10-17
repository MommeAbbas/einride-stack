"""Two-clock dwell state; eligibility is independent of HTTP and ROS."""
from dataclasses import dataclass


@dataclass
class Dwell:
    route_id: int
    status: str
    sim_start: float
    wall_start: float


class Organiser:
    def __init__(self,dwell_seconds,wall_lock_seconds):
        self.dwell_seconds,self.wall_lock_seconds=dwell_seconds,wall_lock_seconds
        self.dwells={}

    def decide(self,truck,route,at_dock,sim_now,wall_now):
        status=route['status']
        if status in ('loading_pending','loading_complete'):
            self.dwells.pop(truck,None)
            return ('arrived_at_source' if status=='loading_pending' else 'arrived_at_destination') if at_dock else None
        if status not in ('loading_in_progress','unloading_in_progress'):
            return None
        entry=self.dwells.get(truck)
        if entry is None or entry.route_id!=route['route_id'] or entry.status!=status or sim_now<entry.sim_start:
            entry=Dwell(route['route_id'],status,sim_now,wall_now)
            self.dwells[truck]=entry
        if not at_dock:
            entry.sim_start=sim_now
            return None
        if sim_now-entry.sim_start>=self.dwell_seconds and wall_now-entry.wall_start>=self.wall_lock_seconds:
            return 'loading_complete' if status=='loading_in_progress' else 'unloading_complete'
        return None

    def acknowledged(self,truck,route,action,sim_now,wall_now):
        if action.startswith('arrived_at_'):
            status='loading_in_progress' if action=='arrived_at_source' else 'unloading_in_progress'
            self.dwells[truck]=Dwell(route['route_id'],status,sim_now,wall_now)


def target_dock(route):
    """loading_complete means drive to DESTINATION, despite its loading prefix."""
    return route['source'] if route['status'] in ('loading_pending','loading_in_progress') else route['destination']
