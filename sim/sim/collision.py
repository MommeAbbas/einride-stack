"""True planar footprint contact episodes, independent of belief or collision guard."""
from itertools import combinations
import math
from .geometry import rectangle, overlap, separation


class CollisionMonitor:
    def __init__(self,length,width,major_speed,release_distance):
        self.length,self.width=length,width
        self.major_speed,self.release_distance=major_speed,release_distance
        self.active=set(); self.counter=0

    def update(self,robots):
        events=[]; minimum=math.inf
        for a,b in combinations(sorted(robots),2):
            pa,va,valid_a=robots[a]; pb,vb,valid_b=robots[b]
            ra=rectangle(*pa,self.length,self.width); rb=rectangle(*pb,self.length,self.width)
            distance=separation(ra,rb); minimum=min(minimum,distance)
            key=(a,b)
            if overlap(ra,rb) and key not in self.active:
                self.active.add(key); self.counter+=1
                valid=valid_a and valid_b
                speed=math.hypot(va[0]-vb[0],va[1]-vb[1]) if valid else 0.
                events.append(dict(event_id=self.counter,first_robot=a,second_robot=b,
                                   relative_speed=speed,speed_valid=valid,
                                   severity=2 if valid and speed>=self.major_speed else 1))
            elif distance>self.release_distance:
                self.active.discard(key)
        return events,minimum
