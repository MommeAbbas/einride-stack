"""Capture-time ordered queue, watermarks and bounded sensor lag."""
import heapq


class MeasurementQueue:
    def __init__(self, sensors, lag):
        self.watermarks={s:float('-inf') for s in sensors}; self.lag=lag
        self.pending=[]; self.sequence=0; self.processed=float('-inf'); self.dropped=0

    def push(self,sensor,stamp,value):
        self.watermarks[sensor]=max(self.watermarks.get(sensor,float('-inf')),stamp)
        if stamp<self.processed:
            self.dropped+=1; return False
        self.sequence+=1; heapq.heappush(self.pending,(stamp,self.sequence,sensor,value)); return True

    def ready(self,now):
        threshold=max(min(self.watermarks.values()),now-self.lag)
        while self.pending and self.pending[0][0]<=threshold:
            stamp,_,sensor,value=heapq.heappop(self.pending)
            if stamp<self.processed: self.dropped+=1; continue
            self.processed=stamp
            yield stamp,sensor,value
