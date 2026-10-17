"""Timed-risk request policy only; all paths/recoveries/turn execution stay in Nav2."""


class RiskGate:
    def __init__(self,wait_limit,resume_window,look_cooldown,receipt_timeout):
        self.wait_limit=wait_limit;self.resume_window=resume_window
        self.look_cooldown=look_cooldown;self.receipt_timeout=receipt_timeout
        self.reset()

    def reset(self):
        self.state='DRIVE';self.since=None;self.resume_until=0.;self.last_look=-float('inf');self.heading=None

    def resume(self,now):
        self.state='DRIVE';self.since=None;self.resume_until=now+self.resume_window
        return 'DRIVE','Risk: resume for Nav2 replan',None

    def step(self,now,high,heading=None,look_terminal=None,confirmed=False,completion=None):
        if self.state=='LOOK':
            if look_terminal is None:return 'LOOK','Risk: observe uncertain region',self.heading
            if look_terminal and confirmed:
                self.last_look=now
                return self.resume(now)
            if look_terminal and completion is not None and now-completion<self.receipt_timeout:
                return 'LOOK','Risk: observe uncertain region',self.heading
            self.last_look=now;self.state='WAIT';self.since=now
        if self.state=='WAIT':
            if not high or now-self.since>=self.wait_limit:return self.resume(now)
            return 'WAIT','Risk: bounded hold',None
        if now<self.resume_until or not high:return 'DRIVE','Risk: clear or replan window',None
        if heading is not None and now-self.last_look>=self.look_cooldown:
            self.state='LOOK';self.heading=heading;self.last_look=now
            return 'LOOK','Risk: observe uncertain region',self.heading
        self.state='WAIT';self.since=now
        return 'WAIT','Risk: bounded hold',None
