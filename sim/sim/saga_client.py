"""Local Saga HTTP adapter. Never log authentication values or response bodies."""
import requests
from urllib.parse import urlparse


class SagaClient:
    def __init__(self,url,timeout,tokens=(),admin=None):
        if urlparse(url).hostname not in ('localhost','127.0.0.1','::1'):
            raise ValueError('Simulation organiser must use a loopback Saga server')
        self.url,self.timeout=url.rstrip('/'),timeout
        self.tokens=dict(tokens); self.admin=admin
        self.session=requests.Session()

    def route(self,truck):
        r=self.session.get(self.url+'/api/v1/route',
            headers={'Authorization':'Bearer '+self.tokens[truck]},timeout=self.timeout)
        r.raise_for_status(); return r.json()

    def status(self,truck):
        r=self.session.get(self.url+'/api/v1/status',
            headers={'Authorization':'Bearer '+self.tokens[truck]},timeout=self.timeout)
        r.raise_for_status(); return r.json()

    def action(self,truck,action,status):
        r=self.session.post(f'{self.url}/admin/trucks/{truck}/actions/{action}',
                            auth=self.admin,data={'expected_status':status},timeout=self.timeout)
        r.raise_for_status()
        # Saga deliberately returns HTTP 200 even on rejected/locked actions.
        return 'saga-error' not in r.headers.get('HX-Trigger','')

    def start(self):
        r=self.session.post(self.url+'/admin/event/start',auth=self.admin,timeout=self.timeout)
        r.raise_for_status(); return 'saga-error' not in r.headers.get('HX-Trigger','')
