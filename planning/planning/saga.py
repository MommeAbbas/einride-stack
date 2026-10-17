"""Own-truck-only Saga polling and normalisation; never reads public score or simulator audit."""
import requests


def normalise(data):
    route=data.get('route'); result=dict(truck_id=data['truck_id'],event_state=data['event_state'],has_route=route is not None)
    if route:
        status=route['status']; source=route['source']['tag_id']; destination=route['destination']['tag_id']
        result.update(route_id=route['route_id'],route_status=status,source_dock=source,destination_dock=destination,
                      next_dock=route['next_dock']['tag_id'] if route.get('next_dock') else -1,
                      held_docks=[source,destination] if status in ('loading_pending','loading_in_progress') else [destination])
    return result


class SagaClient:
    def __init__(self,url,token,timeout): self.url=url.rstrip('/'); self.token=token; self.timeout=timeout
    def poll(self):
        response=requests.get(self.url+'/api/v1/route',headers={'Authorization':'Bearer '+self.token},timeout=self.timeout)
        response.raise_for_status(); return normalise(response.json())
