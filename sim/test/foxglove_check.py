#!/usr/bin/env python3
"""Read actual ROS CDR payloads through Foxglove WebSocket (no external Python deps).
This verifies bridge schemas/data, not a claim that a human inspected the GUI.
"""
import argparse
import base64
import json
import os
import socket
import struct
import time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class WebSocket:
    def __init__(self,host,port,protocol=None,path='/'):
        self.sock=socket.create_connection((host,port),timeout=5);self.buffer=b''
        key=base64.b64encode(os.urandom(16)).decode()
        request=f'GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n'
        if protocol:request+=f'Sec-WebSocket-Protocol: {protocol}\r\n'
        self.sock.sendall((request+'\r\n').encode())
        while b'\r\n\r\n' not in self.buffer:self.buffer+=self.sock.recv(65536)
        headers,self.buffer=self.buffer.split(b'\r\n\r\n',1)
        if b' 101 ' not in headers.split(b'\r\n')[0]:raise RuntimeError(headers.decode())
    def read(self,n):
        while len(self.buffer)<n:
            data=self.sock.recv(max(65536,n-len(self.buffer)))
            if not data:raise EOFError('WebSocket closed')
            self.buffer+=data
        value,self.buffer=self.buffer[:n],self.buffer[n:];return value
    def send(self,message,opcode=1):
        body=json.dumps(message).encode() if isinstance(message,dict) else message
        mask=os.urandom(4);length=len(body)
        header=bytes([128|opcode,128|length]) if length<126 else bytes([128|opcode,128|126])+struct.pack('!H',length) if length<65536 else bytes([128|opcode,128|127])+struct.pack('!Q',length)
        self.sock.sendall(header+mask+bytes(x^mask[i%4] for i,x in enumerate(body)))
    def receive(self):
        first,second=self.read(2);size=second&127
        if size==126:size=struct.unpack('!H',self.read(2))[0]
        elif size==127:size=struct.unpack('!Q',self.read(8))[0]
        mask=self.read(4) if second&128 else None;body=self.read(size)
        if mask:body=bytes(x^mask[i%4] for i,x in enumerate(body))
        return first&15,body


def check(port,topics,duration):
    ws=WebSocket('127.0.0.1',port,'foxglove.sdk.v1, foxglove.websocket.v1')
    channels={};subscriptions={};counts={topic:0 for topic in topics};details={}
    end=time.monotonic()+duration
    try:
        while time.monotonic()<end and not all(counts.values()):
            try:opcode,body=ws.receive()
            except socket.timeout:continue
            if opcode==9:ws.send(body,10);continue
            if opcode==1:
                message=json.loads(body)
                if message.get('op')=='advertise':
                    new=[]
                    for c in message['channels']:
                        if c['topic'] in counts and c['id'] not in channels:
                            assert c['encoding']=='cdr',c
                            channels[c['id']]=c;ident=len(subscriptions)+1
                            subscriptions[ident]=c;new.append({'id':ident,'channelId':c['id']})
                    if new:ws.send({'op':'subscribe','subscriptions':new})
            elif opcode==2 and body and body[0]==1:
                ident=struct.unpack('<I',body[1:5])[0];c=subscriptions[ident]
                decoded=deserialize_message(body[13:],get_message(c['schemaName']))
                counts[c['topic']]+=1;details[c['topic']]={'schema':c['schemaName'],'bytes':len(body)-13}
                if hasattr(decoded,'markers'):details[c['topic']]['markers']=len(decoded.markers)
                if hasattr(decoded,'ranges'):details[c['topic']]['rays']=len(decoded.ranges)
                if hasattr(decoded,'format'):details[c['topic']]['image_format']=decoded.format
    finally:ws.sock.close()
    result={'received':counts,'details':details,'passed':all(counts.values())}
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8765);p.add_argument('--duration',type=float,default=30);p.add_argument('--output');p.add_argument('topics',nargs='+');a=p.parse_args()
    result=check(a.port,a.topics,a.duration);encoded=json.dumps(result,indent=2)
    if a.output:
        from pathlib import Path
        Path(a.output).write_text(encoded+'\n')
    print(encoded)
    raise SystemExit(0 if result['passed'] else 1)
