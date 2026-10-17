"""Configure installed Fast DDS explicit localhost peers for a full-stack run."""
import xml.etree.ElementTree as ET


def write_discovery_profile(path,domain,capacity):
    # RTPS standard well-known port mapping: PB + DG*domain + d1 + PG*participant.
    # These are protocol constants, not robot tuning: 7400, 250, 10, 2 [UDP ports].
    ports=[7400+250*domain+10+2*i for i in range(capacity)]
    if domain<0 or not 1<=capacity<=120 or ports[-1]>65535:
        raise ValueError('DDS participant capacity/domain exceeds disjoint UDP port range')
    root=ET.Element('profiles',xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles')
    participant=ET.SubElement(root,'participant',profile_name='autonomy_localhost',is_default_profile='true')
    rtps=ET.SubElement(participant,'rtps')
    peers=ET.SubElement(ET.SubElement(rtps,'builtin'),'initialPeersList')
    for port in ports:
        udp=ET.SubElement(ET.SubElement(peers,'locator'),'udpv4')
        ET.SubElement(udp,'address').text='127.0.0.1'
        ET.SubElement(udp,'port').text=str(port)
    ET.indent(root)
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
