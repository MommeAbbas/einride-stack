import xml.etree.ElementTree as ET
import pytest
from sim.discovery import write_discovery_profile


def test_explicit_peers_cover_late_participants_on_localhost(tmp_path):
    path=tmp_path/'discovery.xml';write_discovery_profile(path,88,64)
    root=ET.parse(path)
    assert {e.text for e in root.iter() if e.tag.endswith('address')}=={'127.0.0.1'}
    ports=[int(e.text) for e in root.iter() if e.tag.endswith('port')]
    assert ports==list(range(29410,29538,2))
    assert 29474 in ports and 29478 in ports # Actual Planning/SLAM late-peer ports.


def test_invalid_capacity_rejected(tmp_path):
    for domain,capacity in [(88,0),(88,121),(232,64),(-1,64)]:
        with pytest.raises(ValueError):write_discovery_profile(tmp_path/'bad.xml',domain,capacity)
