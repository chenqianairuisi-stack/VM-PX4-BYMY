#!/usr/bin/env python3
"""Generate the flight world with the same PX4 vehicle but no balloon visuals."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

root = Path(__file__).resolve().parents[1]
tree = ET.parse(root/'worlds/dual_uav.world')
world = tree.getroot().find('world')
for include in world.findall('include'):
    if include.findtext('name') == 'target':
        model = ET.parse(root/'models/fpv_target/fpv_target.sdf').getroot().find('model')
        model.set('name', 'target')
        model.insert(0, include.find('pose'))
        for link in model.findall('link'):
            for visual in list(link.findall('visual')):
                if visual.get('name', '').startswith('tracking_'):
                    link.remove(visual)
        world.remove(include)
        world.insert(2, model)
for plugin in list(world.findall('plugin')):
    if plugin.get('name') == 'balloon_contact':
        world.remove(plugin)
if len(sys.argv) > 2 and sys.argv[2] == 'virtual_contact':
    config = yaml.safe_load((root/'config/drone_virtual_contact.yaml').read_text())
    threshold = config['safe_visual_follow']['ros__parameters']['virtual_contact_threshold']
    plugin = ET.SubElement(world, 'plugin', {
        'name': 'virtual_contact', 'filename': 'libvirtual_contact.so',
    })
    ET.SubElement(plugin, 'threshold').text = str(threshold)
ET.indent(tree)
tree.write(sys.argv[1], encoding='unicode', xml_declaration=True)
