#!/usr/bin/env python3
"""XML file load functions"""

import xml.etree.ElementTree as ET
import tempfile


def insert_namespace(xml_file_path, namespace):
    """Inserts namespace into xml file"""

    with open(xml_file_path, encoding='latin-1') as f:
        tree = ET.parse(f)
        root = tree.getroot()

        for elem in root.iter():
            for key, value in list(elem.attrib.items()):
                if namespace == "":
                    elem.attrib[key] = value.replace('$(NAMESPACE_FRAME)$', '')
                else:
                    elem.attrib[key] = value.replace('$(NAMESPACE_FRAME)$', namespace+'/')

    temp_file_path = tempfile.NamedTemporaryFile(mode='w', delete=False)
    tree.write(temp_file_path.name, encoding='utf-8', xml_declaration=True)
    temp_file_path.close()
    return temp_file_path.name
