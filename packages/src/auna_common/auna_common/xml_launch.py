#!/usr/bin/env python3

# Copyright 2025 Harun Teper
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


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
                    elem.attrib[key] = value.replace('$(NAMESPACE_FRAME)$', namespace + '/')

    temp_file_path = tempfile.NamedTemporaryFile(mode='w', delete=False)
    tree.write(temp_file_path.name, encoding='utf-8', xml_declaration=True)
    temp_file_path.close()
    return temp_file_path.name
