#!/usr/bin/env python3
"""Yaml file load functions"""

import tempfile
from typing import Dict
import typing
import yaml


def get_yaml(file_path):
    """Opens yaml from file path"""
    with open(file_path, "rb") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)


def get_yaml_value(file_path, keys):
    """Gets value from yaml file using keys"""

    with open(file_path, "rb") as stream:
        try:
            yaml_file = yaml.safe_load(stream)
            if not isinstance(yaml_file, dict):
                raise AttributeError('keys_exists() expects dict as first argument.')
            if len(keys) == 0:
                raise AttributeError('keys_exists() expects at least two arguments, one given.')
            yaml_content = yaml_file
            for key in keys:
                try:
                    yaml_content = yaml_content[key]
                except KeyError:
                    return None
            return yaml_content
        except yaml.YAMLError as error:
            print(error)


def get_temp_file(yaml_file: yaml):
    """Return temp file path of cloned yaml file"""
    temp_file_path = tempfile.NamedTemporaryFile(mode='w', delete=False)
    yaml.safe_dump(yaml_file, temp_file_path)
    temp_file_path.close()
    return temp_file_path.name


def substitute_values(yaml_file: dict, substitutions: Dict[str, typing.Any]):
    """Substitures values in keys of substitutions if the keys are in the yaml_file"""
    for key in yaml_file:
        if isinstance(yaml_file[key], dict):
            substitute_values(yaml_file[key], substitutions)
        else:
            for sub in substitutions:
                if sub == key:
                    yaml_file[sub] = substitutions[sub]
    return yaml_file


def insert_namespace(yaml_file: dict, namespace: str):
    """Substitutes the tag '$(NAMESPACE)$' with the given namespace in the yaml"""
    for key in yaml_file:
        if isinstance(yaml_file[key], dict):
            insert_namespace(yaml_file[key], namespace)
        else:
            if namespace == '':
                if isinstance(yaml_file[key], str) and '$(NAMESPACE)$' in yaml_file[key]:
                    yaml_file[key] = yaml_file[key].replace('$(NAMESPACE)$', namespace)
                if isinstance(yaml_file[key], str) and '$(NAMESPACE_FRAME)$' in yaml_file[key]:
                    yaml_file[key] = yaml_file[key].replace('$(NAMESPACE_FRAME)$', namespace)
                if isinstance(yaml_file[key], str) and '$(NAMESPACE_TOPIC)$' in yaml_file[key]:
                    yaml_file[key] = yaml_file[key].replace('$(NAMESPACE_TOPIC)$', namespace)
            else:
                if isinstance(yaml_file[key], str) and '$(NAMESPACE)$' in yaml_file[key]:
                    yaml_file[key] = yaml_file[key].replace('$(NAMESPACE)$', namespace)
                if isinstance(yaml_file[key], str) and '$(NAMESPACE_FRAME)$' in yaml_file[key]:
                    yaml_file[key] = yaml_file[key].replace('$(NAMESPACE_FRAME)$', namespace+'/')
                if isinstance(yaml_file[key], str) and '$(NAMESPACE_TOPIC)$' in yaml_file[key]:
                    yaml_file[key] = yaml_file[key].replace('$(NAMESPACE_TOPIC)$', namespace)
                
    return yaml_file
