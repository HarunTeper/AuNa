#!/usr/bin/env python3

import os
import tempfile
from typing import Dict
import typing
import ruamel.yaml
from ament_index_python.packages import get_package_share_directory

# TODO move to auna_common

# Returns a loaded yaml as dictionaries
# Input:
#   Subfolder: subfolder in /package_name/config
#   filename: filename in /package_name/config/subfolder


def get_yaml(package_name, subfolder, filename):

    pkg_dir = get_package_share_directory(package_name)
    config_file_dir = os.path.join(pkg_dir, 'config', subfolder)

    with open(config_file_dir+'/'+filename+".yaml", "r") as stream:
        try:
            return ruamel.yaml.safe_load(stream)
        except ruamel.yaml.YAMLError as exc:
            print(exc)

# Returns values from yaml file.
# Input:
#   filename: filename in /package_name/config/subfolder
#   key: List of key values to search


def get_yaml_value(package_name, subfolder, filename, keys):

    pkg_dir = get_package_share_directory(package_name)
    config_file_dir = os.path.join(pkg_dir, 'config', subfolder)

    with open(config_file_dir+'/'+filename+".yaml", "r") as stream:
        try:
            yaml = ruamel.yaml.safe_load(stream)
            if not isinstance(yaml, dict):
                raise AttributeError('keys_exists() expects dict as first argument.')
            if len(keys) == 0:
                raise AttributeError('keys_exists() expects at least two arguments, one given.')
            yaml_content = yaml
            for key in keys:
                try:
                    yaml_content = yaml_content[key]
                except KeyError:
                    return None
            return yaml_content
        except ruamel.yaml.YAMLError as error:
            print(error)

# Creates a temporary yaml file in tmp and returns the path
# Input:
#   yaml_file: Yaml file as dictionaries


def get_temp_file(yaml_file: ruamel.yaml):
    rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
    yaml = ruamel.yaml.YAML()
    yaml.boolean_representation = ['False','True']
    yaml.dump(yaml_file, rewritten_yaml,)
    rewritten_yaml.close()
    return rewritten_yaml.name

# Substitutes the values of the keys in substitutions if the keys are in the yaml_file
# Input:
#   yaml_file: Yaml file as dictionaries
#   substitutions: Dictionary of substitutions


def substitute_values(yaml_file: dict, substitutions: Dict[str, typing.Any]):
    for key in yaml_file:
        if isinstance(yaml_file[key], dict):
            substitute_values(yaml_file[key], substitutions)
        else:
            for sub in substitutions:
                if sub == key:
                    yaml_file[sub] = substitutions[sub]
    return yaml_file

# Inserts a namespace in the given yaml_file
# Input:
#   yaml_file: Yaml file as dictionaries
#   namespace: Namespace string


def insert_namespace(yaml_file: dict, namespace: str):
    for key in yaml_file:
        if isinstance(yaml_file[key], dict):
            insert_namespace(yaml_file[key], namespace)
        else:
            if isinstance(yaml_file[key], str) and '$(NAMESPACE)$' in yaml_file[key]:
                yaml_file[key] = yaml_file[key].replace('$(NAMESPACE)$', namespace)
    return yaml_file
