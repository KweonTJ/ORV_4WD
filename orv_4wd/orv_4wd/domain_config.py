"""Shared domain selection for vehicle launch, bridge launch and shell tools."""
from pathlib import Path

import yaml


def default_domain_config():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('orv_bringup')) / 'config/domain_bridge.yaml'


def domain_id(value):
    # Linux DDS domain range; do not silently truncate floats or accept YAML bools.
    if isinstance(value, str) and value.isascii() and value.isdecimal():
        value = int(value)
    if type(value) is not int or not 0 <= value <= 232:
        raise ValueError('domain ID must be an integer from 0 to 232')
    return value


def load_domains(config, vehicle='', operator=''):
    data = yaml.safe_load(Path(config).expanduser().read_text())
    if not isinstance(data, dict):
        raise ValueError('domain bridge config must be a YAML mapping')
    vehicle_id = domain_id(vehicle if vehicle != '' else data.get('from_domain'))
    operator_id = domain_id(operator if operator != '' else data.get('to_domain'))
    if vehicle_id == operator_id:
        raise ValueError('vehicle and operator domains must be different')
    return vehicle_id, operator_id
