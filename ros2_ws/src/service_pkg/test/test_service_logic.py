from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CLIENT_PATH = PACKAGE_ROOT / 'service_pkg' / 'service_client.py'
SERVER_PATH = PACKAGE_ROOT / 'service_pkg' / 'service_server.py'


def _load_module(path, module_name):
    spec = spec_from_file_location(module_name, path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


service_client = _load_module(CLIENT_PATH, 'service_pkg_service_client_src')
service_server = _load_module(SERVER_PATH, 'service_pkg_service_server_src')


def test_parse_toggle_value_defaults_true():
    assert service_client.parse_toggle_value(['service_client']) is True


def test_parse_toggle_value_true_variants():
    assert service_client.parse_toggle_value(['service_client', 'true']) is True
    assert service_client.parse_toggle_value(['service_client', '1']) is True
    assert service_client.parse_toggle_value(['service_client', 'yes']) is True
    assert service_client.parse_toggle_value(['service_client', 'on']) is True


def test_parse_toggle_value_false_variants():
    assert service_client.parse_toggle_value(['service_client', 'false']) is False
    assert service_client.parse_toggle_value(['service_client', '0']) is False
    assert service_client.parse_toggle_value(['service_client', 'no']) is False
    assert service_client.parse_toggle_value(['service_client', 'off']) is False


def test_parse_toggle_value_invalid_raises():
    with pytest.raises(ValueError, match='Invalid toggle value'):
        service_client.parse_toggle_value(['service_client', 'invalid'])


def test_build_toggle_message():
    assert service_server.build_toggle_message(True) == 'Talker enabled: True'
    assert service_server.build_toggle_message(False) == 'Talker enabled: False'
