import pytest

from orv_4wd.domain_config import domain_id, load_domains


@pytest.mark.parametrize('value', [True, False, None, -1, 233, 1.5, '1.5', '', 'abc'])
def test_invalid_domain_ids(value):
    with pytest.raises(ValueError):
        domain_id(value)


def test_domain_selection_and_collision(tmp_path):
    config = tmp_path / 'bridge.yaml'
    config.write_text('from_domain: 71\nto_domain: 72\n')
    assert load_domains(config) == (71, 72)
    assert load_domains(config, '73', '74') == (73, 74)
    with pytest.raises(ValueError, match='different'):
        load_domains(config, vehicle='72')
    config.write_text('from_domain: true\nto_domain: 72\n')
    with pytest.raises(ValueError):
        load_domains(config)
