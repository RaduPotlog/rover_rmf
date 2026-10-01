# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import pytest

from rover_rmf_fleet_adapter.application.actions import ACTION_HANDLERS, WaitHandler
from rover_rmf_fleet_adapter.domain.actions import MAX_WAIT_SEC, Wait
from rover_rmf_fleet_adapter.domain.model import DomainError


def test_wait_parses_the_dashboard_description():
    assert Wait.parse({'duration_sec': 30}) == Wait(30.0)
    assert Wait.parse({'duration_sec': 2.5}).duration_sec == 2.5


@pytest.mark.parametrize('description', [
    None, 30, 'x', {}, {'duration_sec': '30'}, {'duration_sec': True}, {'duration_sec': 0},
    {'duration_sec': -1}, {'duration_sec': float('nan')}, {'duration_sec': MAX_WAIT_SEC + 1},
])
def test_wait_refuses_bad_descriptions(description):
    with pytest.raises(DomainError):
        Wait.parse(description)


def test_wait_handler_is_done_at_the_deadline_not_before():
    handler = ACTION_HANDLERS['wait']({'duration_sec': 10}, 100.0, link=None)
    assert isinstance(handler, WaitHandler)
    assert not handler.tick(100.0)
    assert not handler.tick(109.9)
    assert handler.tick(110.0)
