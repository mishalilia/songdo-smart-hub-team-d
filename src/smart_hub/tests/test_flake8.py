# Author: Mikhail Li (12236295)
import multiprocessing

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # flake8 checks files in a process pool. Fork after the rclpy tests have started DDS
    # threads can deadlock the workers, so the pool uses "spawn".
    multiprocessing.set_start_method('spawn', force=True)
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
