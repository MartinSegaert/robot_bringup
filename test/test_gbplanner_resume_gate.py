import pathlib
import sys


SCRIPT_DIR = pathlib.Path(__file__).parents[1] / 'scripts' / 'ros2'
sys.path.insert(0, str(SCRIPT_DIR))

from gbplanner_resume_gate import GbplannerResumeGate


def test_gate_is_open_before_a_resume_transition():
    gate = GbplannerResumeGate(0.1)

    assert gate.command_allowed(0)


def test_gate_rejects_old_path_and_waits_after_fresh_path():
    gate = GbplannerResumeGate(0.1)
    gate.start(1_000_000_000)

    assert not gate.observe_path(999_999_999, 1_010_000_000)
    assert not gate.observe_path(1_000_000_000, 1_010_000_000)
    assert not gate.command_allowed(2_000_000_000)

    assert gate.observe_path(1_000_000_001, 2_000_000_000)
    assert not gate.command_allowed(2_099_999_999)
    assert gate.command_allowed(2_100_000_000)
    assert gate.command_allowed(2_100_000_001)


def test_cancel_opens_gate_and_discards_latched_path():
    gate = GbplannerResumeGate(0.1)
    gate.start(10)
    assert gate.observe_path(11, 20)

    gate.cancel()

    assert gate.command_allowed(20)
    assert not gate.waiting
