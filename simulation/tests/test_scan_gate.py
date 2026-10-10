import itertools

from qc_warehouse.autonomy.frames import BasePose
from qc_warehouse.autonomy.scan_gate import ScanGate

_clock = itertools.count(1)


def _offer(gate, pose):
    return gate.offer(pose, next(_clock))


def _gate():
    return ScanGate(min_distance=0.15, min_heading=0.15)


def test_first_scan_always_processed_then_warmup():
    gate = _gate()
    assert _offer(gate, BasePose(0, 0, 0))
    # scans 2-4 are rejected for stabilisation even with large motion
    assert not any(_offer(gate, BasePose(1.0 * k, 0, 0)) for k in range(1, 4))
    assert _offer(gate, BasePose(5.0, 0, 0))


def _warm(gate):
    _offer(gate, BasePose(0, 0, 0))
    for _ in range(3):
        _offer(gate, BasePose(0, 0, 0))


def test_distance_or_heading_triggers():
    gate = _gate()
    _warm(gate)
    assert not _offer(gate, BasePose(0.10, 0, 0))
    assert _offer(gate, BasePose(0.16, 0, 0))
    assert not _offer(gate, BasePose(0.16, 0, 0.10))
    assert _offer(gate, BasePose(0.16, 0, 0.26))


def test_heading_wraps():
    gate = _gate()
    _warm(gate)
    _offer(gate, BasePose(1.0, 0, 3.1))   # processed (distance)
    assert not _offer(gate, BasePose(1.0, 0, -3.1))  # only 0.08 rad across the wrap


def test_resync_moves_reference_pose_but_ignores_stale_acks():
    gate = _gate()
    _warm(gate)
    gate.resync(BasePose(1.0, 0, 0), 10**9)
    assert not _offer(gate, BasePose(1.1, 0, 0))
    gate.resync(BasePose(5.0, 0, 0), 1)  # older than the reference: ignored
    assert gate.reference[0] == BasePose(1.0, 0, 0)


def test_withdraw_only_undoes_its_own_prediction():
    gate = ScanGate(min_distance=0.15, min_heading=0.15)
    for k in range(4):
        gate.offer(BasePose(0, 0, 0), k)
    before = gate.reference
    assert gate.offer(BasePose(1.0, 0, 0), 10)
    gate.withdraw(10, before)
    assert gate.reference == before
    assert gate.offer(BasePose(1.0, 0, 0), 11)
    gate.resync(BasePose(1.2, 0, 0), 12)        # a newer ack arrived meanwhile
    gate.withdraw(11, before)                     # must not roll back past it
    assert gate.reference == (BasePose(1.2, 0, 0), 12)
