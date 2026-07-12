import numpy as np
import pytest

import config as cfg
from delta_robot_kinematics import DeltaKinematics
from delta_robot_integration import CameraInterface, BaseCoupling
from test_data_generator import generate_robot_frame_layout

# Not trying to re-verify the whole workspace here, just enough to catch
# the "someone edited the kinematics and it quietly stopped agreeing with
# itself" case in one command instead of another full audit.

kin = DeltaKinematics()


def test_roundtrip_error_is_negligible():
    for pt in [(20, 30, -150), (-90, 60, -160), (80, 20, -180), (10, -90, -200)]:
        angles = kin.inverse(*pt)
        back = kin.forward(*angles)
        err = np.linalg.norm(np.array(pt) - np.array(back))
        assert err < 1e-9


def test_shallow_point_is_unreachable():
    # z=-60 puts the platform too close to the base for this geometry -
    # legs can't fold that far. Should come back all None, not raise.
    assert kin.inverse(0, 0, -60) == (None, None, None)


def test_home_position_is_reachable():
    angles = kin.inverse(0, 0, -150)
    assert all(a is not None for a in angles)


def test_joint_limits_are_enforced():
    angles_checked = kin.inverse(0, 0, -150, enforce_limits=True)
    angles_raw = kin.inverse(0, 0, -150, enforce_limits=False)
    for checked in angles_checked:
        if checked is not None:
            assert cfg.THETA_MIN <= checked <= cfg.THETA_MAX
    assert angles_raw is not None  # sanity - raw solve still returns something


def test_camera_offset_applied_correctly():
    cam = CameraInterface(offset_x=10.0, offset_y=-5.0, offset_z=2.0)
    x, y, z = cam.to_robot_frame(0.0, 0.0, -170.0)
    assert (round(x, 6), round(y, 6), round(z, 6)) == (10.0, -5.0, -168.0)


def test_reposition_only_suggested_when_unreachable():
    coupling = BaseCoupling(kin)
    reachable_pt = (0.0, 0.0, -150.0)
    assert coupling.suggest_reposition(*reachable_pt) is None

    unreachable_pt = (300.0, 300.0, -170.0)
    dx, dy = coupling.suggest_reposition(*unreachable_pt)
    assert (dx, dy) != (0.0, 0.0)


def test_generated_layout_respects_min_separation():
    weeds = generate_robot_frame_layout(6, seed=5, z_fixed=-170)
    for i, (x1, y1, _) in enumerate(weeds):
        for x2, y2, _ in weeds[i + 1:]:
            assert np.hypot(x1 - x2, y1 - y2) >= cfg.MIN_WEED_SEPARATION - 1e-9


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
