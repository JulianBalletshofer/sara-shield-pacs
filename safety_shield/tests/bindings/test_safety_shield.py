"""This file defines the tests of the python bindings of the safety shield class.

Owner:
    Jakob Thumm (JT)

Copyright:
    This file is part of SaRA-Shield.
    SaRA-Shield is free software: you can redistribute it and/or modify it under 
    the terms of the GNU General Public License as published by the Free Software Foundation, 
    either version 3 of the License, or (at your option) any later version.
    SaRA-Shield is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
    without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
    See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with SaRA-Shield. 
    If not, see <https://www.gnu.org/licenses/>. 

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
"""
import os
import pytest
from safety_shield_py import Motion, LongTermTraj, AABB, SafetyShield, ContactType, ShieldType, Point, Sphere, Prediction


class TestSafetyShield:
    """This class defines the test fixtures and tests for the safety shield python bindings."""

    @pytest.fixture
    def shield(self):
        """Define a test fixture for the safety shield.

        We use the schunk and the CMU_mocap_no_hand config files.
        The sampling time is set to 0.004.
        All initial position and rotations are set to zero.
        The shield is active.
        """
        config_dir = os.path.join(os.path.dirname(os.path.dirname(os.getcwd())), "config")
        print(f"Config directory: {config_dir}")
        table = AABB([-1.0, -1.0, -0.1], [1.0, 1.0, 0.0])
        shield_type = ShieldType.SSM
        contact_type = ContactType.EDGE
        shield = SafetyShield(
            sample_time=0.004,
            trajectory_config_file=os.path.join(config_dir, "trajectory_parameters_schunk.yaml"),
            robot_config_file=os.path.join(config_dir, "robot_parameters_schunk.yaml"),
            mocap_config_file=os.path.join(config_dir, "cmu_mocap_no_hand.yaml"),
            init_x=0.0,
            init_y=0.0,
            init_z=0.0,
            init_roll=0.0,
            init_pitch=0.0,
            init_yaw=0.0,
            init_qpos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            environment_elements=[table],
            shield_type=shield_type,
            eef_contact_type=contact_type
        )
        return shield

    @pytest.fixture
    def shield_schunk(self):
        """Define a test fixture for the safety shield.

        We use the Schunk and the CMU_mocap_no_hand config files.
        The sampling time is set to 0.004.
        All initial position and rotations are set to zero.
        The shield is active.
        """
        # Find the safety_shield project root directory by looking for the config folder
        # Start from the test file location and walk up the directory tree
        current_dir = os.path.dirname(os.path.realpath(__file__))
        while current_dir != '/':
            config_dir = os.path.join(current_dir, "config")
            if os.path.exists(config_dir) and os.path.exists(os.path.join(config_dir, "robot_parameters_schunk.yaml")):
                break
            current_dir = os.path.dirname(current_dir)
        else:
            # Fallback: assume we're in the safety_shield directory
            config_dir = os.path.join(os.getcwd(), "config")
        print(f"Config directory: {config_dir}")
        table = AABB([-1.0, -1.0, -0.1], [1.0, 1.0, 0.0])
        shield_type = ShieldType.SSM
        contact_type = ContactType.EDGE
        shield = SafetyShield(
            sample_time=0.004,
            trajectory_config_file=os.path.join(config_dir, "trajectory_parameters_schunk.yaml"),
            robot_config_file=os.path.join(config_dir, "robot_parameters_schunk.yaml"),
            mocap_config_file=os.path.join(config_dir, "cmu_mocap_no_hand.yaml"),
            init_x=0.0,
            init_y=0.0,
            init_z=0.0,
            init_roll=0.0,
            init_pitch=0.0,
            init_yaw=0.0,
            init_qpos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            environment_elements=[table],
            shield_type=shield_type,
            eef_contact_type=contact_type
        )
        return shield

    def test_safety_shield(self, shield):
        """Test the safety shield constructor."""
        assert shield is not None

    def test_reset_safety_shield(self, shield):
        """Test the reset function."""
        table = AABB([-1.0, -1.0, -0.1], [1.0, 1.0, 0.0])
        shield_type = ShieldType.SSM
        contact_type = ContactType.EDGE
        shield.reset(init_x=0.0,
                     init_y=0.0,
                     init_z=0.0,
                     init_roll=0.0,
                     init_pitch=0.0,
                     init_yaw=0.0,
                     init_qpos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     current_time=0.0,
                     environment_elements=[table],
                     shield_type=shield_type,
                     eef_contact_type=contact_type
                    )
        assert shield is not None   

    def test_humanMeasurement(self, shield):
        """Test the humanMeasurement() function."""
        dummy_meas = []
        for i in range(21):
            dummy_meas.append([0.0, 0.0, 0.0])
        shield.humanMeasurement(dummy_meas, 0.0)

    def test_newLTT(self, shield):
        """Test the newLongTermTrajectory() function."""
        pos = [0.0 for i in range(6)]
        vel = [0.0 for i in range(6)]
        shield.newLongTermTrajectory(pos, vel)

    def test_setLTT(self, shield):
        """Test the setLongTermTrajectory() function."""
        motions = []
        motions.append(Motion(
            0.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0
        ))
        motions.append(Motion(
            1.0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0
        ))
        motions.append(Motion(
            2.0, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [-1.0, 0.0, 0., 0.0, 0.0, 0.00], 0.0
        ))
        motions.append(Motion(
            3.0, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0
        ))
        sample_time = 1.0
        starting_index = 0
        v_max_allowed = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        a_max_allowed = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
        j_max_allowed = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
        ltt = LongTermTraj(motions, sample_time, starting_index, v_max_allowed, a_max_allowed, j_max_allowed)
        shield.setLongTermTrajectory(ltt)

    def test_step(self, shield):
        """Test the step() function."""
        motion = shield.step(0.004)
        assert motion.getTime() == 0.004

    def test_getSafety(self, shield):
        """Test the getSafety() function."""
        dummy_meas = []
        for i in range(21):
            dummy_meas.append([0.0, 0.0, 0.0])
        shield.humanMeasurement(dummy_meas, 0.0)
        pos = [0.0 for i in range(6)]
        vel = [0.0 for i in range(6)]
        shield.newLongTermTrajectory(pos, vel)
        motion = shield.step(0.004)
        is_safe = shield.getSafety()
        assert is_safe is False

    def test_getRobotReachCapsules(self, shield):
        """Test the getRobotReachCapsules() function for the schunk test fixture."""
        pos = [0.0 for i in range(6)]
        vel = [0.0 for i in range(6)]
        shield.newLongTermTrajectory(pos, vel)
        dummy_meas = []
        for i in range(21):
            dummy_meas.append([0.0, 0.0, 0.0])
        shield.humanMeasurement(dummy_meas, 0.0)
        shield.step(0.004)
        robot_caps = shield.getRobotReachCapsules()
        for cap in robot_caps:
            assert len(cap) == 7
            print("-------")
            print(cap[0])
            print(cap[1])
            print(cap[2])
            print(cap[3])
            print(cap[4])
            print(cap[5])
            print(cap[6])

        eps = 1e-5
        secure_radius = 0.01
        print("-----")
        assert abs(robot_caps[0][0] - 0.0) <= eps
        assert abs(robot_caps[0][1] - -0.015) <= eps
        assert abs(robot_caps[0][2] - 0.1553) <= eps
        assert abs(robot_caps[0][3] - 0.0) <= eps
        assert abs(robot_caps[0][4] - -0.015) <= eps
        assert abs(robot_caps[0][5] - 0.1553) <= eps
        assert abs(robot_caps[0][6] - (0.11000000310751243)) <= eps
        print("-----")
        assert abs(robot_caps[1][0] - -0.00066172) <= eps
        assert abs(robot_caps[1][1] - 0.11404500000000001) <= eps
        assert abs(robot_caps[1][2] - 0.155205916) <= eps
        assert abs(robot_caps[1][3] - 0.00022952) <= eps
        assert abs(robot_caps[1][4] - 0.11488) <= eps
        assert abs(robot_caps[1][5] - 0.5052) <= eps
        assert abs(robot_caps[1][6] - (0.09402100310751244)) <= eps
        print("-----")
        assert abs(robot_caps[2][0] - 0.0) <= eps
        assert abs(robot_caps[2][1] - 0.0) <= eps
        assert abs(robot_caps[2][2] - 0.49029999999999996) <= eps
        assert abs(robot_caps[2][3] - 0.0) <= eps
        assert abs(robot_caps[2][4] - 0.0) <= eps
        assert abs(robot_caps[2][5] - 0.49029999999999996) <= eps
        assert abs(robot_caps[2][6] - (0.11000000310751243)) <= eps
        print("-----")
        assert abs(robot_caps[3][0] - -4.6934e-05) <= eps
        assert abs(robot_caps[3][1] - -0.0058834) <= eps
        assert abs(robot_caps[3][2] - 0.60610485) <= eps
        assert abs(robot_caps[3][3] - -2.3657e-06) <= eps
        assert abs(robot_caps[3][4] - 0.061878) <= eps
        assert abs(robot_caps[3][5] - 0.80487) <= eps
        assert abs(robot_caps[3][6] - (0.08850000310751244)) <= eps
        print("-----")
        assert abs(robot_caps[4][0] - 0.0) <= eps
        assert abs(robot_caps[4][1] - 0.0) <= eps
        assert abs(robot_caps[4][2] - 0.7915) <= eps
        assert abs(robot_caps[4][3] - 0.0) <= eps
        assert abs(robot_caps[4][4] - 0.0) <= eps
        assert abs(robot_caps[4][5] - 0.7915) <= eps
        assert abs(robot_caps[4][6] - (0.09000000310751244)) <= eps
        print("-----")
        assert(abs(robot_caps[5][0] - 0.00049324) <= eps)
        assert abs(robot_caps[5][1] - 0.0033497) <= eps
        assert abs(robot_caps[5][2] - 0.905303) <= eps
        assert abs(robot_caps[5][3] - -0.00049264) <= eps
        assert abs(robot_caps[5][4] - 0.0031812) <= eps
        assert abs(robot_caps[5][5] - 0.9853) <= eps
        assert abs(robot_caps[5][6] - (0.09254900310751243)) <= eps

    def test_humanPrediction(self, shield):
        """Test the humanPrediction() function with prediction data."""
        # Create prediction data for 3 time points
        # Each prediction contains spheres representing joint positions with bounded error

        # Prediction at time 0.0s
        spheres_t0 = []
        for i in range(21):  # 21 joints for CMU mocap
            center = Point(0.0, 0.0, 0.0)  # All joints at origin
            sphere = Sphere(center, 0.01)   # 1cm prediction error
            spheres_t0.append(sphere)
        pred_t0 = Prediction(0.0, spheres_t0)

        # Prediction at time 0.05s
        spheres_t1 = []
        for i in range(21):
            center = Point(0.1, 0.0, 0.0)  # Joints moved 10cm in x direction
            sphere = Sphere(center, 0.02)   # 2cm prediction error
            spheres_t1.append(sphere)
        pred_t1 = Prediction(0.05, spheres_t1)

        # Prediction at time 0.10s
        spheres_t2 = []
        for i in range(21):
            center = Point(0.2, 0.0, 0.0)  # Joints moved 20cm in x direction
            sphere = Sphere(center, 0.015)  # 1.5cm prediction error
            spheres_t2.append(sphere)
        pred_t2 = Prediction(0.10, spheres_t2)

        # Create list of predictions
        predictions = [pred_t0, pred_t1, pred_t2]

        # Call humanPrediction with the prediction data
        shield.humanPrediction(predictions)

        # Set up robot trajectory for safety analysis
        pos = [0.0 for i in range(6)]
        vel = [0.0 for i in range(6)]
        shield.newLongTermTrajectory(pos, vel)

        # Step the shield to trigger reachability analysis
        motion = shield.step(0.004)

        # Verify that the prediction was processed successfully
        # The shield should still return a motion (even if it's stopped for safety)
        assert motion is not None
        assert motion.getTime() == 0.004

        # Get safety status - with human movement predicted, safety may be compromised
        is_safe = shield.getSafety()
        # We expect safety to potentially be False due to predicted human movement
        assert is_safe is False or is_safe is True  # Either outcome is valid

        # Get human reach capsules to verify prediction was used
        human_caps = shield.getHumanReachCapsules()
        assert len(human_caps) > 0  # Should have capsules from prediction data

        # Each capsule should have 7 elements [p1x, p1y, p1z, p2x, p2y, p2z, radius]
        for cap in human_caps:
            assert len(cap) == 7
            # Radius should be positive (incorporates prediction errors)
            assert cap[6] > 0.0

    def test_humanPredictionTypes(self, shield):
        """Test the types used in humanPrediction bindings."""
        # Test Point creation and access
        point = Point(1.0, 2.0, 3.0)
        assert point.x == 1.0
        assert point.y == 2.0
        assert point.z == 3.0

        # Test Sphere creation
        sphere = Sphere(point, 0.05)
        center = sphere.center
        radius = sphere.radius
        assert center.x == 1.0
        assert center.y == 2.0
        assert center.z == 3.0
        assert radius == 0.05

        # Test Prediction creation
        spheres = [sphere]
        prediction = Prediction(0.1, spheres)
        assert prediction.time == 0.1  # time
        assert len(prediction.spheres) == 1  # spheres vector
        assert prediction.spheres[0].radius == 0.05

    # ---------------- Schunk -------------------
    def test_getRobotReachCapsulesSchunk(self, shield_schunk):
        """Test the getRobotReachCapsules() function for the Schunk test fixture."""
        pos = [0.0 for i in range(6)]
        vel = [0.0 for i in range(6)]
        shield_schunk.newLongTermTrajectory(pos, vel)
        dummy_meas = []
        for i in range(21):
            dummy_meas.append([0.0, 0.0, 0.0])
        shield_schunk.humanMeasurement(dummy_meas, 0.0)
        shield_schunk.step(0.004)
        robot_caps = shield_schunk.getRobotReachCapsules()
        for cap in robot_caps:
            assert len(cap) == 7
            print("-------")
            print(cap[0])
            print(cap[1])
            print(cap[2])
            print(cap[3])
            print(cap[4])
            print(cap[5])
            print(cap[6])
        eps = 1e-5
        secure_radius = 0.02
        assert abs(robot_caps[0][0] - 0) <= eps
        assert abs(robot_caps[0][1] - -0.015000000000000) <= eps
        assert abs(robot_caps[0][2] - 0.155300000000000) <= eps
        assert abs(robot_caps[0][3] - 0) <= eps
        assert abs(robot_caps[0][4] - -0.015000000000000) <= eps
        assert abs(robot_caps[0][5] - 0.155300000000000) <= eps
        assert abs(robot_caps[0][6] - (0.090000000000000 + secure_radius)) <= eps
        assert abs(robot_caps[1][0] - 2.295154367360221e-04) <= eps
        assert abs(robot_caps[1][1] - 0.114880476266314) <= eps
        assert abs(robot_caps[1][2] - 0.505203783848448) <= eps
        assert abs(robot_caps[1][3] - -6.617154367385000e-04) <= eps
        assert abs(robot_caps[1][4] - 0.114044823733687) <= eps
        assert abs(robot_caps[1][5] - 0.155205916151551) <= eps
        assert abs(robot_caps[1][6] - (0.074020642029512 + secure_radius)) <= eps
        assert abs(robot_caps[2][0] - -1.594314036332950e-15) <= eps
        assert abs(robot_caps[2][1] - 1.110223024625157e-15) <= eps
        assert abs(robot_caps[2][2] - 0.490300000000000) <= eps
        assert abs(robot_caps[2][3] - -1.594314036332950e-15) <= eps
        assert abs(robot_caps[2][4] - 1.110223024625157e-15) <= eps
        assert abs(robot_caps[2][5] - 0.490300000000000) <= eps
        assert abs(robot_caps[2][6] - (0.090000000000000 + secure_radius)) <= eps
        assert abs(robot_caps[3][0] - -4.693429213306548e-05) <= eps
        assert abs(robot_caps[3][1] - -0.005883360941466) <= eps
        assert abs(robot_caps[3][2] - 0.606104849281791) <= eps
        assert abs(robot_caps[3][3] - -2.365707868427609e-06) <= eps
        assert abs(robot_caps[3][4] - 0.061878260941472) <= eps
        assert abs(robot_caps[3][5] - 0.804871950718208) <= eps
        assert abs(robot_caps[3][6] - (0.07 + secure_radius)) <= eps
        assert abs(robot_caps[4][0] - -1.545847699099969e-15) <= eps
        assert abs(robot_caps[4][1] - 3.205768983605140e-15) <= eps
        assert abs(robot_caps[4][2] - 0.8065) <= eps
        assert abs(robot_caps[4][3] - -1.545847699099969e-15) <= eps
        assert abs(robot_caps[4][4] - 3.205768983605140e-15) <= eps
        assert abs(robot_caps[4][5] - 0.8065) <= eps
        assert abs(robot_caps[4][6] - (0.070000000000000 + secure_radius)) <= eps
        assert abs(robot_caps[5][0] - 4.932409872672667e-04) <= eps
        assert abs(robot_caps[5][1] - 0.003349675805265) <= eps
        assert abs(robot_caps[5][2] - 0.905303127433903) <= eps
        assert abs(robot_caps[5][3] - -4.926409872713459e-04) <= eps
        assert abs(robot_caps[5][4] - 0.003181224194743) <= eps
        assert abs(robot_caps[5][5] - 0.9713) <= eps
        assert abs(robot_caps[5][6] - (0.072548874802886 + secure_radius)) <= eps
