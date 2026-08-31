import math
import unittest

from rl.slot_assignment_env import (
    ACTION_COUNT,
    FIELD_HEIGHT,
    FIELD_WIDTH,
    HEADING_COUNT,
    MARGIN,
    PHASE_RADII,
    PERMUTATION_COUNT,
    PERMUTATIONS,
    RobotState,
    RewardTerms,
    RewardWeights,
    SlotAssignmentEnv,
    SlotState,
    decode_action,
    encode_action,
    encode_observation,
    goals_for_action,
    score_reward,
    validate_state,
)


def state(phase="PURSUIT", target=(0.73, 0.457, 0.0), previous_action=-1):
    return SlotState(
        phase,
        RobotState(*target),
        0.01,
        -0.02,
        (
            RobotState(0.20, 0.30, 0.0),
            RobotState(0.40, 0.50, math.pi / 2),
            RobotState(0.60, 0.70, math.pi),
        ),
        previous_action,
    )


class SlotAssignmentEnvTests(unittest.TestCase):
    def test_contract_constants_and_all_actions(self):
        self.assertEqual((FIELD_WIDTH, FIELD_HEIGHT, MARGIN), (1.460, 0.914, 0.057))
        self.assertEqual((HEADING_COUNT, PERMUTATION_COUNT, ACTION_COUNT), (24, 6, 144))
        self.assertEqual(PERMUTATIONS, ((0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0)))
        for action in range(ACTION_COUNT):
            heading, permutation = decode_action(action)
            self.assertEqual((heading, permutation), (action // 6, PERMUTATIONS[action % 6]))
            self.assertEqual(encode_action(heading, permutation), action)
        self.assertEqual(decode_action(143), (23, (2, 1, 0)))

    def test_encode_rejects_boolean_out_of_range_and_non_authoritative_permutations(self):
        for heading, permutation in (
            (True, PERMUTATIONS[0]),
            (-1, PERMUTATIONS[0]),
            (HEADING_COUNT, PERMUTATIONS[0]),
            (0, [0, 1, 2]),
            (0, (0, 1, 3)),
            (0, (0, True, 2)),
        ):
            with self.subTest(heading=heading, permutation=permutation):
                with self.assertRaises((TypeError, ValueError)):
                    encode_action(heading, permutation)

    def test_decode_rejects_non_integer_and_out_of_range_actions(self):
        for action in (True, False, -1, 144, 1.0, "1"):
            with self.subTest(action=action):
                with self.assertRaises((TypeError, ValueError)):
                    decode_action(action)

    def test_validate_state_checks_finiteness_bounds_and_previous_action(self):
        self.assertTrue(validate_state(state()))
        for bad in (
            state(target=(math.nan, .4, 0.0)),
            state(target=(-.001, .4, 0.0)),
            state(previous_action=True),
            state(previous_action=144),
            SlotState("OTHER", state().target, 0, 0, state().pursuers),
        ):
            with self.subTest(bad=bad):
                self.assertFalse(validate_state(bad))

    def test_phase_radii_is_immutable(self):
        with self.assertRaises(TypeError):
            PHASE_RADII["PURSUIT"] = 1
        with self.assertRaises(TypeError):
            del PHASE_RADII["PURSUIT"]

    def test_exact_twenty_element_observation_fixture_and_previous_encoding(self):
        sample = SlotState(
            "SURROUND",
            RobotState(.73, .457, 0),
            .05,
            -.05,
            (RobotState(0, 0, 0), RobotState(.73, .457, 0), RobotState(1.46, .914, 0)),
            143,
        )
        self.assertEqual(
            encode_observation(sample),
            (0.0, 1.0, 0.0, .5, .5, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0, .5, .5, 0.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0),
        )
        no_previous = encode_observation(state(previous_action=-1))
        self.assertEqual(len(no_previous), 20)
        self.assertEqual(no_previous[-1], 0.0)

    def test_goals_follow_heading_radius_and_permutation_mapping(self):
        goals = goals_for_action(state(), 0)
        self.assertIsNotNone(goals)
        self.assertEqual(goals[0], (1.04, .457))
        self.assertAlmostEqual(goals[1][0], .575, places=12)
        self.assertAlmostEqual(goals[1][1], .457 + .31 * math.sqrt(3) / 2, places=12)
        self.assertAlmostEqual(goals[2][0], .575, places=12)
        self.assertAlmostEqual(goals[2][1], .457 - .31 * math.sqrt(3) / 2, places=12)
        swapped = goals_for_action(state(), 1)
        self.assertEqual(swapped[0], goals[0])
        self.assertEqual(swapped[1], goals[2])
        self.assertEqual(swapped[2], goals[1])
        surround = goals_for_action(state("SURROUND"), 6)
        self.assertAlmostEqual(surround[0][0], .73 + .24 * math.cos(2 * math.pi / 24))
        capture = goals_for_action(state("CAPTURE"), 0)
        self.assertAlmostEqual(capture[0][0], .73 + .1224)

    def test_near_edge_goals_are_infeasible(self):
        self.assertIsNone(goals_for_action(state(target=(MARGIN, .457, 0)), 12))

    def test_rewards_have_each_required_sign_and_switch_penalty(self):
        self.assertEqual(score_reward(RewardTerms(captured=True)), 100.0)
        self.assertEqual(score_reward(RewardTerms(capture_progress_delta=2)), 20.0)
        self.assertEqual(score_reward(RewardTerms(elapsed_seconds=2)), -2.0)
        self.assertEqual(score_reward(RewardTerms(path_length_metres=2)), -4.0)
        self.assertEqual(score_reward(RewardTerms(crossings=2)), -10.0)
        self.assertEqual(score_reward(RewardTerms(cbf_interventions=2)), -4.0)
        self.assertEqual(score_reward(RewardTerms(), switched=True), -1.0)
        self.assertEqual(score_reward(RewardTerms(invalid_transition=True), switched=True), -100.0)

    def test_reward_terms_reject_negative_costs_but_allow_negative_progress(self):
        for field in ("elapsed_seconds", "path_length_metres", "crossings", "cbf_interventions"):
            with self.subTest(field=field):
                with self.assertRaises(ValueError):
                    score_reward(RewardTerms(**{field: -1}))
        self.assertEqual(score_reward(RewardTerms(capture_progress_delta=-1)), -10.0)

    def test_score_reward_requires_boolean_switched_and_finite_result(self):
        for switched in (0, 1, "true"):
            with self.subTest(switched=switched):
                with self.assertRaises(TypeError):
                    score_reward(RewardTerms(), switched=switched)
        with self.assertRaises(ValueError):
            score_reward(RewardTerms(capture_progress_delta=1e308), RewardWeights(capture_progress_delta=1e308))

    def test_custom_invalid_transition_weight_is_used_by_score_and_every_fail_closed_path(self):
        weights = RewardWeights(invalid_transition=-321)
        self.assertEqual(score_reward(RewardTerms(invalid_transition=True), weights), -321.0)
        with self.assertRaises(ValueError):
            score_reward(RewardTerms(invalid_transition=True), RewardWeights(invalid_transition=math.inf))
        with self.assertRaisesRegex(ValueError, "invalid reward weights"):
            score_reward("invalid terms", RewardWeights(invalid_transition=math.inf))
        initial = state()
        invalid_next = state(target=(math.inf, .457, 0))
        responses = (
            lambda: "wrong response",
            lambda: (invalid_next, RewardTerms(), False),
            lambda: (state(), RewardTerms(elapsed_seconds=math.nan), False),
        )
        for response in responses:
            with self.subTest(response=response):
                env = SlotAssignmentEnv(lambda current, goals: response(), weights)
                observation = env.reset(initial)
                self.assertEqual(env.step(0), (observation, -321.0, True, {"invalid": True}))
        no_callback = SlotAssignmentEnv(lambda current, goals: self.fail("callback was invoked"), weights)
        observation = no_callback.reset(initial)
        self.assertEqual(no_callback.step(True), (observation, -321.0, True, {"invalid": True}))
        edge_env = SlotAssignmentEnv(lambda current, goals: self.fail("callback was invoked"), weights)
        edge_observation = edge_env.reset(state(target=(MARGIN, .457, 0)))
        self.assertEqual(edge_env.step(12), (edge_observation, -321.0, True, {"invalid": True}))

    def test_callback_invalid_transition_preserves_state_and_marks_invalid(self):
        initial = state()
        valid_next = state("SURROUND")
        calls = []

        def transition(current, goals):
            calls.append(current)
            if len(calls) == 1:
                return valid_next, RewardTerms(invalid_transition=True), False
            return valid_next, RewardTerms(), False

        env = SlotAssignmentEnv(transition, RewardWeights(invalid_transition=-321))
        initial_observation = env.reset(initial)
        self.assertEqual(env.step(0), (initial_observation, -321.0, True, {"invalid": True}))
        observation, reward, terminated, info = env.step(1)
        self.assertEqual(calls, [initial, initial])
        self.assertEqual(observation, encode_observation(state("SURROUND", previous_action=1)))
        self.assertEqual(reward, 0.0)
        self.assertFalse(terminated)
        self.assertEqual(info, {"invalid": False})

    def test_callback_unhashable_phase_fails_closed(self):
        initial = state()
        invalid_next = SlotState([], initial.target, 0, 0, initial.pursuers)
        env = SlotAssignmentEnv(
            lambda current, goals: (invalid_next, RewardTerms(), False),
            RewardWeights(invalid_transition=-321),
        )
        observation = env.reset(initial)
        try:
            result = env.step(0)
        except TypeError as error:
            self.fail("step must fail closed for an unhashable phase: {}".format(error))
        self.assertEqual(result, (observation, -321.0, True, {"invalid": True}))

    def test_negative_cost_terms_and_reward_overflow_fail_closed_without_state_pollution(self):
        initial = state()
        weights = RewardWeights(invalid_transition=-321)
        for field in ("elapsed_seconds", "path_length_metres", "crossings", "cbf_interventions"):
            with self.subTest(field=field):
                env = SlotAssignmentEnv(
                    lambda current, goals, field=field: (state("SURROUND"), RewardTerms(**{field: -1}), False),
                    weights,
                )
                observation = env.reset(initial)
                self.assertEqual(env.step(0), (observation, -321.0, True, {"invalid": True}))

        calls = []
        overflow_weights = RewardWeights(capture_progress_delta=1e308, invalid_transition=-321)

        def transition(current, goals):
            calls.append(current)
            if len(calls) == 1:
                return state("SURROUND"), RewardTerms(capture_progress_delta=1e308), False
            return state("SURROUND"), RewardTerms(), False

        env = SlotAssignmentEnv(transition, overflow_weights)
        observation = env.reset(initial)
        self.assertEqual(env.step(0), (observation, -321.0, True, {"invalid": True}))
        next_observation, reward, terminated, info = env.step(1)
        self.assertEqual(calls, [initial, initial])
        self.assertEqual(next_observation, encode_observation(state("SURROUND", previous_action=1)))
        self.assertEqual(reward, 0.0)
        self.assertFalse(terminated)
        self.assertEqual(info, {"invalid": False})

    def test_extreme_integer_reward_term_fails_closed(self):
        env = SlotAssignmentEnv(
            lambda current, goals: (state("SURROUND"), RewardTerms(crossings=10 ** 400), False),
            RewardWeights(invalid_transition=-321),
        )
        observation = env.reset(state())
        self.assertEqual(env.step(0), (observation, -321.0, True, {"invalid": True}))

    def test_score_reward_normalizes_integer_product_overflow(self):
        huge = 10 ** 200
        try:
            score_reward(
                RewardTerms(capture_progress_delta=huge),
                RewardWeights(capture_progress_delta=huge),
            )
        except Exception as error:
            self.assertIs(type(error), ValueError)
            self.assertRegex(str(error), "reward must be finite")
        else:
            self.fail("score_reward must reject an overflowing reward")

    def test_integer_reward_product_overflow_fails_closed_without_state_pollution(self):
        initial = state()
        huge = 10 ** 200
        calls = []

        def transition(current, goals):
            calls.append(current)
            terms = RewardTerms(capture_progress_delta=huge) if len(calls) == 1 else RewardTerms()
            return state("SURROUND"), terms, False

        env = SlotAssignmentEnv(
            transition,
            RewardWeights(capture_progress_delta=huge, invalid_transition=-321),
        )
        observation = env.reset(initial)
        try:
            result = env.step(0)
        except OverflowError as error:
            self.fail("step must fail closed for reward overflow: {}".format(error))
        self.assertEqual(result, (observation, -321.0, True, {"invalid": True}))
        next_observation, reward, terminated, info = env.step(1)
        self.assertEqual(calls, [initial, initial])
        self.assertEqual(next_observation, encode_observation(state("SURROUND", previous_action=1)))
        self.assertEqual(reward, 0.0)
        self.assertFalse(terminated)
        self.assertEqual(info, {"invalid": False})

    def test_step_requires_reset_and_invalid_action_never_calls_callback(self):
        calls = []
        env = SlotAssignmentEnv(lambda current, goals: calls.append((current, goals)))
        with self.assertRaises(RuntimeError):
            env.step(0)
        obs = env.reset(state())
        result = env.step(True)
        self.assertEqual(result, (obs, -100.0, True, {"invalid": True}))
        self.assertEqual(calls, [])

    def test_infeasible_action_never_calls_callback(self):
        calls = []
        env = SlotAssignmentEnv(lambda current, goals: calls.append(1))
        initial = state(target=(MARGIN, .457, 0))
        obs = env.reset(initial)
        self.assertEqual(env.step(12), (obs, -100.0, True, {"invalid": True}))
        self.assertEqual(calls, [])

    def test_successful_transition_updates_previous_action_and_scores(self):
        next_state = state("SURROUND")
        received = []

        def transition(current, goals):
            received.append((current, goals))
            return next_state, RewardTerms(capture_progress_delta=.5), False

        env = SlotAssignmentEnv(transition)
        env.reset(state())
        observation, reward, terminated, info = env.step(7)
        self.assertEqual(received[0][0], state())
        self.assertEqual(len(received[0][1]), 3)
        self.assertEqual(observation, encode_observation(state("SURROUND", previous_action=7)))
        self.assertEqual(reward, 5.0)
        self.assertFalse(terminated)
        self.assertEqual(info, {"invalid": False})

    def test_successful_action_change_applies_switch_penalty(self):
        env = SlotAssignmentEnv(lambda current, goals: (state(), RewardTerms(), False))
        env.reset(state(previous_action=0))
        observation, reward, terminated, info = env.step(1)
        self.assertEqual(observation[-1], 2 / 144)
        self.assertEqual(reward, -1.0)
        self.assertFalse(terminated)
        self.assertEqual(info, {"invalid": False})

    def test_invalid_or_nonfinite_transition_fails_closed(self):
        initial = state()
        invalid_next = state(target=(math.inf, .457, 0))
        cases = (
            lambda: (invalid_next, RewardTerms(), False),
            lambda: (state(), RewardTerms(elapsed_seconds=math.nan), False),
            lambda: (state(), "not terms", False),
            lambda: (state(), RewardTerms(), "not bool"),
        )
        for response in cases:
            with self.subTest(response=response):
                env = SlotAssignmentEnv(lambda current, goals: response())
                obs = env.reset(initial)
                self.assertEqual(env.step(0), (obs, -100.0, True, {"invalid": True}))

    def test_callback_exception_propagates(self):
        def transition(current, goals):
            raise LookupError("callback failure")

        env = SlotAssignmentEnv(transition)
        env.reset(state())
        with self.assertRaisesRegex(LookupError, "callback failure"):
            env.step(0)


if __name__ == "__main__":
    unittest.main()
