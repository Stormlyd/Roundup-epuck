# Slot-assignment environment contract

This directory contains a Python 3.9, standard-library-only environment contract for RL slot assignment. It defines state validation, fixed 144-action decoding, observation encoding, triangular pursuit goals, and a transition callback boundary. An action is `heading_index * 6 + permutation_index`: 24 headings times 6 authoritative permutations gives 144 actions.

`SlotState.pursuers` must always be ordered by stable hardware ID so observations and permutation actions keep the same robot-to-index mapping across transitions.

It contains no learning model and grants no real-robot authority. A future Webots adapter and policy-inference integration remain separate work. Any future wheel-speed command must still pass through the final CBF safety layer; this module never authorizes physical-robot operation.

Run the contract tests from the repository root:

```sh
/usr/bin/python3 -m unittest -v rl.test_slot_assignment_env
```
