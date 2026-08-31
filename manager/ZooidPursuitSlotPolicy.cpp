#include "ZooidPursuitSlotPolicy.h"

bool decodePursuitSlotAction(int action,
                             int& headingIndex,
                             std::array<int, 3>& permutation)
{
    if (action < 0 || action >= PursuitSlotActionCount) return false;

    static const std::array<std::array<int, 3>, PursuitSlotPermutationCount>
        permutations = {{
            {{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}},
            {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}
        }};
    headingIndex = action / PursuitSlotPermutationCount;
    permutation = permutations[action % PursuitSlotPermutationCount];
    return true;
}
