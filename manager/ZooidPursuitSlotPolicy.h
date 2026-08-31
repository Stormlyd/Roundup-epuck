#ifndef ZOOIDPURSUITSLOTPOLICY_H
#define ZOOIDPURSUITSLOTPOLICY_H

#include <array>

#include "ZooidPursuitTypes.h"

constexpr int PursuitSlotHeadingCount = 24;
constexpr int PursuitSlotPermutationCount = 6;
constexpr int PursuitSlotActionCount =
    PursuitSlotHeadingCount * PursuitSlotPermutationCount;

struct PursuitSlotObservation
{
    PursuitWorldState world;
    PursuitPhase phase = PursuitPhase::Idle;
    double radius = 0.0;
    int previousAction = -1;
};

class PursuitSlotPolicy
{
public:
    virtual ~PursuitSlotPolicy() = default;
    virtual bool chooseAction(const PursuitSlotObservation& observation,
                              int& selectedAction) = 0;
};

bool decodePursuitSlotAction(int action,
                             int& headingIndex,
                             std::array<int, 3>& permutation);

#endif // ZOOIDPURSUITSLOTPOLICY_H
