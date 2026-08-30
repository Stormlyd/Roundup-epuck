#include "ZooidPursuitRoles.h"

#include <algorithm>

std::vector<unsigned int> PursuitRoleMap::participantIds() const
{
    if (!valid)
        return {};
    return {targetId, pursuerIds[0], pursuerIds[1], pursuerIds[2]};
}

bool assignPursuitRoles(const std::vector<unsigned int>& freshIds,
                        PursuitRoleMap& out)
{
    std::vector<unsigned int> ids = freshIds;
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    if (ids.size() < 4)
        return false;

    PursuitRoleMap assigned;
    assigned.targetId = ids[0];
    assigned.pursuerIds = {{ids[1], ids[2], ids[3]}};
    assigned.valid = true;
    out = assigned;
    return true;
}
