#include "ZooidTestTargets.h"

#include <algorithm>
#include <iterator>

namespace
{
std::vector<unsigned int> normalized(std::vector<unsigned int> ids)
{
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
}
}

void ZooidTestTargets::startSnapshot(const std::vector<unsigned int>& ids)
{
    activeIds_ = normalized(ids);
    lostIds_.clear();
}

void ZooidTestTargets::recordCommanded(const std::vector<unsigned int>& ids)
{
    activeIds_.insert(activeIds_.end(), ids.begin(), ids.end());
    activeIds_ = normalized(activeIds_);
}

void ZooidTestTargets::retainActive(const std::vector<unsigned int>& currentlyActiveIds)
{
    const std::vector<unsigned int> current = normalized(currentlyActiveIds);
    std::vector<unsigned int> retained;
    std::vector<unsigned int> newlyLost;

    std::set_intersection(activeIds_.begin(), activeIds_.end(),
                          current.begin(), current.end(),
                          std::back_inserter(retained));
    std::set_difference(activeIds_.begin(), activeIds_.end(),
                        current.begin(), current.end(),
                        std::back_inserter(newlyLost));

    activeIds_.swap(retained);
    lostIds_.insert(lostIds_.end(), newlyLost.begin(), newlyLost.end());
    lostIds_ = normalized(lostIds_);
}

const std::vector<unsigned int>& ZooidTestTargets::activeIds() const
{
    return activeIds_;
}

const std::vector<unsigned int>& ZooidTestTargets::lostIds() const
{
    return lostIds_;
}

bool ZooidTestTargets::empty() const
{
    return activeIds_.empty();
}

void ZooidTestTargets::clear()
{
    activeIds_.clear();
    lostIds_.clear();
}
