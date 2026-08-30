#ifndef ZOOIDTESTTARGETS_H
#define ZOOIDTESTTARGETS_H

#include <vector>

class ZooidTestTargets
{
public:
    void startSnapshot(const std::vector<unsigned int>& ids);
    void retainActive(const std::vector<unsigned int>& currentlyActiveIds);
    const std::vector<unsigned int>& activeIds() const;
    const std::vector<unsigned int>& lostIds() const;
    bool empty() const;
    void clear();

private:
    std::vector<unsigned int> activeIds_;
    std::vector<unsigned int> lostIds_;
};

#endif // ZOOIDTESTTARGETS_H
