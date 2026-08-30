#ifndef HRVO_GOAL_H_
#define HRVO_GOAL_H_

#pragma once
#include "Vector2.h"

/**
 * 目标向量
 * 在计算时的中间辅助量
 * by：Luzhimin
 * namespace：lzm
 */

namespace lzm {
    class Simulator;

    class Goal {
    private:

        explicit Goal(const Vector2 &position);

        Vector2 position_;

        friend class Agent;
        friend class Simulator;
    };
}

#endif /* HRVO_GOAL_H_ */
