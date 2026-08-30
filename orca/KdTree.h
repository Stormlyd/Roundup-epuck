#ifndef HRVO_KD_TREE_H_
#define HRVO_KD_TREE_H_

#pragma once

#include <cstddef>
#include <vector>
#include "Vector2.h"

/**
 * KdTree
 * 在计算时的中间辅助量
 * 主要用于节点查找
 * by：Luzhimin
 * namespace：lzm
 */

namespace lzm {
    class Agent;
    class Simulator;

    class KdTree {
    private:

        class Node {
        public:

            Node() : begin_(0), end_(0), left_(0), right_(0), maxX_(0.0f), maxY_(0.0f), minX_(0.0f), minY_(0.0f) { }

            std::size_t begin_;

            std::size_t end_;

            std::size_t left_;

            std::size_t right_;

            float maxX_;

            float maxY_;

            float minX_;

            float minY_;
        };

        static const std::size_t HRVO_MAX_LEAF_SIZE = 10;

        explicit KdTree(Simulator *simulator);

        void build();

        void buildRecursive(std::size_t begin, std::size_t end, std::size_t node);

        void query(Agent *agent, float rangeSq) const
        {
            queryRecursive(agent, rangeSq, 0);
        }

        void queryRecursive(Agent *agent, float &rangeSq, std::size_t node) const;

        Simulator *const simulator_;
        std::vector<std::size_t> agents_;
        std::vector<Node> nodes_;

        friend class Agent;
        friend class Simulator;
    };
}

#endif /* HRVO_KD_TREE_H_ */
