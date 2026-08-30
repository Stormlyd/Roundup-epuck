#ifndef AGENT_H_
#define AGENT_H_
#pragma once

#include <cstddef>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "Simulator.h"
#include "Vector2.h"

/**
 * 代理器主要实现导航动态避让算法ORCA
 * 每个Zooid都有自己的ORCA
 * 每个Zooid同步独立地为它们自己选择一个新的速度Va_new并且这个新速度Va_new能够保证在预定的t时间内，持续的与别的机器人无碰撞移动。
 * 每个Zooid选择新速度Va_new时，都要尽可能的接近它们自己的期望速度Va_pre。
 * 每个Zooid之间不允许进行沟通，所以它们只能够观察得到别的Zooid的当前位置和当前速度。然而，每个机器人可以假设其它机器人也是使用和自己一样的策略来选择新的速度Va_new。
 * 这个问题无法使用中心协调的方式解决，因为每个机器人的期望速度只有它们自己知道。
 * by：Luzhimin
 * namespace：lzm
 */

namespace lzm {
    class Simulator;
    /**
     * @brief 模拟中的代理
     */
    class Agent {
    private:

        /**
         * @brief 期望速度集合
         */
        class Candidate {
        public:
            /**
             * @brief Constructor
             */
            Candidate() : velocityObstacle1_(0), velocityObstacle2_(0) { }

            /**
             * @brief 候选点的位置。
             */
            Vector2 position_;

            /**
             * @brief 第一速度障碍物的数
             */
            int velocityObstacle1_;

            /**
             * @brief 第二速度障碍物数
             */
            int velocityObstacle2_;
        };

        /**
         * @brief 障碍速度集合
         */
        class VelocityObstacle {
        public:
            /**
             * @brief Constructor
             */
            VelocityObstacle() { }

            /**
             * @brief 混合往复速度障碍物的顶点位置 顶点
             */
            Vector2 apex_;

            /**
             * @brief 混合往复速度障碍物第一侧的方向。边1
             */
            Vector2 side1_;

            /**
             * @brief 混合往复速度障碍物第二侧的方向。边2
             */
            Vector2 side2_;
        };

        /**
         * @brief Constructor
         * @param 模拟器。
         */
        explicit Agent(Simulator *simulator);

        /**
         * @brief Constructor
         * @param simulator 当前模拟器
         * @param position  代理期初始位置
         * @param goalNo    代理期目标编号
         */
        Agent(Simulator *simulator, const Vector2 &position, std::size_t goalNo);

        /**
         * @brief Agent
         * @param simulator
         * @param position
         * @param goalNo
         * @param neighborDist
         * @param maxNeighbors
         * @param radius
         * @param velocity
         * @param maxAccel
         * @param goalRadius
         * @param prefSpeed
         * @param maxSpeed
         * @param orientation
         * @param uncertaintyOffset
         */
        Agent(Simulator *simulator, const Vector2 &position, std::size_t goalNo, float neighborDist, std::size_t maxNeighbors, float radius, const Vector2 &velocity, float maxAccel, float goalRadius, float prefSpeed, float maxSpeed, float orientation,
#if DIFFERENTIAL_DRIVE
            float timeToOrientation, float wheelTrack,
#endif /* DIFFERENTIAL_DRIVE */
            float uncertaintyOffset);

        /**
         * @brief 计算当前代理的相邻代理。
         */
        void computeNeighbors();

        /**
         * @brief 计算当前代理的新速度。(期望速度)
         */
        void computeNewVelocity();

        /**
         * @brief 计算当前代理的首选速度。
         */
        void computePreferredVelocity();

#if DIFFERENTIAL_DRIVE
        /**
         * @brief 计算当前代理的车轮速度。
         */
        void computeWheelSpeeds();
#endif /* DIFFERENTIAL_DRIVE */

        /**
         * @brief 将一个相邻代理插入到此代理的相邻代理集合中。
         * @param agentNo   要插入的代理的编号
         * @param rangeSq   该代理周围的平方范围(距离)
         */
        void insertNeighbor(std::size_t agentNo, float &rangeSq);

        /**
         * @brief 更新此代理的方向，位置和速度。
         */
        void update();

        Simulator *const simulator_;

        Vector2 newVelocity_;       //新速度
        Vector2 position_;          //位置
        Vector2 prefVelocity_;      //期望速度
        Vector2 velocity_;          //速度-向量(大小、方向)
        std::size_t goalNo_;        //目标ID
        std::size_t maxNeighbors_;  //最大相邻数

        float goalRadius_;          //目标半径
        float maxAccel_;            //最大加速度
        float maxSpeed_;            //最大速度
        float neighborDist_;        //相邻间距
        float orientation_;         //方向
        float prefSpeed_;           //期望速度值
        float radius_;              //半径
        float uncertaintyOffset_;   //误差值
#if DIFFERENTIAL_DRIVE
        float leftWheelSpeed_;      //左轮速度
        float rightWheelSpeed_;     //右轮速度
        float timeToOrientation_;   //传动方向
        float wheelTrack_;          //轮距
#endif /* DIFFERENTIAL_DRIVE */
        bool reachedGoal_;

        std::multimap<float, Candidate> candidates_;        //候选障碍速度集合：A在此集合中选择此速度时，将在时间t内与B发生碰撞
        std::set<std::pair<float, std::size_t> > neighbors_;//相邻点集合
        std::vector<VelocityObstacle> velocityObstacles_;   //障碍速度集合

        friend class KdTree;
        friend class Simulator;
    };
}

#endif /* AGENT_H_ */



