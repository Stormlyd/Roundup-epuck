#ifndef SIMULATOR_H_
#define SIMULATOR_H_
#pragma once

#include <limits>
#include <vector>
#include "Definitions.h"
#include "Vector2.h"

#define DIFFERENTIAL_DRIVE 1 //差速驱动器个数

/**
 * 模拟器算法托管代理器
 * by：Luzhimin
 * namespace：lzm
 */

namespace lzm {
    class Agent;
    class Goal;
    class KdTree;

    class Simulator {
    public:
        /**
         * @brief Constructor
         */
        Simulator();

        ~Simulator();

        /**
         * @brief 将默认属性的新代理添加到模拟中
         * @param position  代理的起始位置
         * @param goalNo    代理的目标编号
         * @return          代理编号
         */
        std::size_t addAgent(const Vector2 &position, std::size_t goalNo);

        /**
         * @brief 添加一个新代理到模拟器中
         * @param position          这个代理的起始位置(二维)
         * @param goalNo            代理的目标编号
         * @param neighborDist      代理的最大邻居距离（中心点到中心点）。数值越大，模拟的运行时间就越长。如果数值太低，模拟将不安全。必须是非负的
         * @param maxNeighbors      代理的最大邻居计数。数值越大，模拟的运行时间就越长。如果数值太低，模拟将不安全
         * @param radius            代理的半径。必须是非负的。
         * @param goalRadius        代理目标的半径。必须是非负的。
         * @param prefSpeed         期望速度，优先按照这个期望速度去运行。
         * @param maxSpeed          代理的最大速度。
         * @param uncertaintyOffset 该代理的不确定性补偿
         * @param maxAccel          代理运行的最大加速度
         * @param velocity          代理的起始初速度
         * @param orientation       代理的起始方向(弧度)
         * @return                  代理编号
         */
        std::size_t addAgent(const Vector2 &position, std::size_t goalNo,float neighborDist, std::size_t maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed,
#if DIFFERENTIAL_DRIVE
            float timeToOrientation, float wheelTrack,
#endif /* DIFFERENTIAL_DRIVE */
            float uncertaintyOffset = 0.0f, float maxAccel = std::numeric_limits<float>::infinity(), const Vector2 &velocity = Vector2(0.0f, 0.0f), float orientation = 0.0f);

        /**
         * @brief 添加一个新的目标到到代理器中
         * @param position  这个目标的起始位置.
         * @return          目标编号
         */
        std::size_t addGoal(const Vector2 &position);

        /**
         * @brief 执行模拟步骤； 更新每个代理的方向，位置和速度，以及每个代理向其目标的进度。
         */
        void doStep();

        /**
         * @brief 返回指定代理的目标编号
         * @param   代理编号
         * @return  当前代理的目标编号
         */
        std::size_t getAgentGoal(std::size_t agentNo) const;

        /**
         * @brief 返回指定代理的目标半径
         * @param agentNo   要检索其目标代理编号
         * @return          代理当前的目标编号
         */
        float getAgentGoalRadius(std::size_t agentNo) const;

#if DIFFERENTIAL_DRIVE
        /**
         * @brief 返回代理当前左轮的速度
         * @param agentNo   要检索其左轮速度的代理的编号
         * @return          代理当前的左轮速度
         */
        float getAgentLeftWheelSpeed(std::size_t agentNo) const;
#endif /* DIFFERENTIAL_DRIVE */

        /**
         * @brief 返回代理当前的最大加速度
         * @param agentNo   要检索其加速度的代理的编号
         * @return          代理当前的最大加速度
         */
        float getAgentMaxAccel(std::size_t agentNo) const;

        /**
         * @brief 返回指定代理的最大邻居数
         * @param agentNo   要检索其最大邻居数的代理的编号。
         * @return          当前代理的最大邻居数。
         */
        std::size_t getAgentMaxNeighbors(std::size_t agentNo) const;

        /**
         * @brief 返回代理当前的最大速度
         * @param agentNo   要检索其速度的代理的编号
         * @return          代理当前的最大速度
         */
        float getAgentMaxSpeed(std::size_t agentNo) const;

        /**
         * @brief 返回指定代理的最大邻居距离。
         * @param agentNo   要检索其最大邻居距离的代理的编号。
         * @return          代理当前的最大邻居距离
         */
        float getAgentNeighborDist(std::size_t agentNo) const;

        /**
         * @brief 返回指定代理的方向角(弧度)。
         * @param agentNo   要检索其方向角代理的编号。
         * @return          方向角
         */
        float getAgentOrientation(std::size_t agentNo) const;


        /**
         * @brief 返回指定代理的位置。
         * @param agentNo   要检索其位置代理的编号。
         * @return          代理（中心）的当前位置。
         */
        Vector2 getAgentPosition(std::size_t agentNo) const;

        /**
         * @brief 返回当前代理的期望速度
         * @param agentNo   要检索代理的编号
         * @return          当前代理的期望速度
         */
        float getAgentPrefSpeed(std::size_t agentNo) const;

        /**
         * @brief 返回当前代理的半径
         * @param agentNo   要检索的代理编号
         * @return          当前代理的半径
         */
        float getAgentRadius(std::size_t agentNo) const;

        /**
         * @brief 返回代理到目标的状态
         * @param agentNo   要检索的代理编号
         * @return          如果代理已达到目标，则为true；否则为false
         */
        bool getAgentReachedGoal(std::size_t agentNo) const;

#if DIFFERENTIAL_DRIVE

        /**
         * @brief 返回指定代理的右轮速度
         * @param agentNo   要检索的代理编号
         * @return          当前代理的右轮速度
         */
        float getAgentRightWheelSpeed(std::size_t agentNo) const;

        /**
         * @brief 返回指定代理的“定向时间”。时间τ
         * @param agentNo   要检索的代理编号
         * @return          代理定位的当前时间
         */
        float getAgentTimeToOrientation(std::size_t agentNo) const;
#endif /* DIFFERENTIAL_DRIVE */

        /**
         * @brief 返回指定代理的“不确定性偏移量”。误差
         * @param agentNo   要检索的代理编号
         * @return          当前代理的不确定性抵消
         */
        float getAgentUncertaintyOffset(std::size_t agentNo) const;

        /**
         * @brief 返回指定代理的速度
         * @param agentNo   要检索的代理编号
         * @return          代理当前的速度
         */
        Vector2 getAgentVelocity(std::size_t agentNo) const;

#if DIFFERENTIAL_DRIVE
        /**
         * @brief 返回指定代理的车轮轨迹 轮间距
         * @param agentNo   要检索的代理编号
         * @return          当前代理的轮间距
         */
        float getAgentWheelTrack(std::size_t agentNo) const;
#endif /* DIFFERENTIAL_DRIVE */

        /**
         * @brief 返回模拟的全局时间
         * @return  当前的模拟全局时间（初始为零）
         */
        float getGlobalTime() const { return globalTime_; }

        /**
         * @brief 返回指定目标的位置
         * @param goalNo    要获取其位置的目标的编号
         * @return          目标的位置
         */
        Vector2 getGoalPosition(std::size_t goalNo) const;

        /**
         * @brief 返回模拟中的代理器个数
         * @return  模拟器中的代理器个数
         */
        std::size_t getNumAgents() const { return agents_.size(); }

        /**
         * @brief 返回模拟中的目标个数
         * @return  模拟器中的目标个数
         */
        std::size_t getNumGoals() const { return goals_.size(); }

        /**
         * @brief 返回模拟的时间步长。
         * @return 模拟的当前步骤
         */
        float getTimeStep() const { return timeStep_; }

        /**
         * @brief 返回所有代理到达目标位置
         * @return  如果所有代理都达到了目标，则为true； 否则为false。
         */
        bool haveReachedGoals() const { return reachedGoals_; }

        /**
         * @brief 为添加的任何新代理设置默认属性
         * @param neighborDist      新代理的默认最大邻居距离。
         * @param maxNeighbors      新代理的默认最大邻居数。
         * @param radius            新代理的默认半径。
         * @param goalRadius        新代理的默认目标半径。
         * @param prefSpeed         新代理的默认首选速度。
         * @param maxSpeed          新代理的默认最大速度。
         * @param uncertaintyOffset 新代理的默认不确定性偏移量。
         * @param maxAccel          新代理的默认最大加速度。
         * @param velocity          新代理的默认初始速度。
         * @param orientation       新代理的默认初始方向（以弧度为单位）。
         */
        void setAgentDefaults(float neighborDist, std::size_t maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed,
#if DIFFERENTIAL_DRIVE
            float timeToOrientation, float wheelTrack,
#endif /* DIFFERENTIAL_DRIVE */
            float uncertaintyOffset = 0.0f, float maxAccel = std::numeric_limits<float>::infinity(), const Vector2 &velocity = Vector2(), float orientation = 0.0f);

        /**
         * @brief 设置指定代理的目标编号
         * @param agentNo   要修改目标编号的代理编号
         * @param goalNo    要设置的新目标编号
         */
        void setAgentGoal(std::size_t agentNo, std::size_t goalNo);

        /**
         * @brief 设置指定代理的目标半径。
         * @param agentNo       要修改其目标半径的代理编号
         * @param goalRadius    要设置的新目标半径
         */
        void setAgentGoalRadius(std::size_t agentNo, float goalRadius);

        /**
         * @brief 设置指定代理的最大加速度
         * @param agentNo       要修改其最大加速度的代理编号
         * @param goalRadius    要设置的新最大加速度
         */
        void setAgentMaxAccel(std::size_t agentNo, float maxAccel);

        /**
         * @brief 设置指定代理的最大邻居数
         * @param agentNo       要修改其最大邻居数的代理编号
         * @param goalRadius    要设置的新最大邻居数
         */
        void setAgentMaxNeighbors(std::size_t agentNo, std::size_t maxNeighbors);

        /**
         * @brief 设置指定代理的最大速度
         * @param agentNo       要修改其最大速度的代理编号
         * @param goalRadius    要设置的新最大速度
         */
        void setAgentMaxSpeed(std::size_t agentNo, float maxSpeed);

        /**
         * @brief 设置指定代理的最大邻居距离
         * @param agentNo       要修改其最大邻居距离的代理编号
         * @param goalRadius    要设置的新最大邻居距离
         */
        void setAgentNeighborDist(std::size_t agentNo, float neighborDist);

        /**
         * @brief 设置指定代理的方向。
         * @param agentNo       要更改其方向的代理编号。
         * @param orientation   要设置新的方向
         */
        void setAgentOrientation(std::size_t agentNo, float orientation);

        /**
         * @brief 设置指定代理的位置
         * @param agentNo       要修改其位置的代理编号
         * @param goalRadius    要设置的位置
         */
        void setAgentPosition(std::size_t agentNo, const Vector2 &position);

        /**
         * @brief 设置指定代理的期望速度
         * @param agentNo       要修改其期望速度的代理编号
         * @param goalRadius    要设置的期望速度
         */
        void setAgentPrefSpeed(std::size_t agentNo, float prefSpeed);

        /**
         * @brief 设置指定代理的半径
         * @param agentNo       要修改其半径的代理编号
         * @param goalRadius    要设置的半径
         */
        void setAgentRadius(std::size_t agentNo, float radius);

#if DIFFERENTIAL_DRIVE
        void setAgentTimeToOrientation(std::size_t agentNo, float timeToOrientation);
        void setAgentWheelTrack(std::size_t agentNo, float wheelTrack);
#endif /* DIFFERENTIAL_DRIVE */

        /**
         * @brief 设置指定代理的“不确定性偏移量”。
         * @param agentNo               要修改不确定性偏移量的代理编号。
         * @param uncertaintyOffset     设置新的偏移量(误差)
         */
        void setAgentUncertaintyOffset(std::size_t agentNo, float uncertaintyOffset);

        /**
         * @brief 设置指定代理的速度
         * @param agentNo   要修改其速度的代理编号
         * @param velocity  要设置的新速度
         */
        void setAgentVelocity(std::size_t agentNo, const Vector2 &velocity);

        //设置步进时间
        void setTimeStep(float timeStep) { timeStep_ = timeStep; }

        /**
         * @brief 设置指定代理目标的位置
         * @param agentNo       要修改其位置的代理目标编号
         * @param goalRadius    要设置的目标位置
         */
        void setGoalPosition(std::size_t goalNo, const Vector2 &position);

        /**
         * @brief 清空所有目标
         */
        void flushGoals();

        /**
         * @brief 清空所有代理
         */
        void flushAgents();

        /**
         * @brief 根据目标编号删除指定的目标
         * @param goalNo    要删除的目标编号
         */
        void eraseGoal(std::size_t goalNo);

        /**
         * @brief 根据代理编号删除指定的代理
         * @param agentNo   要删除的代理编号
         */
        void eraseAgent(std::size_t agentNo);

    private:
        Simulator(const Simulator &other);
        Simulator &operator=(const Simulator &other);

        Agent *defaults_;
        KdTree *kdTree_;
        float globalTime_;
        float timeStep_;
        bool reachedGoals_;
        std::vector<Agent *> agents_;
        std::vector<Goal *> goals_;

        friend class Agent;
        friend class Goal;
        friend class KdTree;
    };
}

#endif /* SIMULATOR_H_ */
