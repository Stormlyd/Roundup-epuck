
#include <stdexcept>

#include <QDebug>

#include "Simulator.h"
#include "Agent.h"
#include "Goal.h"
#include "KdTree.h"

namespace lzm {
    Simulator::Simulator() : defaults_(NULL), kdTree_(NULL), globalTime_(0.0f), timeStep_(0.0f), reachedGoals_(false)
    {
        kdTree_ = new KdTree(this);
    }

    Simulator::~Simulator()
    {
        delete defaults_;
        defaults_ = NULL;

        delete kdTree_;
        kdTree_ = NULL;

        for (std::vector<Agent *>::iterator iter = agents_.begin(); iter != agents_.end(); ++iter) {
            delete *iter;
            *iter = NULL;
        }

        for (std::vector<Goal *>::iterator iter = goals_.begin(); iter != goals_.end(); ++iter) {
            delete *iter;
            *iter = NULL;
        }
    }

    std::size_t Simulator::addAgent(const Vector2 &position, std::size_t goalNo)
    {
        if (defaults_ == NULL) {
            throw std::runtime_error("Agent defaults not set when adding agent.");
        }

        //创建一个代理
        Agent *agent = new Agent(this, position, goalNo);
        agents_.push_back(agent);

        return agents_.size() - 1;
    }

    std::size_t Simulator::addAgent(const Vector2 &position, std::size_t goalNo, float neighborDist, std::size_t maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed,
#if DIFFERENTIAL_DRIVE
        float timeToOrientation, float wheelTrack,
#endif /* DIFFERENTIAL_DRIVE */
        float uncertaintyOffset, float maxAccel, const Vector2 &velocity, float orientation)
    {
        Agent * agent = new Agent(this, position, goalNo, neighborDist, maxNeighbors, radius, velocity, maxAccel, goalRadius, prefSpeed, maxSpeed, orientation,
#if DIFFERENTIAL_DRIVE
            timeToOrientation, wheelTrack,
#endif /* DIFFERENTIAL_DRIVE */
            uncertaintyOffset);
        agents_.push_back(agent);

        return agents_.size() - 1;
    }

    std::size_t Simulator::addGoal(const Vector2 &position)
    {
        Goal *const goal = new Goal(position);
        goals_.push_back(goal);

        return goals_.size() - 1;
    }

    void Simulator::doStep()
    {
        if (kdTree_ == NULL) {
            throw std::runtime_error("Simulation not initialized when attempting to do step.");
        }

        if (timeStep_ == 0.0f) {
            throw std::runtime_error("Time step not set when attempting to do step.");
        }

        reachedGoals_ = true;

        try
        {
            kdTree_->build();
        }
        catch(int e){
            return ;
        }


        for (std::vector<Agent *>::iterator iter = agents_.begin(); iter != agents_.end(); ++iter) {
            (*iter)->computePreferredVelocity();
            (*iter)->computeNeighbors();
            (*iter)->computeNewVelocity();
#if DIFFERENTIAL_DRIVE
            (*iter)->computeWheelSpeeds();
#endif /* DIFFERENTIAL_DRIVE */
        }

        for (std::vector<Agent *>::iterator iter = agents_.begin(); iter != agents_.end(); ++iter) {
            (*iter)->update();
        }

        globalTime_ += timeStep_;
    }

    std::size_t Simulator::getAgentGoal(std::size_t agentNo) const
    {
        return agents_[agentNo]->goalNo_;
    }

    float Simulator::getAgentGoalRadius(std::size_t agentNo) const
    {
        return agents_[agentNo]->goalRadius_;
    }

#if DIFFERENTIAL_DRIVE
    float Simulator::getAgentLeftWheelSpeed(std::size_t agentNo) const
    {
        return agents_[agentNo]->leftWheelSpeed_;
    }
#endif /* DIFFERENTIAL_DRIVE */

    float Simulator::getAgentMaxAccel(std::size_t agentNo) const
    {
        return agents_[agentNo]->maxAccel_;
    }

    std::size_t Simulator::getAgentMaxNeighbors(std::size_t agentNo) const
    {
        return agents_[agentNo]->maxNeighbors_;
    }

    float Simulator::getAgentMaxSpeed(std::size_t agentNo) const
    {
        return agents_[agentNo]->maxSpeed_;
    }

    float Simulator::getAgentNeighborDist(std::size_t agentNo) const
    {
        return agents_[agentNo]->neighborDist_;
    }

    float Simulator::getAgentOrientation(std::size_t agentNo) const
    {
        return agents_[agentNo]->orientation_;
    }

    Vector2 Simulator::getAgentPosition(std::size_t agentNo) const
    {
        return agents_[agentNo]->position_;
    }

    float Simulator::getAgentPrefSpeed(std::size_t agentNo) const
    {
        return agents_[agentNo]->prefSpeed_;
    }

    float Simulator::getAgentRadius(std::size_t agentNo) const
    {
        return agents_[agentNo]->radius_;
    }

    bool Simulator::getAgentReachedGoal(std::size_t agentNo) const
    {
        return agents_[agentNo]->reachedGoal_;
    }

#if DIFFERENTIAL_DRIVE
    float Simulator::getAgentRightWheelSpeed(std::size_t agentNo) const
    {
        return agents_[agentNo]->rightWheelSpeed_;
    }

    float Simulator::getAgentTimeToOrientation(std::size_t agentNo) const
    {
        return agents_[agentNo]->timeToOrientation_;
    }
#endif /* DIFFERENTIAL_DRIVE */

    float Simulator::getAgentUncertaintyOffset(std::size_t agentNo) const
    {
        return agents_[agentNo]->uncertaintyOffset_;
    }

    Vector2 Simulator::getAgentVelocity(std::size_t agentNo) const
    {
        return agents_[agentNo]->velocity_;
    }

#if DIFFERENTIAL_DRIVE
    float Simulator::getAgentWheelTrack(std::size_t agentNo) const
    {
        return agents_[agentNo]->wheelTrack_;
    }
#endif /* DIFFERENTIAL_DRIVE */

    Vector2 Simulator::getGoalPosition(std::size_t goalNo) const
    {
        return goals_[goalNo]->position_;
    }

    void Simulator::setAgentDefaults(float neighborDist, std::size_t maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed,
#if DIFFERENTIAL_DRIVE
        float timeToOrientation, float wheelTrack,
#endif /* DIFFERENTIAL_DRIVE */
        float uncertaintyOffset, float maxAccel, const Vector2 &velocity, float orientation)
    {
        if (defaults_ == NULL) {
            defaults_ = new Agent(this);
        }

        defaults_->goalRadius_ = goalRadius;
        defaults_->maxAccel_ = maxAccel;
        defaults_->maxNeighbors_ = maxNeighbors;
        defaults_->maxSpeed_ = maxSpeed;
        defaults_->neighborDist_ = neighborDist;
        defaults_->newVelocity_ = velocity;
        defaults_->uncertaintyOffset_ = uncertaintyOffset;
        defaults_->orientation_ = orientation;
        defaults_->prefSpeed_ = prefSpeed;
        defaults_->radius_ = radius;
        defaults_->velocity_ = velocity;

#if DIFFERENTIAL_DRIVE
        defaults_->timeToOrientation_ = timeToOrientation;
        defaults_->wheelTrack_ = wheelTrack;

        defaults_->computeWheelSpeeds();
#endif /* DIFFERENTIAL_DRIVE */
    }

    void Simulator::setAgentGoal(std::size_t agentNo, std::size_t goalNo)
    {
        //第一个代理的目标编号为 1
        agents_[agentNo]->goalNo_ = goalNo;
    }

    void Simulator::setAgentGoalRadius(std::size_t agentNo, float goalRadius)
    {
        agents_[agentNo]->goalRadius_ = goalRadius;
    }

    void Simulator::setAgentMaxAccel(std::size_t agentNo, float maxAccel)
    {
        agents_[agentNo]->maxAccel_ = maxAccel;
    }

    void Simulator::setAgentMaxNeighbors(std::size_t agentNo, std::size_t maxNeighbors)
    {
        agents_[agentNo]->maxNeighbors_ = maxNeighbors;
    }

    void Simulator::setAgentMaxSpeed(std::size_t agentNo, float maxSpeed)
    {
        agents_[agentNo]->maxSpeed_ = maxSpeed;
    }

    void Simulator::setAgentNeighborDist(std::size_t agentNo, float neighborDist)
    {
        agents_[agentNo]->neighborDist_ = neighborDist;
    }

    void Simulator::setAgentOrientation(std::size_t agentNo, float orientation)
    {
        agents_[agentNo]->orientation_ = orientation;
    }

    void Simulator::setAgentPosition(std::size_t agentNo, const Vector2 &position)
    {
        agents_[agentNo]->position_ = position;
    }

    void Simulator::setAgentPrefSpeed(std::size_t agentNo, float prefSpeed)
    {
        agents_[agentNo]->prefSpeed_ = prefSpeed;
    }

    void Simulator::setAgentRadius(std::size_t agentNo, float radius)
    {
        agents_[agentNo]->radius_ = radius;
    }

    void Simulator::eraseGoal(std::size_t goalNo)
    {
        goals_.erase(goals_.begin() + goalNo);
    }

    void Simulator::eraseAgent(std::size_t agentNo)
    {
        if(agentNo < agents_.size())
        {
            //agents_.pop_back();
            agents_.erase(agents_.begin() + agentNo);
        }
        else
        {
            //agents_.pop_back();
             agents_.erase(agents_.begin() + agentNo);
            qDebug()<<"Message: agents range error:"<<agentNo;
        }
    }

    void Simulator::flushGoals()
    {
        for (std::vector<Goal *>::iterator iter = goals_.begin(); iter != goals_.end(); ++iter) {
            delete *iter;
            *iter = NULL;
        }
        goals_.clear();
    }

    void Simulator::flushAgents()
    {
        for (std::vector<Agent *>::iterator iter = agents_.begin(); iter != agents_.end(); ++iter) {
            delete *iter;
            *iter = NULL;
        }
        agents_.clear();
    }


    void Simulator::setGoalPosition(std::size_t goalNo, const Vector2 &position)
    {
        goals_[goalNo]->position_ = position;
    }

#if DIFFERENTIAL_DRIVE
    void Simulator::setAgentTimeToOrientation(std::size_t agentNo, float timeToOrientation)
    {
        agents_[agentNo]->timeToOrientation_ = timeToOrientation;
    }
#endif /* DIFFERENTIAL_DRIVE */

    void Simulator::setAgentUncertaintyOffset(std::size_t agentNo, float uncertaintyOffset)
    {
        agents_[agentNo]->uncertaintyOffset_ = uncertaintyOffset;
    }

    void Simulator::setAgentVelocity(std::size_t agentNo, const Vector2 &velocity)
    {
        agents_[agentNo]->velocity_ = velocity;
    }

#if DIFFERENTIAL_DRIVE
    void Simulator::setAgentWheelTrack(std::size_t agentNo, float wheelTrack)
    {
        agents_[agentNo]->wheelTrack_ = wheelTrack;
    }
#endif /* DIFFERENTIAL_DRIVE */
}
