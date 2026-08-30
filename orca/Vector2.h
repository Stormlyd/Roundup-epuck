#ifndef VECTOR2_H_
#define VECTOR2_H_

#pragma once

#include <cmath>
#include <iosfwd>

/**
 * 二维向量
 * 在计算时的中间辅助量
 * by：Luzhimin
 * namespace：lzm
 */

namespace lzm {

    class Vector2 {
    public:
        //二维向量
        Vector2() : x_(0.0f), y_(0.0f) { }
        //二维向量
        Vector2(float x, float y) : x_(x), y_(y) { }
        //获取点X
        float getX() const { return x_; }
        //获取点Y
        float getY() const { return y_; }
        //设置点X
        void setX(float x) { x_ = x; }
        //设置点Y
        void setY(float y) { y_ = y; }
        //相反向量
        Vector2 operator-() const
        {
            return Vector2(-x_, -y_);
        }
        //x^2 + y^2
        float operator*(const Vector2 &other) const
        {
            return x_ * other.x_ + y_ * other.y_;
        }
        //数乘
        Vector2 operator*(float scalar) const
        {
            return Vector2(x_ * scalar, y_ * scalar);
        }
        //数除
        Vector2 operator/(float scalar) const
        {
            const float invScalar = 1.0f / scalar;

            return Vector2(x_ * invScalar, y_ * invScalar);
        }
        //相加
        Vector2 operator+(const Vector2 &other) const
        {
            return Vector2(x_ + other.x_, y_ + other.y_);
        }
        //差
        Vector2 operator-(const Vector2 &other) const
        {
            return Vector2(x_ - other.x_, y_ - other.y_);
        }
        //相等
        bool operator==(const Vector2 &other) const
        {
            return x_ == other.x_ && y_ == other.y_;
        }
        //不等
        bool operator!=(const Vector2 &other) const
        {
            return !(*this == other);
        }
        //数乘
        Vector2 &operator*=(float scalar)
        {
            x_ *= scalar;
            y_ *= scalar;

            return *this;
        }
        //数除
        Vector2 &operator/=(float scalar)
        {
            const float invScalar = 1.0f / scalar;

            x_ *= invScalar;
            y_ *= invScalar;

            return *this;
        }
        //
        Vector2 &operator+=(const Vector2 &other)
        {
            x_ += other.x_;
            y_ += other.y_;

            return *this;
        }
        //
        Vector2 &operator-=(const Vector2 &other)
        {
            x_ -= other.x_;
            y_ -= other.y_;

            return *this;
        }

    private:
        float x_;
        float y_;
    };

    //模
    inline float abs(const Vector2 &vector)
    {
        return std::sqrt(vector * vector);
    }
    //模的平方
    inline float absSq(const Vector2 &vector)
    {
        return vector * vector;
    }
    //反正切
    inline float atan(const Vector2 &vector)
    {
        return std::atan2(vector.getY(), vector.getX());
    }
    //
    inline float det(const Vector2 &vector1, const Vector2 &vector2)
    {
        return vector1.getX() * vector2.getY() - vector1.getY() * vector2.getX();
    }

    inline Vector2 normalize(const Vector2 &vector)
    {
        return vector / abs(vector);
    }


    inline Vector2 normal(const Vector2 &vector1, const Vector2 &vector2)
    {
        return normalize(Vector2(vector2.getY() - vector1.getY(), vector1.getX() - vector2.getX()));
    }

    inline Vector2 operator*(float scalar, const Vector2 &vector)
    {
        return Vector2(scalar * vector.getX(), scalar * vector.getY());
    }

    std::ostream &operator<<(std::ostream &stream, const Vector2 &vector);
}

#endif /* VECTOR2_H_ */
