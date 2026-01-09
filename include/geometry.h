#ifndef GEOMETRY_H_INCLUDED
#define GEOMETRY_H_INCLUDED

namespace Common {

struct Point2D {
    float x;
    float y;

    float euclidiandistance(const Point2D &other) const;
};

struct FrenetPoint {
    float d;
    float s;
};

struct FrenetState {
    float distance;
    float velocity;
    float accel;
};

} // namespace Common

#endif // GEOMETRY_H_INCLUDED
