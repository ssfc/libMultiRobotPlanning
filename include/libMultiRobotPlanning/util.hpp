//
// Created by take_ on 2023/11/5.
//

#ifndef LIBMULTIROBOTPLANNING_UTIL_HPP
#define LIBMULTIROBOTPLANNING_UTIL_HPP

class Location  // 模板类Location实例化
{
public:
    int x;
    int y;

public:
    Location() = default;

    Location(int input_x, int input_y) :
    x(input_x),
    y(input_y)
    {}

    bool operator==(const Location& other) const
    {
        return std::tie(x, y) == std::tie(other.x, other.y);
    }

    bool operator<(const Location& other) const
    {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }

    friend std::ostream& operator<<(std::ostream& os, const Location& location)
    {
        return os << "(" << location.x << "," << location.y << ")";
    }
};

// 为了使用Location进行map, 吾人的程序里也有。
namespace std
{
    template <>
    struct hash<Location>
    {
        size_t operator()(const Location& location) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, location.x);
            boost::hash_combine(seed, location.y);

            return seed;
        }
    };
}

// Location Custom state for the search
class TimeLocation
{
public:
    int time;
    int x;
    int y;

public:
    TimeLocation(int time, int x, int y) :
            time(time), x(x), y(y)
    {}

    bool operator==(const TimeLocation& other) const
    {
        return time == other.time
               && x == other.x
               && y == other.y;
    }

    bool equal_except_time(const TimeLocation& other) const
    {
        return x == other.x && y == other.y;
    }

    friend std::ostream& operator<<(std::ostream& os, const TimeLocation& s)
    {
        return os << s.time << ": (" << s.x << "," << s.y << ")";
        // return os << "(" << s.x << "," << s.y << ")";
    }
};

namespace std
{
    template <>
    struct hash<TimeLocation>
    {
        size_t operator()(const TimeLocation& s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);

            return seed;
        }
    };
}

#endif //LIBMULTIROBOTPLANNING_UTIL_HPP
