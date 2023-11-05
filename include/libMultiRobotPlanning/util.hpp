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
    Location(int x, int y) : x(x), y(y) {}

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


#endif //LIBMULTIROBOTPLANNING_UTIL_HPP
