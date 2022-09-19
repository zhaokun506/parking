/*map的作用
    //设置障碍物
    绘制图内的图形
*/
#include "common/math/vec2d.h"
class MapPoint
{
public:
    double x;
    double y;
    double phi;
};

class OpenSpaceMap
{

public:
    OpenSpaceMap(/* args */);
    ~OpenSpaceMap();
    void SetXYBounds(double x_min, double x_max, double y_min, double y_max);
    void AddOnebstacle(std::vector<common::math::Vec2d> obstacles_vertices);
    const std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec() const;
    const std::vector<double> XYbounds() const;

private:
    /* data */
    // obstacles_vertices_vec_ in clock wise顺时针 order. Take different approach towards warm start and distance approach
    std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec_;
    //x_min,x_max,y_min,y_max
    std::vector<double> XYbounds_;

};

/*
const放在函数后主要是限制类中的成员函数，const放在函数前是限制函数返回类型为指针时通过指针对返回值的修改。

const放在函数后通常是用在类中，限制类的成员函数不能对成员变量进行修改。同时，被函数后的const修饰的成员函数也不能调用其他非const的成员函数。
*/