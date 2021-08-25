#pragma once
#include <math.h>
namespace navigation{
    namespace types{
        class Vector2D{
            public:
                double x,y;
                Vector2D(){}
                Vector2D(double _x,double _y){
                    x = _x; 
                    y = _y; 
                }
                Vector2D add(const Vector2D& rhs){
                    return Vector2D(x+rhs.x, y+rhs.y);
                }
                Vector2D substract(const Vector2D& rhs){
                    return Vector2D(x-rhs.x, y-rhs.y);
                }
                Vector2D multiply( double rhs){
                    return Vector2D(x*rhs, y*rhs);
                }
                double getLength(){
                    return sqrt(x * x + y * y);
                }
                Vector2D getUnitVector(){
                    return Vector2D(x,y).multiply(1/getLength());
                }
        };
    }
}
