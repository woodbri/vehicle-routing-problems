#ifndef VEC2D_H
#define VEC2D_H

#include <cmath>

class Vec2d {
  private:
    double _x;
    double _y;

  public:
    double x() { return _x; };
    double y() { return _y; };

    Vec2d() {
        _x = 0;
        _y = 0;
    };

    Vec2d( const double x, const double y ) {
        _x = x;
        _y = y;
    };

    Vec2d operator+( const Vec2d &v ) const {
        return Vec2d( this->_x + v._x, this->_y + v._y );
    };

    Vec2d operator-( const Vec2d &v ) const {
        return Vec2d( this->_x - v._x, this->_y - v._y );
    };

    Vec2d operator*( const double f ) const {
        return Vec2d( this->_x * f, this->_y * f );
    };

    double distanceToSquared( const Vec2d p ) const {
        const double dX = p._x - this->_x;
        const double dY = p._y - this->_y;

        return dX * dX + dY * dY;
    };

    double distanceTo( const Vec2d p ) const {
        return sqrt( this->distanceToSquared( p ) );
    };

    double dotProduct( const Vec2d p ) const {
        return this->_x * p._x + this->_y * p._y;
    };

    double length( const Vec2d p ) const {
        return sqrt( _x * _x + _y * _y );
    };

    Vec2d unit( const Vec2d p ) const {
        double scale = 0.0;
        double len = length( p );

        if (len != 0.0)
            scale = 1.0 / len;
        return p * scale;
    };
};


// Non-class functions

double distanceFromLineSegmentToPoint( const Vec2d v, const Vec2d w, const Vec2d p, Vec2d * const q );

double distanceFromLineSegmentToPoint( double segmentX1, double segmentY1, double segmentX2, double segmentY2, double pX, double pY, double *qX, double *qY );


#endif

