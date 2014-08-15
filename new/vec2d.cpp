#include "stdio.h"
#include "math.h"

class Vec2
{
public:
    float _x;
    float _y;

    Vec2()
    {
        _x = 0;
        _y = 0;
    }

    Vec2( const float x, const float y )
    {
        _x = x;
        _y = y;
    }

    Vec2 operator+( const Vec2 &v ) const
    {
        return Vec2( this->_x + v._x, this->_y + v._y );
    }

    Vec2 operator-( const Vec2 &v ) const
    {
        return Vec2( this->_x - v._x, this->_y - v._y );
    }

    Vec2 operator*( const float f ) const
    {
        return Vec2( this->_x * f, this->_y * f );
    }

    float DistanceToSquared( const Vec2 p ) const
    {
        const float dX = p._x - this->_x;
        const float dY = p._y - this->_y;

        return dX * dX + dY * dY;
    }

    float DistanceTo( const Vec2 p ) const
    {
        return sqrt( this->DistanceToSquared( p ) );
    }

    float DotProduct( const Vec2 p ) const
    {
        return this->_x * p._x + this->_y * p._y;
    }
};

// return minimum distance between line segment vw and point p, and the closest point on the line segment, q
float DistanceFromLineSegmentToPoint( const Vec2 v, const Vec2 w, const Vec2 p, Vec2 * const q )
{
    const float distSq = v.DistanceToSquared( w ); // i.e. |w-v|^2 ... avoid a sqrt
    if ( distSq == 0.0 )
    {
        // v == w case
        (*q) = v;

        return v.DistanceTo( p );
    }

    // consider the line extending the segment, parameterized as v + t (w - v)
    // we find projection of point p onto the line
    // it falls where t = [(p-v) . (w-v)] / |w-v|^2

    const float t = ( p - v ).DotProduct( w - v ) / distSq;
    if ( t < 0.0 )
    {
        // beyond the v end of the segment
        (*q) = v;

        return v.DistanceTo( p );
    }
    else if ( t > 1.0 )
    {
        // beyond the w end of the segment
        (*q) = w;

        return w.DistanceTo( p );
    }

    // projection falls on the segment
    const Vec2 projection = v + ( ( w - v ) * t );

    (*q) = projection;

    return p.DistanceTo( projection );
}

float DistanceFromLineSegmentToPoint( float segmentX1, float segmentY1, float segmentX2, float segmentY2, float pX, float pY, float *qX, float *qY )
{
    Vec2 q;

    float distance = DistanceFromLineSegmentToPoint( Vec2( segmentX1, segmentY1 ), Vec2( segmentX2, segmentY2 ), Vec2( pX, pY ), &q );

    (*qX) = q._x;
    (*qY) = q._y;

    return distance;
}

void TestDistanceFromLineSegmentToPoint( float segmentX1, float segmentY1, float segmentX2, float segmentY2, float pX, float pY )
{
    float qX;
    float qY;
    float d = DistanceFromLineSegmentToPoint( segmentX1, segmentY1, segmentX2, segmentY2, pX, pY, &qX, &qY );
    printf( "line segment = ( ( %f, %f ), ( %f, %f ) ), p = ( %f, %f ), distance = %f, q = ( %f, %f )\n",
            segmentX1, segmentY1, segmentX2, segmentY2, pX, pY, d, qX, qY );
}

void TestDistanceFromLineSegmentToPoint()
{
    TestDistanceFromLineSegmentToPoint( 0, 0, 1, 1, 1, 0 );
    TestDistanceFromLineSegmentToPoint( 0, 0, 20, 10, 5, 4 );
    TestDistanceFromLineSegmentToPoint( 0, 0, 20, 10, 30, 15 );
    TestDistanceFromLineSegmentToPoint( 0, 0, 20, 10, -30, 15 );
    TestDistanceFromLineSegmentToPoint( 0, 0, 10, 0, 5, 1 );
    TestDistanceFromLineSegmentToPoint( 0, 0, 0, 10, 1, 5 );
}
