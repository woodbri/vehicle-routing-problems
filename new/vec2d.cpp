#include "stdio.h"
#include "math.h"

#include "vec2d.h"

// return minimum distance between line segment vw and point p, and the closest point on the line segment, q
double distanceFromLineSegmentToPoint( const Vec2d v, const Vec2d w, const Vec2d p, Vec2d * const q ) {
    // i.e. |w-v|^2 ... avoid a sqrt
    const double distSq = v.distanceToSquared( w );

    if ( distSq == 0.0 ) {
        // v == w case
        (*q) = v;

        return v.distanceTo( p );
    }

    // consider the line extending the segment, parameterized as v + t (w - v)
    // we find projection of point p onto the line
    // it falls where t = [(p-v) . (w-v)] / |w-v|^2

    const double t = ( p - v ).dotProduct( w - v ) / distSq;
    if ( t < 0.0 ) {
        // beyond the v end of the segment
        (*q) = v;

        return v.distanceTo( p );
    }
    else if ( t > 1.0 ) {
        // beyond the w end of the segment
        (*q) = w;

        return w.distanceTo( p );
    }

    // projection falls on the segment
    const Vec2d projection = v + ( ( w - v ) * t );

    (*q) = projection;

    return p.distanceTo( projection );
}

double distanceFromLineSegmentToPoint( double segmentX1, double segmentY1, double segmentX2, double segmentY2, double pX, double pY, double *qX, double *qY ) {
    Vec2d q;

    double distance = distanceFromLineSegmentToPoint( Vec2d( segmentX1, segmentY1 ), Vec2d( segmentX2, segmentY2 ), Vec2d( pX, pY ), &q );

    (*qX) = q.x();
    (*qY) = q.y();

    return distance;
}

