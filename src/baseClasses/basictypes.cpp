#include <basictypes.h>
#include <limits>

double VRP_MAX() { return ( std::numeric_limits<double>::max() ); };
double VRP_MIN() { return ( -std::numeric_limits<double>::max() ); };

