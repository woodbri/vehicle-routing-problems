
#include <limits>
#include "route.h"

Route::Route(Problem& p) : P(p) {
    D = 0;
    TWV = 0;
    CV = 0;
    path.clear();
};



