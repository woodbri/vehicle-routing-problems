
#include <iostream>

#include "vrp_assert.h"

int main() {

    try {

    assert(2+2 == 4);
    assert(2+2 == 5);

    }
/*
    catch (AssertFailedException *e) {
        std::cout << e.what() << "\n";
    }
*/
    catch (std::exception& e) {
        std::cout << e.what() << "\n";
    }
    catch(...) {
        std::cout << "Caught unknown exception!"\n";
    }

    return 0;

}
