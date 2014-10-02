#include "Order.h"

void Order::dump() {
    std::cout << oid << ", "
              << pid << ", "
              << did << ", "
              << dist <<  ", "
              << dist2 << std::endl;
}
