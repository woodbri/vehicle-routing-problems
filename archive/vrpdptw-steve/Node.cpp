#include "Node.h"

void Node::dump() {
    std::cout << nid << ", "
              << x << ", "
              << y << ", "
              << demand << ", "
              << tw_open << ", "
              << tw_close << ", "
              << service << ", "
              << pid << ", "
              << did << std::endl;
}
