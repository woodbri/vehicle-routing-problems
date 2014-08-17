//#include <stdexcept>
//#include <algorithm>
//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <string>
//#include <vector>
//#include <math.h>

#include "node.h"
#include "pathnode.h"


    void pathNode::setValues(const Node &depot) {
        setDistPrev(depot);
        totDistFromDepot=distPrev;
        twv=node->lateArrival(totDistFromDepot);
        if (node->earlyArrival(totDistFromDepot)) totDistFromDepot=node->opens();
        totDistFromDepot+=node->getServiceTime();
        cargo=node->getDemand();
        cv= cargo>cargoLimit or cargo < 0;
        twvTot = (twv)? 1:0;
        cvTot = (cv)? 1:0;
}

    void pathNode::setValues(pathNode &prev) {
        
        setDistPrev(prev.getnode());
        totDistFromDepot=prev.totDistFromDepot + distPrev;
        twv=node->lateArrival(totDistFromDepot);
        if (node->earlyArrival(totDistFromDepot)) totDistFromDepot=node->opens();
        totDistFromDepot+=node->getServiceTime();
        cargo=prev.cargo+node->getDemand();
        cv= cargo>cargoLimit or cargo < 0;
        twvTot = (twv)? prev.twvTot+prev.twvTot:prev.twvTot;
        cvTot = (cv)? prev.cvTot+1:prev.cvTot;
}

