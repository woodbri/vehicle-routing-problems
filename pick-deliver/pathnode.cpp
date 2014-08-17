//#include <stdexcept>
//#include <algorithm>
//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <string>
//#include <vector>
//#include <math.h>

#include "order.h"
#include "twnode.h"
#include "pathnode.h"
//#include "Problem.h"


/*     

    void pathNode::setValues(const Node &depot) {
        setDistPrev(depot);
        totDistFromDepot=distPrev;
        //twv=node->lateArrival(totDistFromDepot);
        twv=lateArrival(totDistFromDepot);
        //if (node->earlyArrival(totDistFromDepot)) totDistFromDepot=node->opens();
        if (earlyArrival(totDistFromDepot)) totDistFromDepot=opens();
        //totDistFromDepot+=node->getServiceTime();
        totDistFromDepot+=getServiceTime();
        //cargo=node->getDemand();
        cargo=getDemand();
        cv= cargo>cargoLimit or cargo < 0;
        twvTot = (twv)? 1:0;
        cvTot = (cv)? 1:0;
}

    void pathNode::setValues(pathNode &prev) {
        
        //setDistPrev(prev.getnode());

        totDistFromDepot=prev.totDistFromDepot + distPrev;
        //twv=node->lateArrival(totDistFromDepot);
        twv=lateArrival(totDistFromDepot);
        //if (node->earlyArrival(totDistFromDepot)) totDistFromDepot=node->opens();
        if (earlyArrival(totDistFromDepot)) totDistFromDepot=opens();
        //totDistFromDepot+=node->getServiceTime();
        totDistFromDepot+=getServiceTime();
        //cargo=prev.cargo+node->getDemand();
        cargo=prev.cargo+getDemand();
        cv= cargo>cargoLimit or cargo < 0;
        twvTot = (twv)? prev.twvTot+prev.twvTot:prev.twvTot;
        cvTot = (cv)? prev.cvTot+1:prev.cvTot;
}
*/
