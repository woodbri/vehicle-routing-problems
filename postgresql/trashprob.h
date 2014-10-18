#ifndef TRASHPROB_H
#define TRASHPROB_H

#include <string>
#include <sstream>

#include "vrptools.h"
#include "prob_trash.h"

class TrashProb : public Prob_trash {

  public:

    TrashProb() : Prob_trash() {};

    void addContainers( container_t *containers, int count );
    void addOtherlocs( otherloc_t *otherlocs, int count );
    bool checkNodesOk();
    void addTtimes( ttime_t *ttimes, int count );
    void addVehicles( vehicle_t *vehicles, int count );

    bool isValid() const;
    std::string whatIsWrong() const;

};

#endif
