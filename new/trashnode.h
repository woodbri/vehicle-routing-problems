#ifndef TRASHNODE_H
#define TRASHNODE_H

#include "twnode.h"

class Trashnode : public Twnode {
  private:
    int ntype;              // node type (0=depot, 1=dump, 2=pickup)
    double depotdist;     // distance to nearest depot
    int depotnid;         // nid of the closet depot
    double depotdist2;    // distance to nearest depot
    int depotnid2;        // nid of the closet depot
    double dumpdist;        // distance to nearest dump
    int dumpnid;            // nid of closet dump


  public:
    int getdepotdist() const {return depotdist;};
    int getdepotnid() const {return depotnid;};
    int getdumpdist() const {return dumpdist;};
    int getdumpnid() const {return dumpnid;};
    bool isdepot() const {return ntype==0;};
    bool isdump() const {return ntype==1;};
    bool ispickup() const {return ntype==2;};

    void dump() const;

    void setvalues(int nid, double x, double y, int demand,
                   int tw_open, int tw_close, int service,
                   int ntype);
    void setntype(int v) { ntype = v; };
    void setdepotdist(int nid, double dist, int nid2, double dist2);
    void setdumpdist(int nid, double dist);

    Trashnode() {
        Twnode();
        ntype = -1;
        depotdist = 0.0;
        depotnid = -1;
        depotdist2 = 0.0;
        depotnid2 = -1;
        dumpdist = 0.0;
        dumpnid = -1;
    };

    Trashnode(std::string line);

    ~Trashnode() {};

};

#endif
