#ifndef TRASHNODE_H
#define TRASHNODE_H

#include "twnode.h"

class Trashnode : public Twnode {
  private:
    int ntype;              // node type (0=depot, 1=dump, 2=pickup)
    double vehicledist;     // distance to nearest depot
    int vehiclenid;         // nid of the closet depot
    double vehicledist2;    // distance to nearest depot
    int vehiclenid2;        // nid of the closet depot
    double dumpdist;        // distance to nearest dump
    int dumpnid;            // nid of closet dump


  public:
    int getvehicledist() const {return vehicledist;};
    int getvehiclenid() const {return vehiclenid;};
    int getdumpdist() const {return dumpdist;};
    int getdumpnid() const {return dumpnid;};
    bool isvehicle() const {return ntype==0;};
    bool isdump() const {return ntype==1;};
    bool ispickup() const {return ntype==2;};

    void dump() const;

    void setvalues(int nid, double x, double y, int demand,
                   int tw_open, int tw_close, int service,
                   int ntype);
    void setvehicledist(int nid, double dist, int nid2, double dist2);
    void setdumpdist(int nid, double dist);

    Trashnode() {
        Twnode();
        ntype = -1;
        vehicledist = 0.0;
        vehiclenid = -1;
        vehicledist2 = 0.0;
        vehiclenid2 = -1;
        dumpdist = 0.0;
        dumpnid = -1;
    };

    Trashnode(std::string line);

    ~Trashnode() {};

};

#endif
