#ifndef TRASHNODE_H
#define TRASHNODE_H

#include "tweval.h"

class Trashnode : public Tweval {
  protected:

  public:
    // accessors
//    void dump() const;
    void dumpeval() const;

    // state
    bool isdepot() const {return type==0;};
    bool isStarting() const {return type==0;};
    bool isdump() const {return type==1;};
    bool ispickup() const {return type==2;};
    bool isEnding() const {return type==3;};
    bool isvalid() const;

    // mutators
//    void setvalues(int _nid, double _x, double _y, int _demand,
//                   int _tw_open, int _tw_close, int _service, int _ntype);
//    void setntype(int _ntype) { ntype = _ntype; };

//Constructors
    Trashnode(std::string line);
    ~Trashnode() {};
    Trashnode() : Tweval() { }; 


//  OLD IDEAS DOWN BELLOW ARE COMMENTED OUT
protected:
    double depotdist;       // distance to nearest depot
    long int depotnid;      // nid of the closet depot
    double depotdist2;      // distance to nearest depot
    long int depotnid2;     // nid of the closet depot
    double dumpdist;        // distance to nearest dump
    long int dumpnid;       // nid of closet dump
public:
//accessors
    double getdepotdist() const {return depotdist;};
    long int getdepotnid() const {return depotnid;};
    double getdepotdist2() const {return depotdist2;};
    long int getdepotnid2() const {return depotnid2;};
    double getdumpdist() const {return dumpdist;};
    long int getdumpnid() const {return dumpnid;};

//mutators
    void setdepotdist(int _nid, double _dist, int _nid2, double _dist2);
    void setdumpdist(int _nid, double _dist);
//END OF OLD IDEAS

};

#endif
