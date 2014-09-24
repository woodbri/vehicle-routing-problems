#ifndef DPNODE_H
#define DPNODE_H

#include <string>
#include "tweval.h"

class Dpnode: public Tweval {
private:
    //to know the other node in the order
    int oid;
    int did;
    int pid;
public:
    bool ispickup() const {return demand>0;}
    bool isdelivery() const {return demand<0;}
    bool isdepot() const {return  hasnogoods();}

    void dumpeval() const;
    void dump() const ;
/*accessors*/
    int getdid() const {return  did;};
    int getpid() const {return  pid;};
    int getoid() const {return oid;};
/* mutators */        
    void setoid(int _oid)  {oid=_oid;};
    bool operator== (const Dpnode& other) const{ return getnid()==other.getnid();};
    bool operator< (const Dpnode& other) const{ return getnid()< other.getnid();};

   Dpnode(std::string line);

   Dpnode():Tweval(){};
   ~Dpnode(){};

};    

#endif
