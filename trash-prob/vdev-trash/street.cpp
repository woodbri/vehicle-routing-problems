

#include <iostream>
#include <sstream>
#include <deque>

#include "twpath.h"
#include "street.h"



void Street::dump() const {
     std::cout<< "Street id: " << sid 
     << "\tRequired capacity: " << _reqCapacity
     << "\tRequired time: " << _reqTime;
    path.dump("Containers in street");
}



    void Street::dump(const std::string &title) const {
       path.dump(title);
    }

//
//  
//
//
bool Street::e_insert(Trashnode node) {
    
    assert (node.getnid()>=0);
    path.push_back( node );
    path.evaluate(MAX());
    evalLast(MAX());
    return true;
}


bool Street::e_push_front(Trashnode node) {
    assert (node.getnid()>=0);
    path.push_front( node  );
    
    return insert(node, 1);
}


bool Street::insert(Trashnode node, int at) {
    E_Ret ret = path.e_insert(node, at, getmaxcapacity());
    if (ret == OK) {
        path.evaluate(at, getmaxcapacity());
        evalLast();
    }
    else if (ret == INVALID) return false;
    return true;
}

bool Street::remove(int at) {
    E_Ret ret = path.e_remove(at, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Street::moverange( int rangefrom, int rangeto, int destbefore ) {
    E_Ret ret = path.e_move(rangefrom, rangeto, destbefore, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Street::movereverse( int rangefrom, int rangeto, int destbefore ) {
    E_Ret ret = path.e_movereverse(rangefrom, rangeto, destbefore, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Street::reverse( int rangefrom, int rangeto ) {
    E_Ret ret = path.e_reverse(rangefrom, rangeto, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Street::move( int fromi, int toj ) {
    E_Ret ret = path.e_move(fromi, toj, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Street::swap( const int& i, const int& j ) {
    E_Ret ret = path.e_swap(i, j, getmaxcapacity());
    if (ret == OK) evalLast();
    else if (ret == INVALID) return false;
    return true;
}


bool Street::swap(Street& v2, const int& i1, const int& i2) {
    E_Ret ret = path.e_swap(i1, getmaxcapacity(),
                    v2.getvpath(), i2, v2.getmaxcapacity());
    if (ret == OK) {
        evalLast();
        v2.evalLast();
    }
    else if (ret == INVALID) return false;
    return true;
}


void Street::restorePath(Twpath<Trashnode> oldpath) {
    path = oldpath;
    evalLast();
}
*/

void Street::evalLast() {
    Trashnode last = path[path.size()-1];
    _reqCapacity=last.getcargo();
    _reqTime=last.getTotTime();
}

/**************************************PLOT************************************/
void Street::plot(std::string file,std::string title){
    Twpath<Trashnode> trace=path;
    std::stringstream sidStr;
    sidStr << sid;
    std::string extra=file+" street "+sidStr.str() ;

    Plot<Trashnode> graph( trace );
    graph.setFile( file+".png" );
    graph.setTitle( title+extra );
    graph.drawInit();
    for (int i=0; i<trace.size(); i++){
        if (trace[i].ispickup())  {
             graph.drawPoint(trace[i], 0x0000ff, 9, true);
        } else if (trace[i].isdepot()) {
             graph.drawPoint(trace[i], 0x00ff00, 5, true);
        } else  {
             graph.drawPoint(trace[i], 0xff0000, 7, true);
        }
    }
    plot(graph);
    graph.save();
}

void Street::plot(Plot<Trashnode> graph){
    graph.drawPath(path,graph.makeColor(sid*10), 1, true);
}


