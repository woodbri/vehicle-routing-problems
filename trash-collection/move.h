#ifndef MOVE_H
#define MOVE_H

// This class defines a move object that can be placed on the Tabu list
// and/or can be applied to a given solution to transform it to a new state
// setting attributes to -1 means they are undefined and should be ignored.
//
// We are contemplating three different moves Ins, InterSw, IntraSw
// Ins (insert)
//  - remove a nid from rid1 at pos1 and insert it into rid2 as pos2
// InterSw (inter route swap)
//  - exchange a node with another node in another route
//    swap nid1 at pos1 in rid1 with nid2 at pos2 in rid2
// IntraSw (intra route swap)
//  - exchange nid1 and nid2 in the same route


class Move {
  private:
    int nid1;   // node 1
    int nid2;   // node 2
    int rid1;   // route 1
    int rid2;   // route 2
    int pos1;   // position 1
    int pos2;   // position 2

  public:
    Move() { nid1 = nid2 = rid1 = rid2 = pos1 = pos2 = -1; };
    Move(int _nid1, int _nid2, int _rid1, int _rid2, int _pos1, int _pos2) {
        nid1 = _nid1;
        nid2 = _nid2;
        rid1 = _rid1;
        rid2 = _rid2;
        pos1 = _pos1;
        pos2 = _pos2;
    };

    int getnid1() const { return nid1; };
    int getnid2() const { return nid2; };
    int getrid1() const { return rid1; };
    int getrid2() const { return rid2; };
    int getpos1() const { return pos1; };
    int getpos2() const { return pos2; };

    bool operator==(Move &rhs) const;
    bool tabuEquiv(Move &tabu) const;
    void dump() const;

    void setnid1(int nid) { nid1 = nid; };
    void setnid2(int nid) { nid2 = nid; };
    void setrid1(int rid) { rid1 = rid; };
    void setrid2(int rid) { rid2 = rid; };
    void setpos1(int pos) { pos1 = pos; };
    void setpos2(int pos) { pos2 = pos; };

};

#endif
