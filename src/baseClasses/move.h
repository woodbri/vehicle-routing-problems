/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
#ifndef MOVE_H
#define MOVE_H

#include <limits>
#include <iostream>
#include "vrp_assert.h"



/*!
 * \class Move
 *
 * \brief A class to define a neighborhood Move
 *
 * This class defines a move object that can be placed on the Tabu list
 * and/or can be applied to a given solution to transform it to a new state
 * setting attributes to -1 means they are undefined and should be ignored.
 *
 * We are working with three different moves types: Ins, InterSw, IntraSw
 * - Ins (insert)
 *   - remove a nid from vid1 at pos1 and insert it into vid2 as pos2
 * - InterSw (inter vehicle swap)
 *   - exchange a node with another node in another vehicle
 *     swap nid1 at pos1 in vid1 with nid2 at pos2 in vid2
 * - IntraSw (intra vehicle swap)
 *   - exchange nid1 and nid2 in the same vehicle
 *
 * \warning The vid1 and vid2 members are \c NOT the vehicle ids, they are
 *          the position of the vehicle in the fleet vector. Be careful to
 *          not change the order of the vehicles in the fleet array without
 *          updating the associated Move objects like on the TabuList or in
 *          any neighborhood lists.
 */
class Move
{
protected:
  typedef  unsigned long int UID ;
  typedef  unsigned long int POS ;
  typedef  unsigned long int UINT ;

public:
  class compMove
  {
  public:
    bool operator()( const Move &move1, const Move &move2 ) const {
      return ( Move::bySavings( move1, move2 ) );
    }
  };


  /*! \enum Mtype
   * Enumerated move type for Move
   */
  typedef enum {
    Invalid = -1,   ///< an invalid or undefined move
    Ins = 0,        ///< an Ins move that removes a node from one truck and inserts it into another truck
    IntraSw = 1,    ///< an IntraSw move that swaps two nodes in the same truck
    InterSw = 2     ///< an InterSw move that swaps nodes between two trucks
  } Mtype;

public:

  Move();
  Move( const Move &move );
  Move( Mtype _mtype, UID _nid1, UID _nid2, POS _vid1, POS _vid2, POS _pos1,
        POS _pos2, double _sav );

  void setInsMove( POS fromTruck, POS fromPos, UID fromId, POS toTruck, POS toPos,
                   double save );
  void setIntraSwMove( POS fromTruck, POS fromPos, UID fromId, POS withPos,
                       UID withId, double save );
  void setInterSwMove( POS fromTruck, POS fromPos, UID fromId, POS withTruck,
                       POS withPos, UID withId, double save );

  int getmtype() const { return mtype; };
  UID getnid1() const { return nid1; };
  UID getnid2() const { return nid2; };
  POS getvid1() const { return vid1; };
  POS getvid2() const { return vid2; };
  POS getpos1() const { return pos1; };
  POS getpos2() const { return pos2; };
  double getsavings() const { return savings; };

  bool less( const Move &m ) const;
  bool operator==( const Move &rhs ) const;
  /*!
   * \brief Create a \< operator for the less() function.
   */
  bool operator<( const Move &rhs ) const { return this->less( rhs ); };
  bool isForbidden( const Move &tabu ) const;
  bool isIns() const {return mtype == Move::Ins;};
  bool isIntraSw() const {return mtype == Move::IntraSw;};
  bool isInterSw() const {return mtype == Move::InterSw;};

  void dump() const;
  void Dump() const;

  void setmtype( Mtype _mtype ) { mtype = _mtype; };
  void setnid1( UID nid ) { nid1 = nid; };
  void setnid2( UID nid ) { nid2 = nid; };
  void setvid1( int vid ) { vid1 = vid; };
  void setvid2( int vid ) { vid2 = vid; };
  void setpos1( int pos ) { pos1 = pos; };
  void setpos2( int pos ) { pos2 = pos; };
  void setsavings( double save ) { savings = save; };

  /*!
   * \brief Function used to sort moves in by \c savings in decending order.
   */
  static bool bySavings( const Move &a, const Move &b ) { return ( a.getsavings() == b.getsavings() ) ? a<b : a.getsavings()>b.getsavings(); };

  /*!
   * \brief Function used to sort moves in by \c savings in asecending order.
   */
  static bool bySavingsA( const Move &a, const Move &b ) { return a.getsavings() < b.getsavings(); };

  UID getInsNid() const { return nid1; }
  POS getInsFromTruck() const { assert( mtype == Move::Ins ); return vid1; };
  POS getInsToTruck() const { assert( mtype == Move::Ins ); return vid2; };
  POS getInsFromPos() const { assert( mtype == Move::Ins ); return pos1; };
  POS getInsToPos() const { assert( mtype == Move::Ins ); return pos2; };
  void setInsFromPos( int newPos ) { assert( mtype == Move::Ins ); pos1 = newPos; };
  void setInsToPos( int newPos ) { assert( mtype == Move::Ins ); pos2 = newPos; };

  POS getIntraSwTruck() const { assert( mtype == Move::IntraSw ); return vid1; };
  POS getIntraSwFromPos() const { assert( mtype == Move::IntraSw ); return pos1; };
  POS getIntraSwToPos() const { assert( mtype == Move::IntraSw ); return pos2; };
  UID getIntraSwNid1() const { return nid1; }
  UID getIntraSwNid2() const { return nid2; }

  POS getInterSwTruck1() const { assert( mtype == Move::InterSw ); return vid1; };
  POS getInterSwFromPos() const { assert( mtype == Move::InterSw ); return pos1; };
  POS getInterSwTruck2() const { assert( mtype == Move::InterSw ); return vid2; };
  POS getInterSwToPos() const { assert( mtype == Move::InterSw ); return pos2; };

  bool isTabu( const Move &move ) const;
  bool isTabu( const Move &move, int rule ) const;

private:
  bool insForbidden( const Move &move_e, int rule ) const;
  bool insForbiddenRule0( const Move &other ) const;
  bool insForbiddenRule1( const Move &other ) const;
  bool insForbiddenRule2( const Move &other ) const;
  bool insForbiddenRule3( const Move &other ) const;
  bool insForbiddenRule4( const Move &other ) const;
  bool insForbiddenRule5( const Move &other ) const;
  bool insForbiddenRule6( const Move &other ) const;

private:
  Mtype mtype;    ///< type of move
  UID nid1;       ///< node id of first node
  UID nid2;       ///< node id of second node (if working with 2 nodes)
  POS vid1;       ///< vehicle 1 position in fleet vector
  POS vid2;       ///< vehicle 2 position in fleet vector (if working with 2 vehicles)
  POS pos1;       ///< nid1 position in vehicle at vid1 position
  POS pos2;       ///< nid2 position in vehicle at vid2 position
  double savings; ///< the savings generated by this move
};

#endif
