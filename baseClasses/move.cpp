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

#include <iostream>

#ifdef LOG
#include "logger.h"
#endif

#ifdef DOSTATS
#include "stats.h"
#endif

#include "move.h"

/*!
 * \brief Construct a Move object where the move is not defined and mtype is Invalid
 */
Move::Move() {
    #ifdef DOSTATS
    STATS->inc( "Move::Move (invalid) " );
    #endif
    mtype = Invalid; nid1 = nid2 = vid1 = vid2 = pos1 = pos2 = -1;
    savings = -std::numeric_limits<double>::max();
};

/*!
 * \brief Construct a Move object and assign the appropriate values.
 */
Move::Move( Mtype _mtype, int _nid1, int _nid2, int _vid1, int _vid2, int _pos1,
            int _pos2, double _sav ) {
    #ifdef DOSTATS
    STATS->inc( "Move::Move (valid 8 arguments) " );
    #endif
    mtype = _mtype;
    nid1 = _nid1;
    nid2 = _nid2;
    vid1 = _vid1;
    vid2 = _vid2;
    pos1 = _pos1;
    pos2 = _pos2;
    savings = _sav;
};

Move::Move( const Move &move ) {
    #ifdef DOSTATS
    STATS->inc( "Move::Move (copy) " );
    #endif
    mtype = move.mtype;
    nid1 = move.nid1;
    nid2 = move.nid2;
    vid1 = move.vid1;
    vid2 = move.vid2;
    pos1 = move.pos1;
    pos2 = move.pos2;
    savings = move.savings;
};




/*!
 * \brief Compare two moves and report if they are equal
 */
bool Move::operator==( const Move &rhs ) const {
    return nid1 == rhs.nid1 and nid2 == rhs.nid2 and
           vid1 == rhs.vid1 and vid2 == rhs.vid2 and
           pos1 == rhs.pos1 and pos2 == rhs.pos2;
}

/*!
 * \brief Compare two moves and report if move \< rhs.move
 *
 * This less than comparision operator is used by the TabuList map function
 * for ordering the moves in the container.
 */
bool Move::less( const Move &m ) const {
    return nid1 < m.nid1 or
           ( nid1 == m.nid1 and
             ( nid2 < m.nid2 or
               ( nid2 == m.nid2 and
                 ( vid1 < m.vid1 or
                   ( vid1 == m.vid1 and
                     ( pos1 < m.pos1 or
                       ( pos1 == m.pos1 and
                         pos2 < m.pos2 ) ) ) ) ) ) );
}

/*!
 * \brief Check if a given move is forbidden based on the TabuList
 *
 * The isForbidden() test is not associative ( ie: A.equiv(B) != B.equiv(A) ).
 * This test is used to determine if A would be tabu if B is on the tabu list
 *
 * - prohibition rules for Ins
 *   - Rule PR5 - move removing any order from tabu.vid1.
 *     This rule basically says if we remove a node from vid1
 *     then we are not allowed to add a node back to vid1 until
 *     the tabu length expires.
 *     This rule is to promote the elimiation of vehicles
 *
 * - prohibition rules for IntraSw
 *   - This rules says that if we have swapped either the source
 *     or the destination nodes that they can not be moved again
 *     until the tabu length expires.
 *
 * - prohibition rules for InterSw
 *   - This rules says that if we have swapped either the source
 *     or the destination nodes that they can not be moved again
 *     until the tabu length expires.
 */
bool Move::isForbidden( const Move &tabu ) const {
    if ( *this == tabu ) return true;

    if ( mtype == Ins ) {
        if ( vid2 == tabu.vid1 ) return true;
    }
    else if ( mtype == IntraSw ) {
        if ( nid1 == tabu.nid1 or nid2 == tabu.nid2 or
             nid1 == tabu.nid2 or nid2 == tabu.nid1 ) return true;
    }
    else {
        if ( nid1 == tabu.nid1 or nid2 == tabu.nid2 or
             nid1 == tabu.nid2 or nid2 == tabu.nid1 ) return true;
    }

    return false;
}


/*!
 * \brief Print the move.
 */
void Move::dump() const {
#ifdef LOG
    DLOG( INFO ) << "Move: " << mtype
                 << ",\t" << nid1
                 << ",\t" << nid2
                 << ",\t" << vid1
                 << ",\t" << vid2
                 << ",\t" << pos1
                 << ",\t" << pos2
                 << ",\t" << savings;
#endif
}

/*!
 * \brief Print the move in a more explict format.
 */
void Move::Dump() const {
#ifdef LOG
    switch ( mtype ) {
        case Ins:
            DLOG( INFO ) << "Move: Ins"
                         << "\t    NodeID:" << nid1
                         << "\tFrom Truck:" << vid1
                         << "\t  From Pos:" << pos1
                         << "\t  To Truck:" << vid2
                         << "\t    To Pos:" << pos2
                         << "\t   savings:" << savings;
            break;

        case IntraSw:
            DLOG( INFO ) << "Move: IntraSw"
                         << "\t    NodeID:" << nid1
                         << "\t  At Truck:" << vid1
                         << "\t  From Pos:" << pos1
                         << "\t  with NID:" << nid2
                         << "\t    To Pos:" << pos2
                         << "\t   savings:" << savings;
            break;

        case InterSw:
            DLOG( INFO ) << "Move: InterSw"
                         << "\t    NodeID:" << nid1
                         << "\t  (at Truck:" << vid1
                         << "\t  at Pos:" << pos1
                         << ")\t with NodeID:" << nid2
                         << "\t    (at Truck:" << vid2
                         << "\t      at Pos:" << pos2
                         << ")\t   savings:" << savings;
            break;
    }
#endif
}

void Move::setInsMove( int fromTruck, int fromPos, int fromId, int toTruck,
                       int toPos, double save ) {
    #ifdef DOSTATS
    STATS->inc( "Move::setInsMove " );
    #endif
    vid1 = fromTruck; pos1 = fromPos; nid1 = fromId;
    vid2 = toTruck;   pos2 = toPos;   nid2 = fromId;
    savings = save; mtype = Ins;
};

void Move::setIntraSwMove( int fromTruck, int fromPos, int fromId, int withPos,
                           int withId, double save ) {
    #ifdef DOSTATS
    STATS->inc( "Move::setIntraSwMove " );
    #endif
    vid1 = fromTruck; pos1 = fromPos; nid1 = fromId;
    vid2 = fromTruck; pos2 = withPos; nid2 = withId;
    savings = save; mtype = IntraSw;
};

void Move::setInterSwMove( int fromTruck, int fromPos, int fromId,
                           int withTruck, int withPos, int withId, double save ) {
    #ifdef DOSTATS
    STATS->inc( "Move::setInterSwMove " );
    #endif
    vid1 = fromTruck; pos1 = fromPos; nid1 = fromId;
    vid2 = withTruck; pos2 = withPos; nid2 = withId;
    savings = save; mtype = InterSw;
};


bool Move::isTabu( const Move &move_e ) const  {
    if ( not mtype == move_e.mtype ) return false;

    int rule;

    switch ( mtype ) {
        case Ins: rule = 5; break;

        case IntraSw: rule = 0; break;

        case InterSw: rule = 0; break;
    }

    return isTabu( move_e, rule );
};


bool Move::isTabu( const Move &move_e, int rule ) const  {
    if ( not ( mtype == move_e.mtype ) )  return false;

    switch ( mtype ) {
        case Move::Ins: return insForbidden( move_e, rule );

        case Move::IntraSw: return move_e.isForbidden( *this );

        case Move::InterSw: return move_e.isForbidden( *this );
    }
};



// for the follwoing functions ARE PROHIBITION RULES for INS
// always *this is the one in the tabu list

bool Move::insForbidden( const Move &move_e, int rule ) const {
    assert ( move_e.mtype == Move::Ins );

    switch ( rule ) {
        case 0: return insForbiddenRule0( move_e );

        case 1: return insForbiddenRule1( move_e );

        case 2: return insForbiddenRule2( move_e );

        case 3: return insForbiddenRule3( move_e );

        case 4: return insForbiddenRule4( move_e );

        case 5: return insForbiddenRule5( move_e );

        case 6: return insForbiddenRule6( move_e );
    }
}




/**
  semi inverse Move is prohibited

        M_t (Nid_t , fromTruck_t, fromPos_t, toTruck_t, toPos_t)
        M_e (Nid_e , fromTruck_e, fromPos_e, toTruck_e, toPos_e)

  True when:
        ( Nid_e = Nid_t  and fromTruck_e = toTruck_t

  Example:
  (A, T1,i  T2,j)   is (*this)  move
  (A, T2,j  T1,i)   is move to be evaluated

  The evaluated move is moving back the container to the same position
  as before

  Therfore returns True
*/
bool Move::insForbiddenRule0 ( const Move &move_e ) const {
    return ( move_e.getInsNid() == getInsNid()
             and move_e.getInsFromTruck() == getInsToTruck()
             and move_e.getInsToTruck() == getInsFromTruck() );
}




/**
  forbidding moving the container back to the original truck

    M_t (Nid_t , fromTruck_t, fromPos_t, toTruck_t, toPos_t)
    M_e (Nid_e , fromTruck_e, fromPos_e, toTruck_e, toPos_e)

  True when:
    ( Nid_e = Nid_t  and fromTruck_e = toTruck_t

  Example:
  (A, T1,i  T2,j)   is (*this)  move
  (A, T2,j  Tany,k)   is move to be evaluated

  both have the same  A (Nid)
  The evaluated move is taking out the contaier from a truck we placed it on
    even when it comes from a different truck

  Therfore returns True
*/

bool Move::insForbiddenRule1 ( const Move &move_e ) const {
    return ( move_e.getInsNid() == getInsNid()
             and move_e.getInsFromTruck() == getInsToTruck() );
}



/**
  forbidding inserting a container to a truck when the truck it comes from is
  a truck we inserted to

        M_t (Nid_t , fromTruck_t, fromPos_t, toTruck_t, toPos_t)
        M_e (Nid_e , fromTruck_e, fromPos_e, toTruck_e, toPos_e)

  True when:
        ( fromTruck_e = toTruck_t  and toTruck_e = fromTruck_t

  Example:
  (A, T1,i  T2,j)   is (*this) tabued  move
  (B, T2,j  T1,k)   is move to be evaluated

  The evaluating move is putting a contaier from T2 to T1

  Therfore returns True
*/

bool Move::insForbiddenRule2( const Move &move_e ) const {
    return ( move_e.getInsFromTruck() == getInsToTruck()
             and move_e.getInsToTruck() == getInsFromTruck() );
}



/**
  forbidding reinserting a container back to the original truck

        M_t (Nid_t , fromTruck_t, fromPos_t, toTruck_t, toPos_t)
        M_e (Nid_e , fromTruck_e, fromPos_e, toTruck_e, toPos_e)

  True when:
        ( Nid_e = Nid_t  and toTruck_e = fromTruck_t

  Example:
  (A, T1,i  T2,j)   is (*this) tabued move
  (A, Tany,j  T1,k)   is move to be evaluated

  both have the same  A (Nid)
  The evaluated move is putting back the contaier in the original truck
        even when it comes from a different truck

  Therfore returns True
*/
bool Move::insForbiddenRule3 ( const Move &move_e ) const {
    return ( move_e.getInsNid() == getInsNid()
             and move_e.getInsToTruck() == getInsFromTruck() );
}

/**
  forbidding to move a container recently moved

        M_t (Nid_t , fromTruck_t, fromPos_t, toTruck_t, toPos_t)
        M_e (Nid_e , fromTruck_e, fromPos_e, toTruck_e, toPos_e)

  True when:
         Nid_e = Nid_t

  Example:
  (A, T1,i  T2,j)   is (*this)  tabued move
  (A, Tx,k  Ty,l)   is move to be evaluated

  both have the same  A (Nid)
  we already moved A

  Therfore returns True
*/
bool Move::insForbiddenRule4 ( const Move &move_e ) const {
    return ( move_e.getInsNid() == getInsNid() );
}


/**
  forbidding to remove a container from a recently inserted Truck

        M_t (Nid_t , fromTruck_t, fromPos_t, toTruck_t, toPos_t)
        M_e (Nid_e , fromTruck_e, fromPos_e, toTruck_e, toPos_e)

  True when:
        fromTruck_e = toTruck_t

  Example:
  (A, T1,i  T2,j)   is (*this)  tabued move
  (B, T2,k  Ty,l)   is move to be evaluated

  a move was recently inserted into T2 so we dont allow removing from T2

  Therfore returns True
*/
bool Move::insForbiddenRule5 ( const Move &move_e ) const {
    return ( move_e.getInsFromTruck() == getInsToTruck() );
}

/**
  forbidding to reinsert any order to a truck we used to remove from

        M_t (Nid_t , fromTruck_t, fromPos_t, toTruck_t, toPos_t)
        M_e (Nid_e , fromTruck_e, fromPos_e, toTruck_e, toPos_e)

  True when:
         toTruck_e = fromTruck_t

  Example:
  (A, T1,i  T2,j)   is (*this)  tabued move
  (B, Tx,k  T1,l)   is move to be evaluated

  trying to insert an order to a truck we removed from
  Therfore its frobidden
  returns True
*/
bool Move::insForbiddenRule6 ( const Move &move_e ) const {
    return ( move_e.getInsToTruck() == getInsFromTruck() );
}





