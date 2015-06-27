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
#ifndef TRASHPROB_H
#define TRASHPROB_H

#include <string>
#include <sstream>

//#include "vrptools.h"
#include "prob_trash.h"

class TrashProb : public Prob_trash
{
  std::vector<std::string> errorsFound;
  bool dataIsLatLon;

  template <typename T>
  std::string numbertoString ( T Number ) {
    std::stringstream ss;
    ss << Number;
    return ss.str();
  }

public:

  TrashProb(  container_t *p_containers, unsigned int container_count,
              otherloc_t *p_otherlocs, unsigned int otherloc_count,
              ttime_t *p_ttimes, unsigned int ttime_count,
              vehicle_t *p_vehicles, unsigned int vehicle_count,
              unsigned int check);


  void addContainers( container_t *containers, int count );
  void addOtherlocs( otherloc_t *otherlocs, int count );
  void addTtimes( ttime_t *ttimes, int count );
  void addVehicles( vehicle_t *vehicles, int count, unsigned int check );

  char **getErrorsForPg( int &count );
  bool isValid() const;
  //std::string whatIsWrong() const;
  std::string getErrorsString() const;
  void  whatIsWrong() ;

};

#endif
