#ifndef VRP_OSRMCLIENT_H
#define VRP_OSRMCLIENT_H

#include "DataStructures/Coordinate.h"
#include "Server/DataStructures/RouteParameters.h"
#include "node.h"

#include <string>
#include <deque>
#include <vector>
#include <json/json.h>

class OsrmClient {

  private:

    RouteParameters route_parameters;
    int status;
    std::string err_msg;
    std::string httpContent;

  public:

    OsrmClient();
    void clear();
    void addViaPoint( double lat, double lon );
    void addViaPoint( const Node &node );
    void addViaPoints( const std::deque<Node> &path );
    void setWantGeometry( bool want ) { route_parameters.geometry = want; };
    bool getOsrmViaroute();
    bool getOsrmTime( double &time );
    bool getOsrmGeometry( std::deque<Node> &geom );
    int getStatus() const { return status; };
    std::string getErrorMsg() const { return err_msg; };

  private:
    bool getTime( struct json_object *jtree, double &time );
    bool getGeom( struct json_object *jtree, std::deque<Node> &geom );

  public:
    void dump() {
        std::cout << "----- OsrmClient ----------"
                  << "\nstatus: " << status
                  << "\nerr_msg: " << err_msg
                  << "\ncoordinates.size(): " << route_parameters.coordinates.size()
                  << "\nhttpContent: " << httpContent
                  << "\n";
    };
};
#endif
