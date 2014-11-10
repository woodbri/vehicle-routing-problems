#ifndef VRP_OSRMCLIENT_H
#define VRP_OSRMCLIENT_H

#include "Library/OSRM.h"
#include "node.h"

#include <string>
#include <deque>
#include <vector>
#include <json/json.h>

class OsrmClient {

  private:

    OSRM * routing_machine;
    RouteParameters route_parameters;
    int status;
    std::string err_msg;
    std::string httpContent;
    std::vector<FixedPointCoordinate> viaPoints;

  public:

    OsrmClient();
    //~OsrmClient() { if (routing_machine) delete routing_machine; };
    void clear();
    void addViaPoint( double lat, double lon );
    void addViaPoint( const Node &node );
    void addViaPoints( const std::deque<Node> &path );
    void setWantGeometry( bool want ) { route_parameters.geometry = want; };
    bool getOsrmViaroute( bool wantGeom );
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
                  << "\nviaPoints.size(): " << viaPoints.size()
                  << "\nhttpContent: " << httpContent
                  << "\n";
    };
};
#endif
