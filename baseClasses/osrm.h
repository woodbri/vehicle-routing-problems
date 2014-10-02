#ifndef OSRM_H
#define OSRM_H

#define OSRMCURL_GET 1
#define OSRMCURL_PUT 2
#define OSRMCURL_POST 3
#define OSRMCURL_DELETE 4

// curlpp specific
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

// json-c lib headers
#include <json/json.h>

class OSRM {

  private:
    std::string json;

  public:

    bool getTravelTime(double& ttime) const;
    bool getStatus(int& status) const;

    bool callOSRM(const std::string url);

};

#endif
