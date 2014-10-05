#ifndef TRASHCONFIG_H
#define TRASHCONFIG_H

#include <string>

#include "singleton.h"

class TrashConfig {
  public:
    std::string osrmBaseUrl;
    std::string plotDir;
    std::string plotFontFile;
    //double tabu_w1;
    //double tabu_w2;
    //double tabu_w3;

    TrashConfig() {
        osrmBaseUrl  = "http://localhost:5000/";
        plotDir      = "./";
        plotFontFile = "/usr/share/fonts/truetype/msttcorefonts/Verdana.ttf";
        //tabu_w1     = 1.0;
        //tabu_w2     = 1.0;
        //tabu_w3     = 1.0;
    };
    ~TrashConfig() {};
};

typedef Singleton<TrashConfig> Config; // Global declaration

#define CONFIG Config::Instance()

#endif
/*
    Then you can access parameters via:

    CONFIG->parameter
*/
