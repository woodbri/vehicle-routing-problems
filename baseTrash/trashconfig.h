#ifndef TRASHCONFIG_H
#define TRASHCONFIG_H

#include <string>

#include "config.h"

/*!
 * \class TrashConfig
 * \brief Defines TrashConfig object and initializes some default attributes.
 *
 * TrashConfig is derived from Config and create a global singleton object
 * for storing key/value pairs for configuring the Trash Collection application.
 *
 * \var osrmBaseUrl Sets the location of the OSRM server to use.
 * \var plotDir Sets the location where plot files will get written.
 * \var plotFontFile Sets the location of the default font file for plots.
 * \bug plotFontFile varible may not be working in the code.
 *
 */
class TrashConfig : public Config {
  public:

    TrashConfig() : Config() {
        set("osrmBaseUrl", "http://localhost:5000/");
        set("plotDir",     "./");
        set("plotFontFile", "/usr/share/fonts/truetype/msttcorefonts/Verdana.ttf");
    };
    ~TrashConfig() {};
};

#endif

/*! \var typedef Singleton<TrashConfig> OurTrashConfig
 *  \brief A type definition for a our Config object.
 */
typedef Singleton<TrashConfig> OurTrashConfig;

/*! \def CONFIG
 *  \brief A macro to make it easier to access the our TrashConfig object.
 *
 *  The TrashConfig object is instanciated as a global static object and
 *  can be referenced using the CONFIG macro like:
 *  \code
    CONFIG->method();
 *  \endcode
 */
// redefine baseClasses CONFIG to access TrashConfig
#ifdef CONFIG
#undef CONFIG
#endif
#define CONFIG OurTrashConfig::Instance()

