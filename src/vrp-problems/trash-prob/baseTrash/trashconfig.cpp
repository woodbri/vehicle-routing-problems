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


#ifdef OSRMCLIENT
#include "osrmclient.h"
#endif

#include "trashconfig.h"

/*!
 * \class TrashConfig
 * \brief Defines TrashConfig object and initializes some default attributes.
 *
 * TrashConfig is derived from Config and create a global singleton object
 * for storing key/value pairs for configuring the Trash Collection application.
 * The following keys are currently defined:
 *
 * \arg \c osrmBaseUrl Sets the location of the OSRM server to use.
 * \arg \c plotDir Sets the location where plot files will get written.
 * \arg \c plotFontFile Sets the location of the default font file for plots.
 * \bug \c plotFontFile varible may not be working in the code.
 *
 */

    TrashConfig::TrashConfig() : Config() {
	#ifdef DOPLOT
        set( "Plotting"," set to be done" );
        set( "plotDir",     "./" );
        set( "plotFontFile", "/usr/share/fonts/truetype/msttcorefonts/Verdana.ttf" );
	#endif

        #ifdef OSRMCLIENT
        osrm->useOsrm( true );
        #ifdef DOVRPLOG
           if ( osrm->getConnection() ) set ("osrmClient", " is available");
           else set( "OsrmClient", " is not available");
        #endif
        osrm->useOsrm( false );
        #else
        set( "osrmClient"," not set to be used" );
	#endif
	
	#ifdef DOSTATS
        set( "Statistics"," set to be calculated" );
	#endif

	#ifdef DOVRPLOG
        set( "Logging"," set to be done" );
	#endif

    };


