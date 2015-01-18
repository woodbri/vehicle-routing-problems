Extension to Integrate VRP tool into Postgresql
===============================================

Author: Stephen Woodbridge <woodbri (at) swoodbridge (dot) com>
Date: 2014-10-16
License: BSD 2-clause

-----------------------------------------------------------------------

This is a PostgreSQL extension that provides access to VRP tools via plpgsql.

This extension is designed to work with pgRouting and may get merged into that project at some point.

DEPENDENCIES
============

The follow packages are required to build this on Ubuntu:

 * sudo apt-get install curl libcurl3 libcurl4-gnutls-dev

Other requirments:

 * Build and install curlpp from source at https://code.google.com/p/curlpp/
 * You need to have OSRM datastore running on the same host with OSM data
   for your coverage area. We are using a sharedmemory connection to that.
 * Follow the following to install rapidjson
   \code{.bash}
   git clone https://github.com/miloyip/rapidjson.git
   cd rapidjson
   sudo rsync -a include/* /usr/local/include/.
   \endcode
   Then in the Makefile, ensure -I/usr/local/include is included in CFLAGS

INSTALLATION
============

The installation is straight forward on Ubuntu:

\code{.bash}
make
sudo make install
\endcode

This should build the extension and install it in your database server. To be able to access the functions you will need to create a database and create an extension.

\code{.bash}
createdb -U postgres -h localhost mytestdb
psql -U postgres -h localhost mytestdb -c "create extension postgis"
psql -U postgres -h localhost mytestdb -c "create extension osrm"
psql -U postgres -h localhost mytestdb -c "create extension vrptools"
\endcode

Now you can load your data and will have access to the vrptools commands.


DOCUMENTATION
=============

See README.vrptools for specifics on the the PostgreSQL integration.
See the vehicle-routing-problems documentation for detailes on the specific solvers, there input, output and details of how they function.



SUPPORT
=======

We hope you find this project useful. Feel free to open tickets if you have issue, and to submit pull requests if you have fixes or enhancements to add.

If you are interested in paid support or custom solutions, please contact us at the above email address.

