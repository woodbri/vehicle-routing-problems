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
 * sudo apt-get install libjson0 libjson0-dev

Other requirments:

 * Build and install curlpp from source at https://code.google.com/p/curlpp/
 * You need to have a local OSRM server running with OSM data for your coverage area.

INSTALLATION
============

The installation is straight forward on Ubuntu:

```
make
sudo make install
```

This should build the extension and install it in your database server. To be able to access the functions you will need to create a database and create an extension.

```
createdb -U postgres -h localhost mytestdb
psql -U postgres -h localhost mytestdb -c "create extension postgis"
psql -U postgres -h localhost mytestdb -c "create extension osrm"
psql -U postgres -h localhost mytestdb -c "create extension vrptools"
```

Now you can load your data and will have access to the vrptools commands.


DOCUMENTATION
=============

See README.vrptools for specifics on the the PostgreSQL integration.
See the vehicle-routing-problems documentation for detailes on the specific solvers, there input, output and details of how they function.



SUPPORT
=======

We hope you find this project useful. Feel free to open tickets if you have issue, and to submit pull requests if you have fixes or enhancements to add.

If you are interested in paid support or custom solutions, please contact us at the above email address.

