# OSRM back-end server for VRP

We use the OSRM server for computing routes and travel time in some of our
vehicle routing problem solvers. This requires setting up a OSRM server and
preparing data for it. The goal of this document is to walk throught the
steps to get it built and run locally.

## Building OSRM

I use an older version of git at commit fbbd6ca because I'm running Ubuntu
12.04 LTS but I think you can use up to tag v0.3.10.

https://github.com/Project-OSRM/osrm-backend/wiki/Building-on-Ubuntu

Read the above link to install all the required dependencies.

```
git clone https://github.com/Project-OSRM/osrm-backend.git
cd osrm-backend
git checkout v0.3.10
mkdir build
cd build
cmake ..
make
sudo make install
```

This will create and install:

 * osrm-extract - tool to read .osm and .pbf file to create OSRM input
 * osrm-prepare - tool to read OSRM input and create a graph file for the router
 * osrm-routed - the OSRM routing engine
 * osrm-datastore - tool for managing OSRM instances in memory


We will only work with the first three of these.

## Download data, extract and prepare it

You will need to download your area of interest that covers the area that you
plan to make routes in. Google is you friend:

  https://www.google.com/?gws_rd=ssl#newwindow=1&q=osm+pbf+download

Find an appropriate file and fetch it. We will use Uruguay for our example.

  http://download.geofabrik.de/south-america/uruguay.html

There are a bunch of support files that need to be present for OSRM to work
a sample set is provided in the ``osrm-work`` directory along with the scripts
mentioned below.

```
cd /path/for/osrm-work/
wget -N http://download.geofabrik.de/south-america/uruguay-latest.osm.bz2
time osrm-extract uruguay-latest.osm.pbf
time osrm-prepare uruguay-latest.osrm
```

There are shell scripts to automate this. Edit them for your needs:

 * wget-latest-uruguay
 * do-prepare
 * run-test-server

## Running the OSRM Server

As mentioned above there are some needed support files and if you have 
installed the ``osrm-work`` directory and prepared the data there then
all you need to do it run the ``run-test-server`` to get it running. If
you want to have it start on boot up, I'll leave that as an exercise for
the user.

I have included a server.ini file which osrm-routed reads. I think it has been
deprecated in favor of commandline options. ``osrm-routed --help`` will
display them. For example it doesnot look like the router honors the Port
value in the server.ini file so it should eventually be removed and the
script updated to use the commandline options.


