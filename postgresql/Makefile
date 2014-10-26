
CPP = g++
PG_CPPFLAGS = -DPGROUTING -DNDEBUG

VRPTOOLS_VERSION = 1.0
OBJS = vrptools.o
MODULE_big = vrptools-$(VRPTOOLS_VERSION)
EXTENSION = vrptools
DATA = vrptools--$(VRPTOOLS_VERSION).sql
DOCS = README.vrptools

CURL_CFLAGS := $(shell pkg-config --cflags curlpp)
CURL_LIBS := $(shell pkg-config --libs curlpp)
JSON_LIBS = -L /lib/x86_64-linux-gnu/ -ljson
SHLIB_LINK = $(CURL_LIBS) $(JSON_LIBS)


all:
ifndef PGVER
    @echo "make PGVER=9.2|9.3 clean|install"

else

PG_CONFIG = /usr/lib/postgresql/$(PGVER)/bin/pg_config
PGXS := $(shell $(PG_CONFIG) --pgxs)
include $(PGXS)

endif

