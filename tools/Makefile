
ASTYLE=/usr/local/bin/astyle

all:
	@echo "Usage: make astyle|doxygen|clean"

astyle:
	$(ASTYLE) --options=astyle.conf "../*.cpp" "../*.h"

doxygen:
	mkdir -p ../build/doxy/
	doxygen

clean:
	rm -rf ../build/doxy/*
