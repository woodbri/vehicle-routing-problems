
ASTYLE=/usr/bin/astyle

all:
	@echo "Usage: make astyle|doxygen|clean|fileheader"
	@cat README.md
	@echo "Usage: make astyle|doxygen|clean|fileheader"

astyle:
	$(ASTYLE) --options=google-style.conf "../*.cpp" "../*.h"

doxygen:
	mkdir -p ../build/doxy/
	doxygen

fileheader:
	./add-lic-to-source ..

clean:
	rm -rf ../build/doxy/*
