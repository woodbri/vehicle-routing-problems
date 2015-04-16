#DATADIR = /var/www/osrm-data/uruguay
DATADIR = /home/woodbri/work/vehicle-routing-problems/tools/osrm-work/uruguay/

all:
	@echo "make start|kill|prep|uruguay|montevideo|clean-data|clean-logs|status"

start: prep
	-echo > nohup.out
	osrm-datastore -s $(DATADIR)/montevideo.osrm
	nohup osrm-routed -s &
#	./run-test-server ./montevideo.osrm -i 0.0.0.0 -p 5000 -t 3

kill:
	-sudo rm -f /tmp/osrm.lock
	-sudo rm -f /run/shm/*
	! sudo killall osrm-routed
	! osrm-datastore --springclean

prep: montevideo.osrm

uruguay: uruguay-latest.osm.pbf

montevideo: montevideo.osm.pbf

uruguay-latest.osm.pbf:
	./wget-latest-uruguay

montevideo.osm.pbf: uruguay-latest.osm.pbf
	osmconvert uruguay-latest.osm.pbf -b=-56.45,-34.95,-56.0,-34.7 --complete-ways --out-pbf -o=montevideo.osm.pbf

montevideo.osrm: montevideo.osm.pbf
	@echo "Running extraction: osrm-extract montevideo.osm.pbf"
	time osrm-extract montevideo.osm.pbf > log.prepare
	@echo "Running osrm-prepare montevideo.osrm"
	time osrm-prepare montevideo.osrm >> log.prepare

clean-data:
	rm -f uruguay-latest.* uruguay-download.md5 montevideo.*
	$(MAKE) kill

clean-logs:
	rm -f log.prepare log.osmconvert stxxl.errlog stxxl.log
	@echo > nohup.out

status:
	sudo ipcs -m
	ps auxw | grep osrm
