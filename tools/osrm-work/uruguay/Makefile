#DATADIR = /var/www/osrm-data/uruguay
DATADIR = /home/stevew/work/vehicle-routing-problems/osrm-work/uruguay/

all:
	@echo "make start|kill|prep|uruguay|montevideo|clean-data|clean-logs|clean-all"

start: prep
	-rm -f nohup.out
	osrm-datastore $(DATADIR)/montevideo.osrm
	nohup osrm-routed --sharedmemory=1 &
#	./run-test-server ./montevideo.osrm -i 0.0.0.0 -p 5000 -t 3

kill:
	-sudo rm -f /tmp/osrm.lock
	-sudo rm -f /run/shm/*
	! osrm-datastore --sharedmemory=0
	! sudo killall osrm-routed

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

clean-data: kill
	rm -f uruguay-latest.* uruguay-download.md5 montevideo.*

clean-logs:
	rm -f log.prepare log.osmconvert stxxl.errlog stxxl.log
	@echo > nohup.out

