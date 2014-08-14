#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle : public Path {
  private:
    int maxcapacity;

    int curcapacity;
    double duration;
    double cost;
    int TWV;
    int CV;

    double w1;
    double w2;
    double w3;

  public:

    Vehicle() {
        maxcapacity = 0;
        curcapacity = 0;
        duration    = 0;
        cost        = 0;
        TWV         = 0;
        CV          = 0;
        w1 = w2 = w3 = 1.0;
    };

    int getmaxcapacity() const { return maxcapacity; };
    int getTWV() const { return TWV; };
    int getCV() const { return CV; };
    int getcurcapacity() const { return curcapacity; };
    double getduration() const { return duration; };
    double getcost() const { return cost; };
    double getw1() const { return w1; };
    double getw2() const { return w2; };
    double getw3() const { return w3; };

    void setmaxcapacity(int _maxcapacity) { maxcapacity = _maxcapacity; };
    void setweights(double _w1, double _w2, double _w3) {
        w1 = _w1;
        w2 = _w2;
        w3 = _w3;
    };


    void evaluate();

    double distancetodepot(int i) { return path[i].distance(getdepot()); };
    double distancetodump(int i) { return path[i].distance(getdumpsite()); };

    void dump();


};


#endif

