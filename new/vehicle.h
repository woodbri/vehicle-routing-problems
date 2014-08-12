#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle : public Path {
  private:
    int capacity;
    Path route;
    double D;
    double cost;
    int TWV;
    int CV;

  public:
    int getcapacity() const { return capacity; };
    void setcapacity(int _capacity) { capacity = _capacity; };

    double distancetodepot(int i) { return route[i].distance(getdepot()); };
    double distancetodump(int i) { return route[i].distance(getdumpsite()); };

    double evaluate();

    void dump() const;


};


#endif

