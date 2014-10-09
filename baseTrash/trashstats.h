#ifndef TRASHSTATS_H
#define TRASHSTATS_H

#include <string>
#include <vector>
#include <map>

#include "singleton.h"

class TrashStats {
  private:
    std::map<std::string, double> stats;

  public:

    TrashStats() { stats.clear(); };
    ~TrashStats() {};

    double getval(const std::string key) const;
    std::vector<std::string> getkeys() const;
    void dump(const std::string title) const;

    void inc(const std::string key);
    void set(const std::string key, double val);
    void addto(const std::string key, double val);
    void clear() { stats.clear(); };
};

typedef Singleton<TrashStats> Stats; // Global declaration

#define STATS Stats::Instance()

#endif
/*
    Then you can access parameters via:

    STATS->method();
*/
