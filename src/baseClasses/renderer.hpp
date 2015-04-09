// Implementation of rendering JSON objects built above using standard
// C++ output streams. The visitor pattern is used thanks to to build
// a "renderer" with boost::static_visitor and two top-level render
// routines are provided for rendering JSON objects and arrays.

#ifndef VRP_RENDERER_H
#define VRP_RENDERER_H

#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>

#include <osrm/json_container.hpp>


struct Renderer : mapbox::util::static_visitor<>
{
  Renderer(std::ostream& _out) : out(_out) {}

  void operator () (const osrm::json::String& string) const
  {
    out << "\"" << string.value << "\"";
  }

  void operator () (const osrm::json::Number& number) const
  {
    out.precision(10);
    out << number.value;
  }

  void operator () (const osrm::json::Object& object) const
  {
    out << "{";
    std::unordered_map<std::string, osrm::json::Value>::const_iterator iterator;
    iterator = object.values.begin();
    while (iterator != object.values.end()) {
      out << "\"" << (*iterator).first << "\":";
      mapbox::util::apply_visitor(Renderer(out), (*iterator).second);
      if (++iterator != object.values.end()) {
        out << ",";
      }
    }
    out << "}";
  }

  void operator () (const osrm::json::Array& array) const
  {
    out << "[";
    std::vector<osrm::json::Value>::const_iterator iterator;
    iterator = array.values.begin();
    while (iterator != array.values.end()) {
      mapbox::util::apply_visitor(Renderer(out), *iterator);
      if (++iterator != array.values.end()) {
        out << ",";
      }
    }
    out << "]";
  }

  void operator () (const osrm::json::True&) const
  {
    out << "true";
  }

  void operator () (const osrm::json::False&) const
  {
    out << "false";
  }

  void operator () (const osrm::json::Null&) const
  {
    out << "null";
  }

private:
  std::ostream& out;
};

inline void render(std::ostream& out, const osrm::json::Value& value)
{
  mapbox::util::apply_visitor(Renderer(out), value);
}

inline void render(std::ostream& out, const osrm::json::Object& object)
{
  osrm::json::Value value = object;
  mapbox::util::apply_visitor(Renderer(out), value);
}

inline void render(std::ostream& out, const osrm::json::Array& array)
{
  osrm::json::Value value = array;
  mapbox::util::apply_visitor(Renderer(out), value);
}

inline std::ostream& operator<<(std::ostream& out, const osrm::json::Value& value)
{
  render(out, value);
  return out;
}

inline std::ostream& operator<<(std::ostream& out, const osrm::json::Object& value)
{
  render(out, value);
  return out;
}

#endif                //VRP_RENDERER_H
