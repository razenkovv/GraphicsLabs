#pragma once

#include <SFML/Graphics.hpp>
#include <string>
#include <vector>

#include <iostream>

template <typename T>
class Point;

enum class pointType { LEFT, RIGHT, BEHIND, BETWEEN, ORIGIN, DESTINATION };
enum class interceptionType { SAME, PARALLEL, CROSS, NO_CROSS };
enum class pointTypeToPolygonEdge { TOUCHING, CROSS_LEFT, CROSS_RIGHT, INESSENTIAL };
enum class pointToPolygonType { INSIDE, OUTSIDE };
enum class methods { EO, NZW };

class WinInstance {
 private:
  sf::RenderWindow& window;
  sf::Texture texture;
  sf::Sprite sprite;
  sf::Image image;

  template <typename T>
  interceptionType lineSegmentWithLineInterception(const Point<T>& a1, const Point<T>& a2, const Point<T>& b1, const Point<T>& b2, double& t);

  pointToPolygonType fillPointEvenOdd(const Point<int>& p, const std::vector<Point<int>>& polygon);

  pointToPolygonType fillPointNonZeroWinding(const Point<int>& p, const std::vector<Point<int>>& polygon);

 public:
  WinInstance(sf::RenderWindow&);
  void display();
  bool isOpen() const;
  bool pollEvent(sf::Event& event);
  void close();
  void drawImage(bool update = true);
  void clear(sf::Color color);

  void lineBresenham(const Point<int>& s, const Point<int>& e, const sf::Color& color);

  void polygon(const std::vector<Point<int>>& vertex, const sf::Color& color);

  template <typename T>
  pointType pointPositionToLineSegment(const Point<T>& p, const Point<T>& start, const Point<T>& end);

  bool checkConvex(const std::vector<Point<int>>& vertex);

  template <typename T>
  interceptionType lineSegmentsInterception(const Point<T>& a1, const Point<T>& a2, const Point<T>& b1, const Point<T>& b2, Point<double>& crossCoords);

  bool checkPolygonSimpicity(const std::vector<Point<int>>& vertex, Point<double>& p);

  void boundingBox(const std::vector<Point<int>>& polygon, std::vector<Point<int>>& res);

  pointTypeToPolygonEdge pttpe(const Point<int>& p, const Point<int>& p1, const Point<int>& p2);

  void fillPolygon(const std::vector<Point<int>>& polygon, const methods& method, const sf::Color& color);

  bool saveImage(const std::string& filename);
};

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

template <typename T>
class Point {
 private:
  T x_coord;
  T y_coord;

 public:
  Point() : x_coord(0), y_coord(0) {}
  Point(T x_, T y_) : x_coord(x_), y_coord(y_) {}
  T x() const { return x_coord; }
  T y() const { return y_coord; }
  void setx(T x_) { x_coord = x_; }
  void sety(T y_) { y_coord = y_; }
  void print() const { std::cout << x_coord << " " << y_coord << std::endl; }
};

template <typename T>
pointType WinInstance::pointPositionToLineSegment(const Point<T>& p, const Point<T>& start, const Point<T>& end) {
  T ax = end.x() - start.x();
  T by = p.y() - start.y();
  T bx = p.x() - start.x();
  T ay = end.y() - start.y();
  double s = ax * by - bx * ay;
  if (s > 0)
    return pointType::RIGHT;
  if (s < 0)
    return pointType::LEFT;
  if ((ax * bx < 0) || (ay * by < 0))
    return pointType::BEHIND;
  if ((ax * ax + ay * ay) < (bx * bx + by * by))
    return pointType::BEHIND;
  if (start.x() == p.x() && start.y() == p.y())
    return pointType::ORIGIN;
  if (end.x() == p.x() && end.y() == p.y())
    return pointType::DESTINATION;
  return pointType::BETWEEN;
}

template <typename T>
interceptionType WinInstance::lineSegmentWithLineInterception(const Point<T>& a1, const Point<T>& a2, const Point<T>& b1, const Point<T>& b2, double& t) {
  double nx = b2.y() - b1.y();
  double ny = b1.x() - b2.x();
  double denom = nx * (a2.x() - a1.x()) + ny * (a2.y() - a1.y());
  if (denom == 0) {
    pointType type = pointPositionToLineSegment(b1, b2, a1);
    if (type == pointType::LEFT || type == pointType::RIGHT)
      return interceptionType::PARALLEL;
    else
      return interceptionType::SAME;
  }
  double num = nx * (a1.x() - b1.x()) + ny * (a1.y() - b1.y());
  t = -num / denom;
  return interceptionType::CROSS;
}

template <typename T>
interceptionType WinInstance::lineSegmentsInterception(const Point<T>& a1, const Point<T>& a2, const Point<T>& b1, const Point<T>& b2, Point<double>& crossCoords) {
  double t(0.0);
  interceptionType type = lineSegmentWithLineInterception(b1, b2, a1, a2, t);
  if (type == interceptionType::SAME || type == interceptionType::PARALLEL)
    return type;
  if (t < 0.0 || t > 1.0)
    return interceptionType::NO_CROSS;
  lineSegmentWithLineInterception(a1, a2, b1, b2, t);
  if (t < 0.0 || t > 1.0)
    return interceptionType::NO_CROSS;
  crossCoords.setx(a1.x() + t * (a2.x() - a1.x()));
  crossCoords.sety(a1.y() + t * (a2.y() - a1.y()));
  return interceptionType::CROSS;
}
