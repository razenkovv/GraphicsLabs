#include <limits>
#include <cmath>

#include "functions.h"

WinInstance::WinInstance(sf::RenderWindow& window_) : window(window_) {
  window.clear(sf::Color::White);
  image.create(window.getSize().x, window.getSize().y, sf::Color::White);
  texture.loadFromImage(image);
  sprite.setTexture(texture);
  sprite.setPosition(0, 0);
}

void WinInstance::drawImage(bool update) {
  if (update) {
    window.clear(sf::Color::White);
    texture.update(image);
    sprite.setTexture(texture);
    window.draw(sprite);
    window.display();
  } else {
    window.clear(sf::Color::White);
    window.draw(sprite);
    window.display();
  }
}

void WinInstance::display() {
  window.display();
}

bool WinInstance::isOpen() const {
  return window.isOpen();
}

bool WinInstance::pollEvent(sf::Event& event) {
  return window.pollEvent(event);
}

void WinInstance::close() {
  window.close();
}

void WinInstance::clear(sf::Color color) {
  window.clear(color);
}

void WinInstance::lineBresenham(const Point<int>& s, const Point<int>& e, const sf::Color& color) {
  Point<int> start(s.x(), s.y());
  Point<int> end(e.x(), e.y());
  if (start.x() > end.x()) std::swap(start, end);
  int x, y;
  int ix, iy;
  int dx = abs(end.x() - start.x()), dy = abs(end.y() - start.y());
  bool swapped(false);

  if (dx >= dy) {
    x = start.x(), y = start.y();
    ix = sgn(end.x() - start.x()), iy = sgn(end.y() - start.y());
  } else {
    x = start.y(), y = start.x();
    ix = sgn(end.y() - start.y()), iy = sgn(end.x() - start.x());
    std::swap(dx, dy);
    swapped = true;
  }

  int error = 2 * dy - dx;
  for (int i = 0; i <= dx; i++) {
    if (!swapped)
      image.setPixel(x, y, color);
    else
      image.setPixel(y, x, color);
    if (error >= 0) {
      y += iy;
      error -= 2 * dx;
    }
    x += ix;
    error += 2 * dy;
  }
}

void WinInstance::polygon(const std::vector<Point<int>>& vertex, const sf::Color& color) {
  for (int i = 0; i < vertex.size() - 1; ++i) {
    lineBresenham(vertex[i], vertex[i + 1], color);
  }
}

bool WinInstance::checkConvex(const std::vector<Point<int>>& vertex) {
  pointType type = pointPositionToLineSegment(vertex[0], vertex[1], vertex[2]);
  for (int i = 0; i < vertex.size() - 2; i++) {
    for (int j = 0; j < vertex.size() - 1; j++) {
      if (j != i && j != i + 1) {
        if (pointPositionToLineSegment(vertex[j], vertex[i], vertex[i + 1]) != type)
          return false;
      }
    }
  }
  return true;
}

bool WinInstance::checkPolygonSimpicity(const std::vector<Point<int>>& vertex, Point<double>& p) {
  for (int i = 0; i < vertex.size() - 2; i++) {
    for (int j = 0; j < vertex.size() - 1; j++) {
      if (j != i && j != i + 1 && (j != (i > 0 ? i - 1 : vertex.size() - 2))) {
        if (lineSegmentsInterception(vertex[i], vertex[i + 1], vertex[j], vertex[j + 1], p) == interceptionType::CROSS)
          return false;
      }
    }
  }
  return true;
}

void WinInstance::boundingBox(const std::vector<Point<int>>& polygon, std::vector<Point<int>>& res) {
  int maxx = 0.0, maxy = 0.0, minx = std::numeric_limits<int>::max(), miny = std::numeric_limits<int>::max();
  for (Point<int> vertex : polygon) {
    if (vertex.x() > maxx)
      maxx = vertex.x();
    if (vertex.x() < minx)
      minx = vertex.x();
    if (vertex.y() > maxy)
      maxy = vertex.y();
    if (vertex.y() < miny)
      miny = vertex.y();
  }
  res[0].setx(minx), res[0].sety(miny);
  res[1].setx(minx), res[1].sety(maxy);
  res[2].setx(maxx), res[2].sety(maxy);
  res[3].setx(maxx), res[3].sety(miny);
  res[4].setx(minx), res[4].sety(miny);
  res.resize(5);
  res.shrink_to_fit();
}

pointTypeToPolygonEdge WinInstance::pttpe(const Point<int>& p, const Point<int>& p1, const Point<int>& p2) {
  switch (pointPositionToLineSegment(p, p1, p2)) {
    case pointType::LEFT:
      if (p.y() > p1.y() && p.y() <= p2.y())
        return pointTypeToPolygonEdge::CROSS_LEFT;
      else
        return pointTypeToPolygonEdge::INESSENTIAL;
    case pointType::RIGHT:
      if (p.y() > p2.y() && p.y() <= p1.y())
        return pointTypeToPolygonEdge::CROSS_RIGHT;
      else
        return pointTypeToPolygonEdge::INESSENTIAL;
    case pointType::BETWEEN:
    case pointType::ORIGIN:
    case pointType::DESTINATION:
      return pointTypeToPolygonEdge::TOUCHING;
    default:
      return pointTypeToPolygonEdge::INESSENTIAL;
  }
}

pointToPolygonType WinInstance::fillPointEvenOdd(const Point<int>& p, const std::vector<Point<int>>& polygon) {
  int param(0);
  for (int i = 0; i < polygon.size() - 1; i++) {
    switch (pttpe(p, polygon[i], polygon[i + 1])) {
      case pointTypeToPolygonEdge::TOUCHING:
        return pointToPolygonType::INSIDE;
      case pointTypeToPolygonEdge::CROSS_LEFT:
      case pointTypeToPolygonEdge::CROSS_RIGHT:
        param = 1 - param;
        break;
    }
  }
  if (param)
    return pointToPolygonType::INSIDE;
  else
    return pointToPolygonType::OUTSIDE;
}

pointToPolygonType WinInstance::fillPointNonZeroWinding(const Point<int>& p, const std::vector<Point<int>>& polygon) {
  int param(0);
  for (int i = 0; i < polygon.size() - 1; i++) {
    switch (pttpe(p, polygon[i], polygon[i + 1])) {
      case pointTypeToPolygonEdge::TOUCHING:
        return pointToPolygonType::INSIDE;
      case pointTypeToPolygonEdge::CROSS_LEFT:
        param++;
        break;
      case pointTypeToPolygonEdge::CROSS_RIGHT:
        param--;
        break;
    }
  }
  if (param)
    return pointToPolygonType::INSIDE;
  else
    return pointToPolygonType::OUTSIDE;
}

void WinInstance::fillPolygon(const std::vector<Point<int>>& polygon, const methods& method, const sf::Color& color) {
  std::vector<Point<int>> boundbox(5);
  boundingBox(polygon, boundbox);
  switch (method) {
    case methods::EO:
      for (int i = boundbox[0].x(); i <= boundbox[2].x(); i++) {
        for (int j = boundbox[0].y(); j <= boundbox[2].y(); j++) {
          if (fillPointEvenOdd(Point(i, j), polygon) == pointToPolygonType::INSIDE)
            image.setPixel(i, j, color);
        }
      }
      break;
    case methods::NZW:
      for (int i = boundbox[0].x(); i <= boundbox[2].x(); i++) {
        for (int j = boundbox[0].y(); j <= boundbox[2].y(); j++) {
          if (fillPointNonZeroWinding(Point(i, j), polygon) == pointToPolygonType::INSIDE)
            image.setPixel(i, j, color);
        }
      }
      break;
  }
}

bool WinInstance::saveImage(const std::string& filename) {
  return image.saveToFile(filename);
}

void WinInstance::curveBezier3(const std::vector<Point<int>>& points, const sf::Color& color) {
  if (points.size() != 4) {
    throw std::runtime_error("\n(curveBezier3) Cubic Bézier curves are plotted for 4 points.");
  }
  int H = std::max(abs(points[0].x() - 2 * points[1].x() + points[2].x()) + abs(points[0].y() - 2 * points[1].y() + points[2].y()),
                   abs(points[1].x() - 2 * points[2].x() + points[3].x()) + abs(points[1].y() - 2 * points[2].y() + points[3].y()));
  int N = std::ceil(1 + std::sqrt(3 * H));
  double tau = 1.0 / N;
  double t = tau;
  Point<int> p1, p2(points[0]);
  for (int i = 0; i < N - 1; ++i) {
    p1 = p2;
    p2 = std::pow(1 - t, 3) * points[0] + 3 * t * std::pow(1 - t, 2) * points[1] + 3 * std::pow(t, 2) * (1 - t) * points[2] + std::pow(t, 3) * points[3];
    t += tau;
    lineBresenham(p1, p2, color);
  }
  lineBresenham(p2, points[3], color);
}
