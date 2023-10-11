#include <SFML/Graphics.hpp>
#include <iostream>

#include "functions.h"

int main() {
  sf::RenderWindow window(sf::VideoMode(800, 600), "Window");

  WinInstance w(window);

  // std::vector<Point<int>> vertex{Point(100, 100), Point(200, 50), Point(600, 200), Point(780, 500), Point(300, 570), Point(123, 300), Point(100, 100)};
  std::vector<Point<int>> vertex{Point(400, 25), Point(700, 550), Point(200, 100), Point(700, 100), Point(50, 550), Point(400, 25)};

  // w.lineBresenham(Point(500, 256), Point(100, 567), sf::Color::Black);
  // w.lineBresenham(Point(678, 523), Point(100, 111), sf::Color::Red);
  // w.lineBresenham(Point(345, 123), Point(742, 1), sf::Color::Green);

  w.polygon(vertex, sf::Color::Black);

  if (w.checkConvex(vertex))
    std::cout << "convex" << std::endl;
  else
    std::cout << "nonconvex" << std::endl;

  // Point<double> p(0.0, 0.0);
  // interceptionType type = w.lineSegmentsInterception(vertex[0], vertex[1], vertex[2], vertex[3], p);
  // switch (type) {
  //   case interceptionType::SAME:
  //     std::cout << "SAME\n";
  //     break;
  //   case interceptionType::PARALLEL:
  //     std::cout << "PARALLEL\n";
  //     break;
  //   case interceptionType::CROSS:
  //     std::cout << "CROSS\n";
  //     break;
  //   case interceptionType::NO_CROSS:
  //     std::cout << "NO_CROSS\n";
  //     break;
  // }
  // std::cout << p.x() << " " << p.y() << std::endl;

  Point<double> p1(0.0, 0.0);
  if (w.checkPolygonSimpicity(vertex, p1))
    std::cout << "simple" << std::endl;
  else {
    std::cout << "nonsimple" << std::endl;
    std::cout << p1.x() << " " << p1.y() << std::endl;
  }

  std::vector<Point<int>> boundbox(5);
  w.boundingBox(vertex, boundbox);
  w.polygon(boundbox, sf::Color::Red);

  w.fillPolygon(vertex, methods::NZW, sf::Color::Green);

  w.drawImage();

  while (w.isOpen()) {
    sf::Event event;
    while (w.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        w.close();
    }
  }

  return 0;
}