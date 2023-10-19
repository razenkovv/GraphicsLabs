#include <SFML/Graphics.hpp>
#include <iostream>

#include "functions.h"

int main() {
  sf::RenderWindow window(sf::VideoMode(800, 600), "Window");

  WinInstance w(window);
  // simple polygon
  // std::vector<Point<int>> vertex{Point(100, 100), Point(200, 50), Point(600, 200), Point(780, 500), Point(300, 570), Point(123, 300), Point(100, 100)};

  // nonconvex simple polygon
  //  std::vector<Point<int>> vertex{Point(100, 100), Point(400, 400), Point(600, 200), Point(780, 500), Point(300, 570), Point(123, 300), Point(100, 100)};

  // nonsimple polygon
  // std::vector<Point<int>> vertex{Point(400, 25), Point(700, 550), Point(200, 100), Point(700, 100), Point(50, 550), Point(400, 25)};
  // std::vector<Point<int>> vertex{Point(100, 100), Point(200, 200), Point(100, 100)};

  // Bresenham algorithm
  // w.lineBresenham(Point(0, 0), Point(3, 8), sf::Color::Black);
  // w.lineBresenham(Point(3, 8), Point(0, 0), sf::Color::Red);
  // w.lineBresenham(Point(0, 0), Point(8, 3), sf::Color::Black);
  // w.lineBresenham(Point(8, 3), Point(0, 0), sf::Color::Red);
  // w.lineBresenham(Point(9, 0), Point(3, 8), sf::Color::Black);
  // w.lineBresenham(Point(3, 8), Point(9, 0), sf::Color::Red);
  //  w.lineBresenham(Point(678, 523), Point(100, 111), sf::Color::Red);
  //  w.lineBresenham(Point(345, 123), Point(742, 1), sf::Color::Green);

  // polygon draw
  //  w.polygon(vertex, sf::Color::Black);

  // check convex polygon or not
  // if (w.checkConvex(vertex))
  //   std::cout << "convex" << std::endl;
  // else
  //   std::cout << "nonconvex" << std::endl;

  // check lineSegmentsInterception method
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

  // check simple polygon or not
  //  Point<double> p1(0.0, 0.0);
  //  if (w.checkPolygonSimplicity(vertex, p1))
  //    std::cout << "simple" << std::endl;
  //  else {
  //    std::cout << "nonsimple" << std::endl;
  //    std::cout << p1.x() << " " << p1.y() << std::endl;
  //  }

  // get bounding box
  // std::vector<Point<int>> boundbox(5);
  // w.boundingBox(vertex, boundbox);
  // w.polygon(boundbox, sf::Color::Red);

  // methods::EO or methods::NZW
  // w.fillPolygon(vertex, methods::NZW, sf::Color::Green);

  // // Bezier curve
  // std::vector<Point<int>> points{Point(100, 500), Point(200, 100), Point(525, 110), Point(650, 400)};
  // // std::vector<Point<int>> points{Point(100, 500), Point(780, 110), Point(20, 100), Point(650, 400)}; // with loop
  // std::vector<Point<int>> vertex{points};
  // vertex.insert(vertex.end(), points[0]);
  // w.polygon(vertex, sf::Color::Red);
  // w.curveBezier3(points, sf::Color::Black);

  // Cyrus-Beck clipping
  std::vector<Point<int>> polygon{Point(100, 500), Point(200, 100), Point(525, 100), Point(650, 400), Point(100, 500)};
  // std::vector<Point<int>> polygon{Point(100, 500), Point(650, 400), Point(525, 110), Point(200, 100), Point(100, 500)};
  // Tests
  Point<int> p1(25, 500), p2(200, 300); // usual
  // Point<int> p1(25, 500), p2(700, 300); // usual
  // Point<int> p1(300, 400), p2(200, 300); // inside
  // Point<int> p1(600, 450), p2(700, 500);  // outside
  // Point<int> p1(50, 50), p2(200, 50); //parallel

  w.polygon(polygon, sf::Color::Green);
  w.lineBresenham(p1, p2, sf::Color::Black);
  Point<int> p1_new, p2_new;
  if (w.clipLineCyrusBeck(polygon, p1, p2, p1_new, p2_new)) {
    w.lineBresenham(p1_new, p2_new, sf::Color::Red);
  } else {
    std::cout << "segment is outside the polygon" << std::endl;
  }

  w.saveImage("image.png");
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