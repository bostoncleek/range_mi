#pragma once

#include <random>
#include <algorithm>

namespace wandering_robot {

class GridLine2D {

  public:

    GridLine2D() {}

    GridLine2D(unsigned int height_, unsigned int width_)
      : height(height_), width(width_) {

      // Initialize randomness
      std::random_device random_device;
      gen = std::mt19937(random_device());
      dist = std::uniform_real_distribution<double>(0., 1.);
    }

    void draw(
        double col,
        double row,
        double theta,
        unsigned int * const line,
        double * const widths,
        unsigned int & num_cells);

    void sample(
        double & x, 
        double & y, 
        double & theta);

    unsigned int size() {
      return 2 * std::max(height, width);
    }

  private:
    unsigned int height, width;
    std::mt19937 gen;
    std::uniform_real_distribution<double> dist;
};

}
