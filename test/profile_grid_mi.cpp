/**
 * Compute the time to compute the mutual information
 * in a grid at a variety of map sizes.
 */

#include <vector>
#include <chrono>
#include <fstream>
#include <iostream>
#include <range_mi/grid_mi.hpp>

#include "helpers.hpp"

int main() {

unsigned int num_beams = 200;
unsigned int max_side_length = 1000;
unsigned int max_area = max_side_length * max_side_length;

// Randomize the occupancy grid
std::vector<double> vacancy(max_area);
random_p(vacancy);

// Output file
std::ofstream f;
f.open("profile_grid_mi_results.txt");

for (unsigned int side_length = 1; side_length < max_side_length; side_length++) {
  std::cout << side_length << std::endl;
  // Initialize MI computation
  range_mi::GridMI mi_computer(side_length, side_length);

  // Set the clock to zero
  auto start = std::chrono::high_resolution_clock::now();

  // Compute MI
  mi_computer.compute_mi(vacancy.data(), num_beams);

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> total = end - start;
  
  f << side_length * side_length << " " << total.count() << std::endl;
}

f.close();

}
