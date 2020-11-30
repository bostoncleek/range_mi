#include <cmath>

#include "range_mi/barely_distorted.hpp"
#include "range_mi/distorted.hpp"
#include "range_mi/p_not_measured.hpp"
#include "range_mi/grid_line.hpp"
#include "range_mi/grid_mi.hpp"

range_mi::GridMI::GridMI(
    unsigned int height_,
    unsigned int width_,
    double noise_dev_,
    double noise_half_width_,
    double integration_step_) :
  height(height_),
  width(width_),
  noise_dev(noise_dev_),
  noise_half_width(noise_half_width_),
  integration_step(integration_step_) {

  // Set the mutual information to zero
  mi_ = std::vector<double>(height * width, 0);

  // Initialize the conditioning map
  p_not_measured_ = std::vector<double>(height * width, 1);

  // Initialize the storage vectors
  line = std::vector<unsigned int>(2 * std::max(height, width));
  widths = std::vector<double>(2 * std::max(height, width));
  p_not_measured_single = std::vector<double>(height * width, 0);

  if (noise_dev > 0) {
    // Allocate free space for computing density functions
    pdf = std::vector<double>((4 * noise_half_width + 2)/integration_step);
  }
}

void range_mi::GridMI::compute_mi_beam(
    const double * const vacancy,
    double theta,
    double dtheta,
    double & spatial_interpolation) {

  // Convert the interpolation parameters to
  // x, y, theta
  range_mi::grid_line::sample(
      height, width,
      theta, x, y,
      spatial_interpolation);

  // Compute the intersections of
  // the line with the grid
  range_mi::grid_line::draw(
      height, width,
      x, y, theta,
      line.data(),
      widths.data(),
      num_cells);

  // Accumulate the mutual information
  if (noise_dev <= 0) {
    range_mi::barely_distorted::line<dimension, lower_bound>(
        line.data(),
        vacancy,
        p_not_measured_.data(),
        widths.data(),
        num_cells,
        dtheta,
        mi_.data());
  } else {
    range_mi::distorted::line<dimension>(
        line.data(),
        vacancy,
        widths.data(),
        num_cells,
        noise_dev,
        noise_half_width,
        integration_step,
        dtheta,
        pdf.data(),
        mi_.data());
  }
}

void range_mi::GridMI::compute_mi(
    const double * const vacancy,
    unsigned int num_beams) {

  // Iterate over beams spanning across the map
  double spatial_interpolation = 0;
  double theta = 0;
  double dtheta = (2 * M_PI)/num_beams;
  while (theta < 2 * M_PI) {

    // Compute the mutual information along
    // the beam in direction "theta", translated
    // by "spatial_interpolation". The spatial_interpolation
    // is updated by the compute_mi_beam function.
    compute_mi_beam(
        vacancy,
        theta,
        dtheta,
        spatial_interpolation);

    // Check to see if the spatial interpolation has
    // wrapped back to zero.
    if (spatial_interpolation == 0) {
      // If so, move to the next beam
      theta += dtheta;
    }
  }
}


void range_mi::GridMI::condition(
    const double * const vacancy,
    double x,
    double y,
    double theta_min,
    double theta_max,
    double dtheta) {

  // Clear the old p_measured
  std::fill(p_not_measured_single.begin(), p_not_measured_single.end(), 0);

  // Compute the new one for the given point
  for (double theta = theta_min; theta < theta_max; theta+=dtheta) {
    // Draw a line
    range_mi::grid_line::draw(
        height, width,
        x, y, theta,
        line.data(),
        widths.data(),
        num_cells);

    // Accumulate p_not_measured along the line
    range_mi::p_not_measured::line<dimension>(
        line.data(),
        vacancy,
        widths.data(),
        num_cells,
        dtheta,
        p_not_measured_single.data());
  }

  // Update the probabilities
  for (unsigned int i = 0; i < p_not_measured_.size(); i++) {
    if (lower_bound) {
      p_not_measured_[i] -= 1 - p_not_measured_single[i];
    } else {
      p_not_measured_[i] = std::min(p_not_measured_[i], p_not_measured_single[i]);
    }
    p_not_measured_[i] = std::min(p_not_measured_[i], 1.);
    p_not_measured_[i] = std::max(p_not_measured_[i], 0.);
  }
}
