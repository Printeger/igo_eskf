#ifndef BUTTERWORTH_FILTER_H
#define BUTTERWORTH_FILTER_H

#include <cmath>
#include <vector>

class ButterworthFilter {
 public:
  ButterworthFilter(double cutoff_frequency, double sampling_rate, int order)
      : cutoff_frequency_(cutoff_frequency),
        sampling_rate_(sampling_rate),
        order_(order) {
    calculateCoefficients();
  }

  double apply(double input) {
    // Shift the previous input and output values
    for (int i = order_; i > 0; --i) {
      x_[i] = x_[i - 1];
      y_[i] = y_[i - 1];
    }

    // Insert the new input value
    x_[0] = input;

    // Apply the filter formula
    double output = 0.0;
    for (int i = 0; i <= order_; ++i) {
      output += b_[i] * x_[i];
    }
    for (int i = 1; i <= order_; ++i) {
      output -= a_[i] * y_[i];
    }

    // Store the new output value
    y_[0] = output;

    return output;
  }

 private:
  void calculateCoefficients() {
    double PI = 3.14159265358979323846;
    double wc = tan(PI * cutoff_frequency_ / sampling_rate_);
    double k1 = wc * wc;
    double k2 = 1.414213562 * wc;
    double k3 = 1.0;

    double norm = k1 + k2 + k3;
    a_.resize(order_ + 1);
    b_.resize(order_ + 1);
    a_[0] = 1.0;
    a_[1] = 2.0 * (k1 - k3) / norm;
    a_[2] = (k1 - k2 + k3) / norm;
    b_[0] = k1 / norm;
    b_[1] = 2.0 * b_[0];
    b_[2] = b_[0];
    x_.resize(order_ + 1, 0.0);
    y_.resize(order_ + 1, 0.0);
  }

  double cutoff_frequency_;
  double sampling_rate_;
  int order_;
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> x_;
  std::vector<double> y_;
};

#endif  // BUTTERWORTH_FILTER_H