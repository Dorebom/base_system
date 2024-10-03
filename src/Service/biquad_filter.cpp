#include "biquad_filter.hpp"

#include <cmath>

BiquadFilter::BiquadFilter() {
    mode_ = BiquadMode::NO_MODE;
    is_init_ = false;
    sample_freq_ = 0.0;
    cutoff_freq_ = 0.0;
    q_value_ = 0.0;
    gain_ = 0.0;
    a0_ = 1.0;
    a1_ = 1.0;
    a2_ = 1.0;
    b0_ = 1.0;
    b1_ = 1.0;
    b2_ = 1.0;
    prev_output_1_ = 0.0;
    prev_output_2_ = 0.0;
    prev_input_1_ = 0.0;
    prev_input_2_ = 0.0;
}

BiquadFilter::BiquadFilter(BiquadMode mode, double sample_freq,
                           double cutoff_freq, double q_value, double gain) {
    mode_ = mode;
    is_init_ = false;
    sample_freq_ = sample_freq;
    cutoff_freq_ = cutoff_freq;
    q_value_ = q_value;
    gain_ = gain;
    a0_ = 1.0;
    a1_ = 1.0;
    a2_ = 1.0;
    b0_ = 1.0;
    b1_ = 1.0;
    b2_ = 1.0;
    prev_output_1_ = 0.0;
    prev_output_2_ = 0.0;
    prev_input_1_ = 0.0;
    prev_input_2_ = 0.0;
    calc_biquad_param();
}

BiquadFilter::~BiquadFilter() {
}

void BiquadFilter::reset() {
    is_init_ = false;
}

void BiquadFilter::calc_biquad_param() {
    double omega = 2.0 * M_PI * cutoff_freq_ / sample_freq_;
    double alpha = sin(omega) / (2.0 * q_value_);
    double beta = sqrt(gain_);
    double cos_omega = cos(omega);

    switch (mode_) {
        case BiquadMode::LPF:
            b0_ = (1.0 - cos_omega) * 0.5;
            b1_ = 1.0 - cos_omega;
            b2_ = (1.0 - cos_omega) * 0.5;
            a0_ = 1.0 + alpha;
            a1_ = -2.0 * cos_omega;
            a2_ = 1.0 - alpha;
            break;
        case BiquadMode::HPF:
            b0_ = (1.0 + cos_omega) * 0.5;
            b1_ = -(1.0 + cos_omega);
            b2_ = (1.0 + cos_omega) * 0.5;
            a0_ = 1.0 + alpha;
            a1_ = -2.0 * cos_omega;
            a2_ = 1.0 - alpha;
            break;
        case BiquadMode::BPF:
            // alpha ???
            b0_ = alpha;
            b1_ = 0.0;
            b2_ = -alpha;
            a0_ = 1.0 + alpha;
            a1_ = -2.0 * cos_omega;
            a2_ = 1.0 - alpha;
            break;
        case BiquadMode::NOTCH:
            // alpha ???
            b0_ = 1.0;
            b1_ = -2.0 * cos_omega;
            b2_ = 1.0;
            a0_ = 1.0 + alpha;
            a1_ = -2.0 * cos_omega;
            a2_ = 1.0 - alpha;
            break;
        default:
            break;
    }

    a0_norm = a0_ / a0_;
    a1_norm = a1_ / a0_;
    a2_norm = a2_ / a0_;
    b0_norm = b0_ / a0_;
    b1_norm = b1_ / a0_;
    b2_norm = b2_ / a0_;
}

void BiquadFilter::set_param_lpf(double sample_freq, double cutoff_freq,
                                 double q_value) {
    mode_ = BiquadMode::LPF;
    sample_freq_ = sample_freq;
    cutoff_freq_ = cutoff_freq;
    q_value_ = q_value;
    calc_biquad_param();
}

void BiquadFilter::set_param_hpf(double sample_freq, double cutoff_freq,
                                 double q_value) {
    mode_ = BiquadMode::HPF;
    sample_freq_ = sample_freq;
    cutoff_freq_ = cutoff_freq;
    q_value_ = q_value;
    calc_biquad_param();
}

void BiquadFilter::set_param_bpf(double sample_freq, double cutoff_freq,
                                 double q_value) {
    mode_ = BiquadMode::BPF;
    sample_freq_ = sample_freq;
    cutoff_freq_ = cutoff_freq;
    q_value_ = q_value;
    calc_biquad_param();
}

void BiquadFilter::set_param_notch(double sample_freq, double cutoff_freq,
                                   double q_value) {
    mode_ = BiquadMode::NOTCH;
    sample_freq_ = sample_freq;
    cutoff_freq_ = cutoff_freq;
    q_value_ = q_value;
    calc_biquad_param();
}

double BiquadFilter::update(double input) {
    double output = 0.0;

    if (!is_init_) {
        is_init_ = true;
        prev_output_1_ = input;
        prev_output_2_ = input;
        prev_input_1_ = input;
        prev_input_2_ = input;
    }

    output = b0_norm * input + b1_norm * prev_input_1_ +
             b2_norm * prev_input_2_ - a1_norm * prev_output_1_ -
             a2_norm * prev_output_2_;

    prev_input_2_ = prev_input_1_;
    prev_input_1_ = input;
    prev_output_2_ = prev_output_1_;
    prev_output_1_ = output;

    return output;
}