#pragma once

enum class BiquadMode
{
    NO_MODE,
    LPF,
    HPF,
    BPF,
    NOTCH  //,
    // PEAK,
    // LOW_SHELF,
    // HIGH_SHELF
};

class BiquadFilter {
private:
    BiquadMode mode_;

    bool is_init_;

    double sample_freq_;
    double cutoff_freq_;
    double q_value_;
    double gain_;

    double a0_, a1_, a2_;
    double b0_, b1_, b2_;

    double a0_norm, a1_norm, a2_norm;
    double b0_norm, b1_norm, b2_norm;

    double prev_output_1_, prev_output_2_;
    double prev_input_1_, prev_input_2_;

    void calc_biquad_param();

public:
    BiquadFilter();
    BiquadFilter(BiquadMode mode, double sample_freq, double cutoff_freq,
                 double q_value, double gain);
    ~BiquadFilter();

    void set_param_lpf(double sample_freq, double cutoff_freq, double q_value);
    void set_param_hpf(double sample_freq, double cutoff_freq, double q_value);
    void set_param_bpf(double sample_freq, double cutoff_freq, double q_value);
    void set_param_notch(double sample_freq, double cutoff_freq,
                         double q_value);

    // void set_mode(BiquadMode mode);
    // void set_sample_rate(double sample_rate);
    // void set_cutoff_freq(double cutoff_freq);
    // void set_q_value(double q_value);
    // void set_gain(double gain);

    void reset();
    double update(double input);
};