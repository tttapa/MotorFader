#pragma once

class EMA_f {
  public:
    EMA_f(float pole) : alpha(1 - pole) {}
    float filter(float value) {
      filtered = filtered + (value - filtered) * alpha;
      return filtered;
    }
    float operator()(float value) { return filter(value); }
  private:
    const float alpha;
    float filtered = 0;
};
