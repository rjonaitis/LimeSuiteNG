#pragma once

namespace lime {

enum class TRXDir : bool { Rx, Tx };

struct Range {
    Range(double min = 0, double max = 0, double step = 0)
        : min(min)
        , max(max)
        , step(step){};
    double min;
    double max;
    double step;
};

template<class T> class DeltaVariable
{
  public:
    DeltaVariable(T init)
        : mValue(init)
        , mLastValue(0){};
    void set(T val) { mValue = val; }
    void add(T val) { mValue += val; }
    T delta() { return mValue - mLastValue; } // value change since last reset
    T value() const { return mValue; }
    void checkpoint() { mLastValue = mValue; }

  private:
    T mValue;
    T mLastValue;
};

} // namespace lime
