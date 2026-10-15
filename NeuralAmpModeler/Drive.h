#pragma once

#include <algorithm>
#include <cmath>

namespace dsp
{
namespace drive
{

// Tube Screamer-like drive, mono, zero-latency, cheap.
// Bypass is handled by the caller (kDriveActive).
class TubeScreamer
{
public:
  TubeScreamer() = default;

  void Reset(const double sampleRate)
  {
    mSampleRate = sampleRate > 0.0 ? sampleRate : 48000.0;
    mPreHP.SetCutoff(mSampleRate, 720.0);
    mPreLP.SetCutoff(mSampleRate, 7000.0);
    mPostLP.SetCutoff(mSampleRate, 3000.0);
    mPreHP.Reset();
    mPreLP.Reset();
    mPostLP.Reset();
  }

  // gain: 0..1 (0 = clean, 1 = full saturation)
  // volume: 0..2 (1 = unity-ish)
  void SetParams(const double gain, const double volume)
  {
    mGain = std::clamp(gain, 0.0, 1.0);
    mVolume = std::max(volume, 0.0);
    mGainLinear = 1.0 + mGain * 99.0;
    mOutputComp = 1.0 / (1.0 + mGain * 4.0);
  }

  void Process(const float* in, float* out, const size_t numFrames)
  {
    for (size_t i = 0; i < numFrames; i++)
    {
      float x = in[i];
      x = mPreHP.Process(x);
      x = mPreLP.Process(x);
      x *= static_cast<float>(mGainLinear);
      x = _Clip(x);
      x = mPostLP.Process(x);
      out[i] = x * static_cast<float>(mOutputComp * mVolume);
    }
  }

private:
  struct OnePoleHP
  {
    double a = 0.0, x1 = 0.0, y1 = 0.0;
    void SetCutoff(const double sr, const double fc)
    {
      const double rc = 1.0 / (2.0 * 3.14159265358979323846 * fc);
      const double dt = 1.0 / sr;
      a = rc / (rc + dt);
    }
    void Reset() { x1 = y1 = 0.0; }
    inline float Process(const float x)
    {
      const double y = a * (y1 + x - x1);
      x1 = x;
      y1 = y;
      return static_cast<float>(y);
    }
  };

  struct OnePoleLP
  {
    double a = 0.0, y1 = 0.0;
    void SetCutoff(const double sr, const double fc)
    {
      const double rc = 1.0 / (2.0 * 3.14159265358979323846 * fc);
      const double dt = 1.0 / sr;
      a = dt / (rc + dt);
    }
    void Reset() { y1 = 0.0; }
    inline float Process(const float x)
    {
      y1 += a * (x - y1);
      return static_cast<float>(y1);
    }
  };

  static inline float _Clip(const float x)
  {
    if (x >= 0.0f)
      return std::tanh(x * 0.7f) * 0.7f;
    else
      return std::tanh(x * 1.3f) * 0.85f;
  }

  double mSampleRate = 48000.0;
  double mGain = 0.0, mVolume = 1.0;
  double mGainLinear = 1.0, mOutputComp = 1.0;
  OnePoleHP mPreHP;
  OnePoleLP mPreLP, mPostLP;
};

} // namespace drive
} // namespace dsp
