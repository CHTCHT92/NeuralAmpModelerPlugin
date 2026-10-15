#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace dsp
{
namespace delay
{

// Stereo analog-ish delay (BBD/tape flavor): LP in the feedback loop + soft saturation.
// Time: 80..800 ms. Feedback: 0..10. Mix: 0..1.
class AnalogDelay
{
public:
  static constexpr double kMinTimeMs = 80.0;
  static constexpr double kMaxTimeMs = 800.0;

  void Reset(const double sampleRate)
  {
    mSampleRate = sampleRate > 0.0 ? sampleRate : 48000.0;
    const size_t maxSamples = static_cast<size_t>(std::ceil((kMaxTimeMs / 1000.0) * mSampleRate * 1.05)) + 4;
    for (int c = 0; c < 2; c++)
    {
      mBuffer[c].assign(maxSamples, 0.0f);
      mWritePos[c] = 0;
      mFeedbackLP[c].SetCutoff(mSampleRate, 4000.0);
      mFeedbackLP[c].Reset();
    }
    mDelaySamples[0] = mDelaySamples[1] = 0.0;
  }

  void SetParams(const double timeMs, const double feedback010, const double mix)
  {
    const double clampedMs = std::clamp(timeMs, kMinTimeMs, kMaxTimeMs);
    mTimeMs = clampedMs;
    mFeedback = std::clamp(feedback010 / 10.0, 0.0, 1.0) * 0.95;
    mMix = std::clamp(mix, 0.0, 1.0);
    // Right channel 2% longer -> wide stereo image.
    mDelaySamples[0] = (clampedMs / 1000.0) * mSampleRate;
    mDelaySamples[1] = (clampedMs / 1000.0 * 1.02) * mSampleRate;
  }

  void Process(iplug::sample** in, iplug::sample** out, const size_t numFrames)
  {
    const float mix = static_cast<float>(mMix);
    const float dry = 1.0f - mix;
    const float fb = static_cast<float>(mFeedback);

    for (size_t i = 0; i < numFrames; i++)
    {
      for (int c = 0; c < 2; c++)
      {
        const float x = in[c][i];
        const float delayed = _ReadInterpolated(c, mDelaySamples[c]);
        const float filtered = mFeedbackLP[c].Process(delayed);
        const float saturated = std::tanh(filtered * 0.9f);
        mBuffer[c][mWritePos[c]] = x + saturated * fb;
        mWritePos[c] = (mWritePos[c] + 1) % mBuffer[c].size();
        out[c][i] = dry * x + mix * delayed;
      }
    }
  }

private:
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

  inline float _ReadInterpolated(const int c, const double delaySamples) const
  {
    const size_t size = mBuffer[c].size();
    const double readPos = static_cast<double>(mWritePos[c]) - delaySamples;
    double rp = std::fmod(readPos, static_cast<double>(size));
    if (rp < 0.0)
      rp += static_cast<double>(size);
    const size_t i0 = static_cast<size_t>(rp);
    const size_t i1 = (i0 + 1) % size;
    const float frac = static_cast<float>(rp - static_cast<double>(i0));
    return mBuffer[c][i0] * (1.0f - frac) + mBuffer[c][i1] * frac;
  }

  double mSampleRate = 48000.0;
  double mTimeMs = 300.0;
  double mFeedback = 0.0;
  double mMix = 0.3;
  double mDelaySamples[2] = {0.0, 0.0};

  std::vector<float> mBuffer[2];
  size_t mWritePos[2] = {0, 0};
  OnePoleLP mFeedbackLP[2];
};

} // namespace delay
} // namespace dsp
