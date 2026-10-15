#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace dsp
{
namespace reverb
{

enum class Type { Room = 0, Plate = 1, Spring = 2 };

// ============================================================================
// Freeverb (Room / Plate)
// ============================================================================
class Freeverb
{
public:
  void Reset(const double sampleRate, const float damping, const float roomSize)
  {
    mSampleRate = sampleRate > 0.0 ? sampleRate : 48000.0;
    const double srScale = mSampleRate / 44100.0;
    static const int combTuning[8] = {1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617};
    static const int allpassTuning[4] = {556, 441, 341, 225};
    static const int stereoSpread = 23;

    for (int c = 0; c < 2; c++)
    {
      const int spread = c * stereoSpread;
      for (int i = 0; i < 8; i++)
      {
        mComb[c][i].Init(static_cast<int>(std::lround((combTuning[i] + spread) * srScale)));
        mComb[c][i].SetDamp(damping);
        mComb[c][i].SetFeedback(roomSize);
      }
      for (int i = 0; i < 4; i++)
        mAllpass[c][i].Init(static_cast<int>(std::lround((allpassTuning[i] + spread) * srScale)));
    }
    mInputGain = 0.015f;
  }

  void SetRoomSize(const float roomSize)
  {
    mRoomSize = std::min(roomSize, 0.98f);
    for (int c = 0; c < 2; c++)
      for (int j = 0; j < 8; j++)
        mComb[c][j].SetFeedback(mRoomSize);
  }

  void SetDamping(const float damping)
  {
    mDamping = damping;
    for (int c = 0; c < 2; c++)
      for (int j = 0; j < 8; j++)
        mComb[c][j].SetDamp(mDamping);
  }

  void Process(iplug::sample** in, iplug::sample** out, const size_t numFrames)
  {
    for (size_t i = 0; i < numFrames; i++)
    {
      for (int c = 0; c < 2; c++)
      {
        const float x = in[c][i];
        float wet = 0.0f;
        for (int j = 0; j < 8; j++)
          wet += mComb[c][j].Process(x * mInputGain);
        for (int j = 0; j < 4; j++)
          wet = mAllpass[c][j].Process(wet);
        out[c][i] = wet;
      }
    }
  }

private:
  struct Comb
  {
    std::vector<float> buffer;
    int pos = 0;
    float filterStore = 0.0f;
    float damp = 0.2f;
    float feedback = 0.84f;
    void Init(const int size)
    {
      buffer.assign(std::max(size, 1), 0.0f);
      pos = 0;
      filterStore = 0.0f;
    }
    void SetDamp(const float d) { damp = d; }
    void SetFeedback(const float f) { feedback = f; }
    inline float Process(const float x)
    {
      const float y = buffer[pos];
      filterStore = y * (1.0f - damp) + filterStore * damp;
      buffer[pos] = x + filterStore * feedback;
      pos = (pos + 1) % static_cast<int>(buffer.size());
      return y;
    }
  };

  struct Allpass
  {
    std::vector<float> buffer;
    int pos = 0;
    float feedback = 0.5f;
    void Init(const int size)
    {
      buffer.assign(std::max(size, 1), 0.0f);
      pos = 0;
    }
    inline float Process(const float x)
    {
      const float y = buffer[pos];
      const float out = -x + y;
      buffer[pos] = x + y * feedback;
      pos = (pos + 1) % static_cast<int>(buffer.size());
      return out;
    }
  };

  double mSampleRate = 48000.0;
  float mInputGain = 0.015f;
  float mRoomSize = 0.84f;
  float mDamping = 0.2f;
  Comb mComb[2][8];
  Allpass mAllpass[2][4];
};

// ============================================================================
// Spring reverb — true model: dispersive allpass chain + resonant tank.
// ============================================================================
class SpringReverb
{
public:
  void Reset(const double sampleRate, const float decay, const float tension)
  {
    mSampleRate = sampleRate > 0.0 ? sampleRate : 48000.0;
    static const int apLengths[8] = {89, 97, 113, 127, 149, 163, 181, 211};
    for (int c = 0; c < 2; c++)
    {
      for (int i = 0; i < 8; i++)
        mAllpass[c][i].Init(apLengths[i], 0.6f);
      const int delaySamples = static_cast<int>(std::lround(0.050 * mSampleRate));
      mTank[c].Init(delaySamples);
      mTank[c].SetFeedback(decay);
      mLP[c].SetCutoff(mSampleRate, 3500.0);
      mLP[c].Reset();
    }
    mTension = tension;
  }

  void SetDecay(const float decay)
  {
    mDecay = std::min(decay, 0.95f);
    for (int c = 0; c < 2; c++)
      mTank[c].SetFeedback(mDecay);
  }

  void SetTension(const float tension) { mTension = tension; }

  void Process(iplug::sample** in, iplug::sample** out, const size_t numFrames)
  {
    for (size_t i = 0; i < numFrames; i++)
    {
      for (int c = 0; c < 2; c++)
      {
        float x = in[c][i];
        for (int j = 0; j < 8; j++)
          x = mAllpass[c][j].Process(x, mTension);
        const float delayed = mTank[c].Read();
        const float filtered = mLP[c].Process(delayed);
        mTank[c].Write(x + filtered * mDecay);
        out[c][i] = delayed;
      }
    }
  }

private:
  struct DispAllpass
  {
    std::vector<float> buffer;
    int pos = 0;
    float baseFeedback = 0.6f;
    void Init(const int size, const float fb)
    {
      buffer.assign(std::max(size, 1), 0.0f);
      pos = 0;
      baseFeedback = fb;
    }
    inline float Process(const float x, const float tension)
    {
      const float y = buffer[pos];
      const float fb = std::min(baseFeedback + tension * 0.3f, 0.85f);
      const float out = -x + y;
      buffer[pos] = x + y * fb;
      pos = (pos + 1) % static_cast<int>(buffer.size());
      return out;
    }
  };

  struct FeedbackDelay
  {
    std::vector<float> buffer;
    int pos = 0;
    float feedback = 0.7f;
    void Init(const int size)
    {
      buffer.assign(std::max(size, 1), 0.0f);
      pos = 0;
    }
    void SetFeedback(const float f) { feedback = f; }
    inline float Read() const { return buffer[pos]; }
    inline void Write(const float x)
    {
      buffer[pos] = x;
      pos = (pos + 1) % static_cast<int>(buffer.size());
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

  double mSampleRate = 48000.0;
  float mDecay = 0.7f;
  float mTension = 0.5f;
  DispAllpass mAllpass[2][8];
  FeedbackDelay mTank[2];
  OnePoleLP mLP[2];
};

// ============================================================================
// Wrapper: picks Room / Plate / Spring.
// Time 0..2 s -> mapped to roomSize / decay. Time=0 -> ~50 ms tail.
// ============================================================================
class Reverb3
{
public:
  void Reset(const double sampleRate)
  {
    mSampleRate = sampleRate > 0.0 ? sampleRate : 48000.0;
    mFreeverb.Reset(mSampleRate, 0.2f, 0.84f);
    mSpring.Reset(mSampleRate, 0.7f, 0.5f);
    _ApplyTypeAndTime();
  }

  void SetParams(const double timeSec, const double mix, const Type type)
  {
    const double clampedTime = std::clamp(timeSec, 0.0, 2.0);
    const bool timeChanged = (std::abs(clampedTime - mTimeSec) > 1e-6);
    const bool typeChanged = (type != mType);
    mTimeSec = clampedTime;
    mMix = std::clamp(mix, 0.0, 1.0);
    mType = type;
    if (timeChanged || typeChanged)
      _ApplyTypeAndTime();
  }

  void Process(iplug::sample** in, iplug::sample** out, const size_t numFrames)
  {
    const float mix = static_cast<float>(mMix);
    const float dry = 1.0f - mix;

    _EnsureWetBuffers(numFrames);
    iplug::sample* wetPtrs[2] = {mWet[0].data(), mWet[1].data()};

    if (mType == Type::Spring)
      mSpring.Process(in, wetPtrs, numFrames);
    else
      mFreeverb.Process(in, wetPtrs, numFrames);

    for (int c = 0; c < 2; c++)
      for (size_t i = 0; i < numFrames; i++)
        out[c][i] = dry * in[c][i] + mix * wetPtrs[c][i] * mWetScale;
  }

private:
  void _EnsureWetBuffers(const size_t numFrames)
  {
    for (int c = 0; c < 2; c++)
      if (mWet[c].size() < numFrames)
        mWet[c].resize(numFrames, 0.0f);
  }

  void _ApplyTypeAndTime()
  {
    const double t = mTimeSec / 2.0;

    switch (mType)
    {
      case Type::Room:
      {
        const float roomSize = static_cast<float>(0.55 + t * 0.40);
        const float damping = 0.30f;
        mFreeverb.SetRoomSize(roomSize);
        mFreeverb.SetDamping(damping);
        mWetScale = 1.0f;
        break;
      }
      case Type::Plate:
      {
        const float roomSize = static_cast<float>(0.70 + t * 0.27);
        const float damping = 0.10f;
        mFreeverb.SetRoomSize(roomSize);
        mFreeverb.SetDamping(damping);
        mWetScale = 0.9f;
        break;
      }
      case Type::Spring:
      {
        const float decay = static_cast<float>(0.5 + t * 0.45);
        const float tension = 0.5f;
        mSpring.SetDecay(decay);
        mSpring.SetTension(tension);
        mWetScale = 1.1f;
        break;
      }
    }
  }

  double mSampleRate = 48000.0;
  double mTimeSec = 1.0;
  double mMix = 0.3;
  Type mType = Type::Room;
  float mWetScale = 1.0f;

  Freeverb mFreeverb;
  SpringReverb mSpring;
  std::vector<float> mWet[2];
};

} // namespace reverb
} // namespace dsp
