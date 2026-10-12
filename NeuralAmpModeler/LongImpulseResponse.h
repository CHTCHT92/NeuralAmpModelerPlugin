//
//  LongImpulseResponse.h
//
//  Drop-in replacement for dsp::ImpulseResponse (AudioDSPTools) with a much more capable engine:
//  - IRs up to kMaxSeconds long (500 ms) instead of 8192 samples,
//  - zero latency: the first kBlock taps are convolved directly, the rest with a partitioned FFT,
//  - same gain staging and cubic resampling as the original, so existing IRs sound the same.
//
//  Header-only on purpose (no project file changes needed).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "../AudioDSPTools/dsp/Resample.h"
#include "../AudioDSPTools/dsp/wav.h"

namespace lir
{
// Radix-2 complex FFT working on separate re/im arrays. Inverse is NOT scaled.
class FFT
{
public:
  explicit FFT(const size_t n)
  : mN(n)
  , mCos(n / 2)
  , mSin(n / 2)
  , mRev(n)
  {
    const double twoPi = 6.283185307179586476925286766559;
    for (size_t k = 0; k < n / 2; k++)
    {
      mCos[k] = static_cast<float>(std::cos(twoPi * static_cast<double>(k) / static_cast<double>(n)));
      mSin[k] = static_cast<float>(std::sin(twoPi * static_cast<double>(k) / static_cast<double>(n)));
    }
    size_t bits = 0;
    while ((static_cast<size_t>(1) << bits) < n)
      bits++;
    for (size_t i = 0; i < n; i++)
    {
      size_t r = 0;
      for (size_t b = 0; b < bits; b++)
        if (i & (static_cast<size_t>(1) << b))
          r |= static_cast<size_t>(1) << (bits - 1 - b);
      mRev[i] = static_cast<uint32_t>(r);
    }
  }

  void Forward(float* re, float* im) const { _Transform(re, im, false); }
  void Inverse(float* re, float* im) const { _Transform(re, im, true); }

private:
  void _Transform(float* re, float* im, const bool inverse) const
  {
    for (size_t i = 0; i < mN; i++)
    {
      const size_t j = mRev[i];
      if (j > i)
      {
        std::swap(re[i], re[j]);
        std::swap(im[i], im[j]);
      }
    }
    const float sign = inverse ? 1.0f : -1.0f;
    for (size_t len = 2; len <= mN; len <<= 1)
    {
      const size_t half = len >> 1;
      const size_t step = mN / len;
      for (size_t i = 0; i < mN; i += len)
      {
        for (size_t k = 0; k < half; k++)
        {
          const float wr = mCos[k * step];
          const float wi = sign * mSin[k * step];
          const size_t a = i + k;
          const size_t b = a + half;
          const float tr = re[b] * wr - im[b] * wi;
          const float ti = re[b] * wi + im[b] * wr;
          re[b] = re[a] - tr;
          im[b] = im[a] - ti;
          re[a] += tr;
          im[a] += ti;
        }
      }
    }
  }

  size_t mN;
  std::vector<float> mCos, mSin;
  std::vector<uint32_t> mRev;
};

// Zero-latency convolution of a mono stream with a fixed impulse response.
//   taps [0, kBlock)   : direct (time domain) convolution
//   taps [kBlock, end) : uniform partitioned FFT convolution (overlap-add), whose output for block m+1
//                        only depends on input blocks <= m, hence no latency.
class PartitionedConvolver
{
public:
  static constexpr size_t kBlock = 256;

  explicit PartitionedConvolver(const std::vector<float>& ir)
  : mFFT(2 * kBlock)
  {
    const size_t B = kBlock;
    const size_t n = ir.size();
    const size_t bins = B + 1;

    // Direct part: reversed so that a dot product with the last B input samples is the convolution.
    mHead.assign(B, 0.0f);
    for (size_t k = 0; k < std::min(n, B); k++)
      mHead[B - 1 - k] = ir[k];
    mHist.assign(2 * B, 0.0f);

    // FFT part
    mP = n > B ? (n - B + B - 1) / B : 0;
    mHre.assign(mP * bins, 0.0f);
    mHim.assign(mP * bins, 0.0f);
    std::vector<float> re(2 * B), im(2 * B);
    for (size_t p = 0; p < mP; p++)
    {
      std::fill(re.begin(), re.end(), 0.0f);
      std::fill(im.begin(), im.end(), 0.0f);
      for (size_t j = 0; j < B; j++)
      {
        const size_t idx = B + p * B + j;
        if (idx < n)
          re[j] = ir[idx];
      }
      mFFT.Forward(re.data(), im.data());
      std::copy(re.begin(), re.begin() + bins, mHre.begin() + p * bins);
      std::copy(im.begin(), im.begin() + bins, mHim.begin() + p * bins);
    }
    mXre.assign(mP * bins, 0.0f);
    mXim.assign(mP * bins, 0.0f);
    mAccRe.assign(bins, 0.0f);
    mAccIm.assign(bins, 0.0f);
    mWorkRe.assign(2 * B, 0.0f);
    mWorkIm.assign(2 * B, 0.0f);
    mBlockIn.assign(B, 0.0f);
    mTailCur.assign(B, 0.0f);
    mOverlap.assign(B, 0.0f);
  }

  void Process(const float* in, float* out, const size_t numFrames)
  {
    const size_t B = kBlock;
    for (size_t i = 0; i < numFrames; i++)
    {
      const float x = in[i];
      mHist[mPos] = x;
      mHist[mPos + B] = x;
      mBlockIn[mPos] = x;
      out[i] = _Dot(&mHist[mPos + 1], mHead.data(), B) + mTailCur[mPos];
      if (++mPos == B)
      {
        mPos = 0;
        if (mP > 0)
          _ProcessBlock();
      }
    }
  }

private:
  static float _Dot(const float* a, const float* b, const size_t n)
  {
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
    size_t i = 0;
    for (; i + 4 <= n; i += 4)
    {
      s0 += a[i] * b[i];
      s1 += a[i + 1] * b[i + 1];
      s2 += a[i + 2] * b[i + 2];
      s3 += a[i + 3] * b[i + 3];
    }
    for (; i < n; i++)
      s0 += a[i] * b[i];
    return (s0 + s1) + (s2 + s3);
  }

  // Called each time kBlock new input samples have been collected.
  void _ProcessBlock()
  {
    const size_t B = kBlock;
    const size_t bins = B + 1;
    float* re = mWorkRe.data();
    float* im = mWorkIm.data();

    // Spectrum of the block that just completed, stored in the frequency-domain delay line.
    std::copy(mBlockIn.begin(), mBlockIn.end(), re);
    std::fill(re + B, re + 2 * B, 0.0f);
    std::fill(im, im + 2 * B, 0.0f);
    mFFT.Forward(re, im);
    const size_t slot = mFdlPos;
    std::copy(re, re + bins, mXre.begin() + slot * bins);
    std::copy(im, im + bins, mXim.begin() + slot * bins);

    // Multiply-accumulate against every partition of the tail.
    std::fill(mAccRe.begin(), mAccRe.end(), 0.0f);
    std::fill(mAccIm.begin(), mAccIm.end(), 0.0f);
    float* accRe = mAccRe.data();
    float* accIm = mAccIm.data();
    for (size_t p = 0; p < mP; p++)
    {
      const size_t s = (slot + mP - p) % mP;
      const float* xr = &mXre[s * bins];
      const float* xi = &mXim[s * bins];
      const float* hr = &mHre[p * bins];
      const float* hi = &mHim[p * bins];
      for (size_t k = 0; k < bins; k++)
      {
        accRe[k] += xr[k] * hr[k] - xi[k] * hi[k];
        accIm[k] += xr[k] * hi[k] + xi[k] * hr[k];
      }
    }
    mFdlPos = (slot + 1) % mP;

    // Back to time domain (Hermitian-symmetric spectrum, so the result is real).
    for (size_t k = 0; k < bins; k++)
    {
      re[k] = accRe[k];
      im[k] = accIm[k];
    }
    im[0] = 0.0f;
    im[B] = 0.0f;
    for (size_t k = 1; k < B; k++)
    {
      re[2 * B - k] = accRe[k];
      im[2 * B - k] = -accIm[k];
    }
    mFFT.Inverse(re, im);

    // Overlap-add: first half belongs to the next block, second half to the one after.
    const float scale = 1.0f / static_cast<float>(2 * B);
    for (size_t j = 0; j < B; j++)
    {
      mTailCur[j] = re[j] * scale + mOverlap[j];
      mOverlap[j] = re[B + j] * scale;
    }
  }

  FFT mFFT;
  size_t mP = 0; // number of FFT partitions
  size_t mPos = 0; // position in the current block
  size_t mFdlPos = 0; // next slot of the frequency-domain delay line
  std::vector<float> mHead, mHist;
  std::vector<float> mHre, mHim; // partition spectra, [mP][B+1]
  std::vector<float> mXre, mXim; // input spectra delay line, [mP][B+1]
  std::vector<float> mAccRe, mAccIm, mWorkRe, mWorkIm;
  std::vector<float> mBlockIn, mTailCur, mOverlap;
};
} // namespace lir

class LongImpulseResponse
{
public:
  struct IRData
  {
    std::vector<float> mRawAudio;
    double mRawAudioSampleRate = 48000.0;
  };

  // Longest IR that is used (longer ones are cut, with a short fade-out).
  static constexpr double kMaxSeconds = 0.5;

  LongImpulseResponse(const char* fileName, const double sampleRate)
  : mWavState(dsp::wav::LoadReturnCode::ERROR_OTHER)
  , mSampleRate(sampleRate)
  {
    mWavState = dsp::wav::Load(fileName, mRawAudio, mRawAudioSampleRate);
    if (mWavState == dsp::wav::LoadReturnCode::SUCCESS)
      _Build();
  }

  LongImpulseResponse(const IRData& irData, const double sampleRate)
  : mWavState(dsp::wav::LoadReturnCode::SUCCESS)
  , mRawAudio(irData.mRawAudio)
  , mRawAudioSampleRate(irData.mRawAudioSampleRate)
  , mSampleRate(sampleRate)
  {
    _Build();
  }

  double** Process(double** inputs, const size_t numChannels, const size_t numFrames)
  {
    _PrepareBuffers(numChannels, numFrames);
    if (numChannels == 0 || mConvolver == nullptr)
      return mPointers.data();

    for (size_t i = 0; i < numFrames; i++)
      mIn[i] = static_cast<float>(inputs[0][i]);
    mConvolver->Process(mIn.data(), mOut.data(), numFrames);
    for (size_t i = 0; i < numFrames; i++)
      mOutputs[0][i] = static_cast<double>(mOut[i]);
    // Copy out for more-than-mono.
    for (size_t c = 1; c < numChannels; c++)
      std::copy(mOutputs[0].begin(), mOutputs[0].begin() + numFrames, mOutputs[c].begin());
    return mPointers.data();
  }

  IRData GetData() const
  {
    IRData irData;
    irData.mRawAudio = mRawAudio;
    irData.mRawAudioSampleRate = mRawAudioSampleRate;
    return irData;
  }
  double GetSampleRate() const { return mSampleRate; }
  dsp::wav::LoadReturnCode GetWavState() const { return mWavState; }

private:
  void _Build()
  {
    const double sampleRate = mSampleRate > 0.0 ? mSampleRate : 48000.0;

    std::vector<float> resampled;
    if (mRawAudioSampleRate == sampleRate)
    {
      resampled = mRawAudio;
    }
    else
    {
      // Cubic resampling, same as the original engine
      std::vector<float> padded(mRawAudio.size() + 2, 0.0f);
      std::copy(mRawAudio.begin(), mRawAudio.end(), padded.begin() + 1);
      dsp::ResampleCubic<float>(padded, mRawAudioSampleRate, sampleRate, 0.0, resampled);
    }

    const size_t maxLength = static_cast<size_t>(std::ceil(kMaxSeconds * sampleRate));
    const size_t length = std::min(resampled.size(), maxLength);
    // Same gain staging as the original engine:
    // https://github.com/sdatkinson/NeuralAmpModelerPlugin/issues/100#issuecomment-1455273839
    const float gain = static_cast<float>(std::pow(10.0, -18.0 * 0.05) * 48000.0 / sampleRate);
    std::vector<float> weights(length);
    for (size_t i = 0; i < length; i++)
      weights[i] = gain * resampled[i];

    // If the IR had to be cut, fade out the last 5 ms so that the cut doesn't click.
    if (resampled.size() > maxLength)
    {
      const size_t fade = std::min(length, static_cast<size_t>(std::lround(0.005 * sampleRate)));
      for (size_t i = 0; i < fade; i++)
      {
        const double g = 0.5 * (1.0 + std::cos(3.14159265358979323846 * static_cast<double>(i + 1) / static_cast<double>(fade)));
        weights[length - fade + i] *= static_cast<float>(g);
      }
    }

    mConvolver = std::make_unique<lir::PartitionedConvolver>(weights);
  }

  void _PrepareBuffers(const size_t numChannels, const size_t numFrames)
  {
    if (mOutputs.size() < numChannels)
      mOutputs.resize(numChannels);
    for (auto& channel : mOutputs)
      if (channel.size() < numFrames)
        channel.resize(numFrames);
    if (mIn.size() < numFrames)
    {
      mIn.resize(numFrames);
      mOut.resize(numFrames);
    }
    mPointers.resize(std::max<size_t>(numChannels, 1));
    for (size_t c = 0; c < numChannels; c++)
      mPointers[c] = mOutputs[c].data();
  }

  dsp::wav::LoadReturnCode mWavState;
  // Keep a copy of the raw audio so that it can be resampled when the sample rate changes.
  std::vector<float> mRawAudio;
  double mRawAudioSampleRate = 48000.0;
  double mSampleRate;

  std::unique_ptr<lir::PartitionedConvolver> mConvolver;
  std::vector<float> mIn, mOut;
  std::vector<std::vector<double>> mOutputs;
  std::vector<double*> mPointers;
};
