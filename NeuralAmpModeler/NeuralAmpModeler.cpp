#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <system_error>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <utility>

#include "Colors.h"
#include "../NeuralAmpModelerCore/NAM/activations.h"
#include "../NeuralAmpModelerCore/NAM/get_dsp.h"
// clang-format off
#include "NeuralAmpModeler.h"
#include "IPlug_include_in_plug_src.h"
// clang-format on
#include "architecture.hpp"

#include "NeuralAmpModelerControls.h"

using namespace iplug;
using namespace igraphics;

const double kDCBlockerFrequency = 5.0;

// Styles
const IVColorSpec colorSpec{
  DEFAULT_BGCOLOR,
  PluginColors::NAM_THEMECOLOR,
  PluginColors::NAM_THEMECOLOR.WithOpacity(0.3f),
  PluginColors::NAM_THEMECOLOR.WithOpacity(0.4f),
  PluginColors::MOUSEOVER,
  DEFAULT_SHCOLOR,
  PluginColors::NAM_THEMECOLOR,
  COLOR_RED,
  PluginColors::NAM_THEMECOLOR.WithContrast(0.1f),
};

const IVStyle style =
  IVStyle{true, true, colorSpec,
          {DEFAULT_TEXT_SIZE + 3.f, EVAlign::Middle, PluginColors::NAM_THEMEFONTCOLOR},
          {DEFAULT_TEXT_SIZE + 3.f, EVAlign::Bottom, PluginColors::NAM_THEMEFONTCOLOR},
          DEFAULT_HIDE_CURSOR, DEFAULT_DRAW_FRAME, false, DEFAULT_EMBOSS, 0.2f, 2.f,
          DEFAULT_SHADOW_OFFSET, DEFAULT_WIDGET_FRAC, DEFAULT_WIDGET_ANGLE};
const IVStyle titleStyle =
  DEFAULT_STYLE.WithValueText(IText(30, COLOR_WHITE, "Michroma-Regular")).WithDrawFrame(false).WithShadowOffset(2.f);
const IVStyle radioButtonStyle =
  style
    .WithColor(EVColor::kON, PluginColors::NAM_THEMECOLOR)
    .WithColor(EVColor::kOFF, PluginColors::NAM_THEMECOLOR.WithOpacity(0.1f))
    .WithColor(EVColor::kX1, PluginColors::NAM_THEMECOLOR.WithOpacity(0.6f));

EMsgBoxResult _ShowMessageBox(iplug::igraphics::IGraphics* pGraphics, const char* str, const char* caption,
                              EMsgBoxType type)
{
#ifdef OS_MAC
  return pGraphics->ShowMessageBox(caption, str, type);
#else
  return pGraphics->ShowMessageBox(str, caption, type);
#endif
}

const std::string kCalibrateInputParamName = "CalibrateInput";
const bool kDefaultCalibrateInput = false;
const std::string kInputCalibrationLevelParamName = "InputCalibrationLevel";
const double kDefaultInputCalibrationLevel = 12.0;


NeuralAmpModeler::NeuralAmpModeler(const InstanceInfo& info)
: Plugin(info, MakeConfig(kNumParams, kNumPresets))
{
  _InitToneStack();
  nam::activations::Activation::enable_fast_tanh();
  GetParam(kInputLevel)->InitGain("Input", 0.0, -20.0, 20.0, 0.1);
  GetParam(kToneBass)->InitDouble("Bass", 5.0, 0.0, 10.0, 0.1);
  GetParam(kToneMid)->InitDouble("Middle", 5.0, 0.0, 10.0, 0.1);
  GetParam(kToneTreble)->InitDouble("Treble", 5.0, 0.0, 10.0, 0.1);
  GetParam(kOutputLevel)->InitGain("Output", 0.0, -40.0, 40.0, 0.1);
  GetParam(kNoiseGateThreshold)->InitGain("Threshold", -80.0, -100.0, 0.0, 0.1);
  GetParam(kNoiseGateActive)->InitBool("NoiseGateActive", true);
  GetParam(kEQActive)->InitBool("ToneStack", true);
  GetParam(kOutputMode)->InitEnum("OutputMode", 1, {"Raw", "Normalized", "Calibrated"});
  GetParam(kIRToggle)->InitBool("IRToggle", true);
  GetParam(kCalibrateInput)->InitBool(kCalibrateInputParamName.c_str(), kDefaultCalibrateInput);
  GetParam(kInputCalibrationLevel)
    ->InitDouble(kInputCalibrationLevelParamName.c_str(), kDefaultInputCalibrationLevel, -60.0, 60.0, 0.1, "dBu");
  GetParam(kSlim)->InitDouble("Slim", 0.0, 0.0, 1.0, 0.01);

  // ===== NOUVEAUX PARAMS : DRIVE / DELAY / REVERB =====
  GetParam(kDriveActive)->InitBool("DriveActive", false);
  GetParam(kDriveGain)->InitDouble("Drive", 0.0, 0.0, 1.0, 0.01);
  GetParam(kDriveVolume)->InitDouble("Drive Volume", 1.0, 0.0, 2.0, 0.01);

  GetParam(kDelayActive)->InitBool("DelayActive", false);
  GetParam(kDelayTime)->InitDouble("Delay Time", 300.0, 80.0, 800.0, 1.0, "ms");
  GetParam(kDelayTime)->SetDisplayFunc([](double value, WDL_String& str) {
    str.SetFormatted(64, "%.0f ms", value);
  });
  GetParam(kDelayFeedback)->InitDouble("Delay Feedback", 3.0, 0.0, 10.0, 0.1);
  GetParam(kDelayMix)->InitDouble("Delay Mix", 0.3, 0.0, 1.0, 0.01);

  GetParam(kReverbActive)->InitBool("ReverbActive", false);
  GetParam(kReverbTime)->InitDouble("Reverb Time", 1.0, 0.0, 2.0, 0.01, "s");
  GetParam(kReverbType)->InitEnum("Reverb Type", 0, {"Room", "Plate", "Spring"});
  GetParam(kReverbMix)->InitDouble("Reverb Mix", 0.3, 0.0, 1.0, 0.01);
  // ===================================================

  GetParam(kModelIndex)->InitEnum("Model", 0, kMaxModels);
  GetParam(kModelIndex)->SetDisplayFunc([this](double value, WDL_String& str) {
    std::lock_guard<std::mutex> lock(mModelListMutex);
    if (mModelNames.empty())
    {
      str.Set("-");
      return;
    }
    const int last = static_cast<int>(mModelNames.size()) - 1;
    const int idx = std::clamp(static_cast<int>(std::lround(value)), 0, last);
    str.Set(mModelNames[static_cast<size_t>(idx)].c_str());
  });
  GetParam(kIRIndex)->InitEnum("Cab", 0, kMaxIRs);
  GetParam(kIRIndex)->SetDisplayFunc([this](double value, WDL_String& str) {
    std::lock_guard<std::mutex> lock(mIRListMutex);
    if (mIRNames.empty())
    {
      str.Set("-");
      return;
    }
    const int last = static_cast<int>(mIRNames.size()) - 1;
    const int idx = std::clamp(static_cast<int>(std::lround(value)), 0, last);
    str.Set(mIRNames[static_cast<size_t>(idx)].c_str());
  });
  GetParam(kBankIndex)->InitEnum("Bank", 0, kMaxBanks);
  GetParam(kBankIndex)->SetDisplayFunc([this](double value, WDL_String& str) {
    std::lock_guard<std::mutex> lock(mModelListMutex);
    if (mBankNames.empty())
    {
      str.Set("-");
      return;
    }
    const int last = static_cast<int>(mBankNames.size()) - 1;
    const int idx = std::clamp(static_cast<int>(std::lround(value)), 0, last);
    str.Set(mBankNames[static_cast<size_t>(idx)].c_str());
  });
  GetParam(kIRBankIndex)->InitEnum("Cab Bank", 0, kMaxBanks);
  GetParam(kIRBankIndex)->SetDisplayFunc([this](double value, WDL_String& str) {
    std::lock_guard<std::mutex> lock(mIRListMutex);
    if (mIRBankNames.empty())
    {
      str.Set("-");
      return;
    }
    const int last = static_cast<int>(mIRBankNames.size()) - 1;
    const int idx = std::clamp(static_cast<int>(std::lround(value)), 0, last);
    str.Set(mIRBankNames[static_cast<size_t>(idx)].c_str());
  });

  mNoiseGateTrigger.AddListener(&mNoiseGateGain);

  mMakeGraphicsFunc = [&]() {
#ifdef OS_IOS
    auto scaleFactor = GetScaleForScreen(PLUG_WIDTH, PLUG_HEIGHT) * 0.85f;
#else
    auto scaleFactor = 1.0f;
#endif
    return MakeGraphics(*this, PLUG_WIDTH, PLUG_HEIGHT, PLUG_FPS, scaleFactor);
  };
  mLayoutFunc = [&](IGraphics* pGraphics) {
    pGraphics->AttachCornerResizer(EUIResizerMode::Scale, false);
    pGraphics->AttachTextEntryControl();
    pGraphics->EnableMouseOver(true);
    pGraphics->EnableTooltips(true);
    pGraphics->EnableMultiTouch(true);

    pGraphics->LoadFont("Roboto-Regular", ROBOTO_FN);
    pGraphics->LoadFont("Michroma-Regular", MICHROMA_FN);

    const auto gearSVG = pGraphics->LoadSVG(GEAR_FN);
    const auto fileSVG = pGraphics->LoadSVG(FILE_FN);
    const auto globeSVG = pGraphics->LoadSVG(GLOBE_ICON_FN);
    const auto crossSVG = pGraphics->LoadSVG(CLOSE_BUTTON_FN);
    const auto rightArrowSVG = pGraphics->LoadSVG(RIGHT_ARROW_FN);
    const auto leftArrowSVG = pGraphics->LoadSVG(LEFT_ARROW_FN);
    const auto modelIconSVG = pGraphics->LoadSVG(MODEL_ICON_FN);
    const auto irIconOnSVG = pGraphics->LoadSVG(IR_ICON_ON_FN);
    const auto irIconOffSVG = pGraphics->LoadSVG(IR_ICON_OFF_FN);
    const auto slimIconSVG = pGraphics->LoadSVG(SLIMMABLE_ICON_FN);

    const auto backgroundBitmap = pGraphics->LoadBitmap(BACKGROUND_FN);
    const auto fileBackgroundBitmap = pGraphics->LoadBitmap(FILEBACKGROUND_FN);
    const auto inputLevelBackgroundBitmap = pGraphics->LoadBitmap(INPUTLEVELBACKGROUND_FN);
    const auto linesBitmap = pGraphics->LoadBitmap(LINES_FN);
    const auto knobBackgroundBitmap = pGraphics->LoadBitmap(KNOBBACKGROUND_FN);
    const auto switchHandleBitmap = pGraphics->LoadBitmap(SLIDESWITCHHANDLE_FN);
    const auto meterBackgroundBitmap = pGraphics->LoadBitmap(METERBACKGROUND_FN);

    const auto b = pGraphics->GetBounds();
    const auto mainArea = b.GetPadded(-20);
    const auto contentArea = mainArea.GetPadded(-10);
    const auto titleHeight = 50.0f;
    const auto titleArea = contentArea.GetFromTop(titleHeight);

    // Areas for knobs
    const auto knobsPad = 20.0f;
    const auto knobsExtraSpaceBelowTitle = 25.0f;
    const auto singleKnobPad = -2.0f;
    const auto knobsArea = contentArea.GetFromTop(NAM_KNOB_HEIGHT)
                             .GetReducedFromLeft(knobsPad)
                             .GetReducedFromRight(knobsPad)
                             .GetVShifted(titleHeight + knobsExtraSpaceBelowTitle);
    const auto inputKnobArea = knobsArea.GetGridCell(0, kInputLevel, 1, numKnobs).GetPadded(-singleKnobPad);
    const auto noiseGateArea = knobsArea.GetGridCell(0, kNoiseGateThreshold, 1, numKnobs).GetPadded(-singleKnobPad);
    const auto bassKnobArea = knobsArea.GetGridCell(0, kToneBass, 1, numKnobs).GetPadded(-singleKnobPad);
    const auto midKnobArea = knobsArea.GetGridCell(0, kToneMid, 1, numKnobs).GetPadded(-singleKnobPad);
    const auto trebleKnobArea = knobsArea.GetGridCell(0, kToneTreble, 1, numKnobs).GetPadded(-singleKnobPad);
    const auto outputKnobArea = knobsArea.GetGridCell(0, kOutputLevel, 1, numKnobs).GetPadded(-singleKnobPad);

    const auto ngToggleArea =
      noiseGateArea.GetVShifted(noiseGateArea.H()).SubRectVertical(2, 0).GetReducedFromTop(10.0f);
    const auto eqToggleArea = midKnobArea.GetVShifted(midKnobArea.H()).SubRectVertical(2, 0).GetReducedFromTop(10.0f);

    // Areas for model and IR
    const auto fileWidth = 200.0f;
    const auto fileHeight = 30.0f;
    const auto irYOffset = 38.0f;
    const auto modelArea =
      contentArea.GetFromBottom((2.0f * fileHeight)).GetFromTop(fileHeight).GetMidHPadded(fileWidth).GetVShifted(-1);
    const auto slimIconArea =
      IRECT(modelArea.R + 6.f, modelArea.MH() - 14.f, modelArea.R + 6.f + 2.f * 28.f, modelArea.MH() + 14.f);
    const auto modelIconArea = modelArea.GetFromLeft(30).GetTranslated(-40, 10);
    const auto irArea = modelArea.GetVShifted(irYOffset);
    const auto irSwitchArea = irArea.GetFromLeft(30.0f).GetHShifted(-40.0f).GetScaledAboutCentre(0.6f);

    // Areas for meters
    const auto inputMeterArea = contentArea.GetFromLeft(30).GetHShifted(-20).GetMidVPadded(100).GetVShifted(-25);
    const auto outputMeterArea = contentArea.GetFromRight(30).GetHShifted(20).GetMidVPadded(100).GetVShifted(-25);

    // Misc Areas
    const auto settingsButtonArea = CornerButtonArea(b);

    // ===== ZONE FX : 2ème rangée de knobs + ligne de switches =====
    const auto fxKnobsArea = knobsArea.GetVShifted(NAM_KNOB_HEIGHT + 25.f);
    const auto fxKnobPad = -2.0f;
    const auto driveGainArea = fxKnobsArea.GetGridCell(0, 0, 1, 7).GetPadded(fxKnobPad);
    const auto driveVolumeArea = fxKnobsArea.GetGridCell(0, 1, 1, 7).GetPadded(fxKnobPad);
    const auto delayTimeArea = fxKnobsArea.GetGridCell(0, 2, 1, 7).GetPadded(fxKnobPad);
    const auto delayFeedbackArea = fxKnobsArea.GetGridCell(0, 3, 1, 7).GetPadded(fxKnobPad);
    const auto delayMixArea = fxKnobsArea.GetGridCell(0, 4, 1, 7).GetPadded(fxKnobPad);
    const auto reverbTimeArea = fxKnobsArea.GetGridCell(0, 5, 1, 7).GetPadded(fxKnobPad);
    const auto reverbMixArea = fxKnobsArea.GetGridCell(0, 6, 1, 7).GetPadded(fxKnobPad);

    // Ligne de switches pour les bypass FX (au-dessus de la 2ème rangée)
    const auto fxSwitchesArea = fxKnobsArea.GetVShifted(-40.f).GetFromTop(30.f);
    const auto driveSwitchArea = fxSwitchesArea.GetGridCell(0, 0, 1, 6).GetPadded(-4.f);
    const auto delaySwitchArea = fxSwitchesArea.GetGridCell(0, 1, 1, 6).GetPadded(-4.f);
    const auto reverbSwitchArea = fxSwitchesArea.GetGridCell(0, 2, 1, 6).GetPadded(-4.f);
    const auto reverbTypeArea = fxSwitchesArea.GetGridCell(0, 3, 1, 6);
    // =============================================================

    // Model loader button
    auto loadModelCompletionHandler = [&](const WDL_String& fileName, const WDL_String& path) {
      if (fileName.GetLength())
      {
        const std::string msg = _StageModel(fileName);
        if (msg.empty())
        {
          _SetModelFolderFromFile(fileName, true);
        }
        if (msg.size())
        {
          std::stringstream ss;
          ss << "Failed to load NAM model. Message:\n\n" << msg;
          _ShowMessageBox(GetUI(), ss.str().c_str(), "Failed to load model!", kMB_OK);
        }
        std::cout << "Loaded: " << fileName.Get() << std::endl;
      }
    };

    // IR loader button
    auto loadIRCompletionHandler = [&](const WDL_String& fileName, const WDL_String& path) {
      if (fileName.GetLength())
      {
        mIRPath = fileName;
        const dsp::wav::LoadReturnCode retCode = _StageIR(fileName);
        if (retCode != dsp::wav::LoadReturnCode::SUCCESS)
        {
          std::stringstream message;
          message << "Failed to load IR file " << fileName.Get() << ":\n";
          message << dsp::wav::GetMsgForLoadReturnCode(retCode);
          _ShowMessageBox(GetUI(), message.str().c_str(), "Failed to load IR!", kMB_OK);
        }
        else
        {
          _SetIRFolderFromFile(fileName, true);
        }
      }
    };

    pGraphics->AttachBackground(BACKGROUND_FN);
    pGraphics->AttachControl(new IBitmapControl(b, linesBitmap));
    pGraphics->AttachControl(new IVLabelControl(titleArea, "NEURAL AMP MODELER", titleStyle));
    pGraphics->AttachControl(new ISVGControl(modelIconArea, modelIconSVG));

#ifdef NAM_PICK_DIRECTORY
    const std::string defaultNamFileString = "Select model directory...";
    const std::string defaultIRString = "Select IR directory...";
#else
    const std::string defaultNamFileString = "Select model...";
    const std::string defaultIRString = "Select IR...";
#endif
    const char* const getUrl = "https://www.neuralampmodeler.com/users#comp-marb84o5";
    pGraphics->AttachControl(
      new NAMFileBrowserControl(modelArea, kMsgTagClearModel, defaultNamFileString.c_str(), "nam",
                                loadModelCompletionHandler, style, fileSVG, crossSVG, leftArrowSVG, rightArrowSVG,
                                fileBackgroundBitmap, globeSVG, "Get NAM Models", getUrl),
      kCtrlTagModelFileBrowser);

    auto hideSlimOverlay = [](IControl* pCaller) {
      IGraphics* ui = pCaller->GetUI();
      if (auto* backdrop = ui->GetControlWithTag(kCtrlTagSlimOverlayBackdrop))
        backdrop->Hide(true);
      if (auto* knob = ui->GetControlWithTag(kCtrlTagSlimKnob))
        knob->Hide(true);
      ui->SetAllControlsDirty();
    };
    auto showSlimOverlay = [](IControl* pCaller) {
      IGraphics* ui = pCaller->GetUI();
      if (auto* backdrop = ui->GetControlWithTag(kCtrlTagSlimOverlayBackdrop))
        backdrop->Hide(false);
      if (auto* knob = ui->GetControlWithTag(kCtrlTagSlimKnob))
        knob->Hide(false);
      ui->SetAllControlsDirty();
    };

    pGraphics
      ->AttachControl(
        new NAMSquareButtonControl(slimIconArea, DefaultClickActionFunc, slimIconSVG), kCtrlTagSlimmableIcon)
      ->SetAnimationEndActionFunction(showSlimOverlay)
      ->Hide(true);

    pGraphics->AttachControl(new ISVGSwitchControl(irSwitchArea, {irIconOffSVG, irIconOnSVG}, kIRToggle));
    pGraphics->AttachControl(
      new NAMFileBrowserControl(irArea, kMsgTagClearIR, defaultIRString.c_str(), "wav", loadIRCompletionHandler, style,
                                fileSVG, crossSVG, leftArrowSVG, rightArrowSVG, fileBackgroundBitmap, globeSVG,
                                "Get IRs", getUrl),
      kCtrlTagIRFileBrowser);
    pGraphics->AttachControl(
      new NAMSwitchControl(ngToggleArea, kNoiseGateActive, "Noise Gate", style, switchHandleBitmap));
    pGraphics->AttachControl(new NAMSwitchControl(eqToggleArea, kEQActive, "EQ", style, switchHandleBitmap));

    // The knobs (rangée 1)
    pGraphics->AttachControl(new NAMKnobControl(inputKnobArea, kInputLevel, "", style, knobBackgroundBitmap));
    pGraphics->AttachControl(new NAMKnobControl(noiseGateArea, kNoiseGateThreshold, "", style, knobBackgroundBitmap));
    pGraphics->AttachControl(
      new NAMKnobControl(bassKnobArea, kToneBass, "", style, knobBackgroundBitmap), -1, "EQ_KNOBS");
    pGraphics->AttachControl(
      new NAMKnobControl(midKnobArea, kToneMid, "", style, knobBackgroundBitmap), -1, "EQ_KNOBS");
    pGraphics->AttachControl(
      new NAMKnobControl(trebleKnobArea, kToneTreble, "", style, knobBackgroundBitmap), -1, "EQ_KNOBS");
    pGraphics->AttachControl(new NAMKnobControl(outputKnobArea, kOutputLevel, "", style, knobBackgroundBitmap));

    // ===== KNOBS FX (rangée 2) =====
    pGraphics->AttachControl(new NAMKnobControl(driveGainArea, kDriveGain, "", style, knobBackgroundBitmap));
    pGraphics->AttachControl(new NAMKnobControl(driveVolumeArea, kDriveVolume, "", style, knobBackgroundBitmap));
    pGraphics->AttachControl(new NAMKnobControl(delayTimeArea, kDelayTime, "", style, knobBackgroundBitmap));
    pGraphics->AttachControl(new NAMKnobControl(delayFeedbackArea, kDelayFeedback, "", style, knobBackgroundBitmap));
    pGraphics->AttachControl(new NAMKnobControl(delayMixArea, kDelayMix, "", style, knobBackgroundBitmap));
    pGraphics->AttachControl(new NAMKnobControl(reverbTimeArea, kReverbTime, "", style, knobBackgroundBitmap));
    pGraphics->AttachControl(new NAMKnobControl(reverbMixArea, kReverbMix, "", style, knobBackgroundBitmap));
    // ==============================

    // ===== SWITCHES FX =====
    pGraphics->AttachControl(
      new NAMSwitchControl(driveSwitchArea, kDriveActive, "Drive", style, switchHandleBitmap));
    pGraphics->AttachControl(
      new NAMSwitchControl(delaySwitchArea, kDelayActive, "Delay", style, switchHandleBitmap));
    pGraphics->AttachControl(
      new NAMSwitchControl(reverbSwitchArea, kReverbActive, "Reverb", style, switchHandleBitmap));
    // Sélecteur de type reverb : 3 radio buttons
    pGraphics->AttachControl(new IVRadioButtonControl(reverbTypeArea, kReverbType,
                                                      {"Room", "Plate", "Spring"}, radioButtonStyle, false, nullptr));
    // =======================

    // The meters
    pGraphics->AttachControl(new NAMMeterControl(inputMeterArea, meterBackgroundBitmap, style), kCtrlTagInputMeter);
    pGraphics->AttachControl(new NAMMeterControl(outputMeterArea, meterBackgroundBitmap, style), kCtrlTagOutputMeter);

    // Settings/help/about box
    pGraphics->AttachControl(new NAMCircleButtonControl(
      settingsButtonArea,
      [pGraphics](IControl* pCaller) {
        pGraphics->GetControlWithTag(kCtrlTagSettingsBox)->As<NAMSettingsPageControl>()->HideAnimated(false);
      },
      gearSVG));

    pGraphics
      ->AttachControl(new NAMSettingsPageControl(b, backgroundBitmap, inputLevelBackgroundBitmap, switchHandleBitmap,
                                                 crossSVG, style, radioButtonStyle),
                      kCtrlTagSettingsBox)
      ->Hide(true);

    const auto slimKnobArea = b.GetCentredInside(100.f, NAM_KNOB_HEIGHT + 24.f);
    pGraphics->AttachControl(new NAMSlimOverlayBackdropControl(b, hideSlimOverlay), kCtrlTagSlimOverlayBackdrop)
      ->Hide(true);
    pGraphics
      ->AttachControl(new NAMKnobControl(slimKnobArea, kSlim, "Slim", style, knobBackgroundBitmap), kCtrlTagSlimKnob)
      ->Hide(true);

    pGraphics->ForAllControlsFunc([](IControl* pControl) {
      pControl->SetMouseEventsWhenDisabled(true);
      pControl->SetMouseOverWhenDisabled(true);
    });

    // Griser les knobs des FX si bypass au démarrage
    auto setFxEnabled = [&](int activeParam, std::initializer_list<int> knobs) {
      const bool on = GetParam(activeParam)->Bool();
      for (int k : knobs)
        pGraphics->GetControlWithParamIdx(k)->SetDisabled(!on);
    };
    setFxEnabled(kDriveActive, {kDriveGain, kDriveVolume});
    setFxEnabled(kDelayActive, {kDelayTime, kDelayFeedback, kDelayMix});
    setFxEnabled(kReverbActive, {kReverbTime, kReverbType, kReverbMix});
  };

  mModelWorker = std::thread([this]() { _ModelWorkerLoop(); });
  mIRWorker = std::thread([this]() { _IRWorkerLoop(); });
}

NeuralAmpModeler::~NeuralAmpModeler()
{
  mModelWorkerQuit = true;
  mIRWorkerQuit = true;
  if (mModelWorker.joinable())
    mModelWorker.join();
  if (mIRWorker.joinable())
    mIRWorker.join();
  _DeallocateIOPointers();
}

void NeuralAmpModeler::ProcessBlock(iplug::sample** inputs, iplug::sample** outputs, int nFrames)
{
  const size_t numChannelsExternalIn = (size_t)NInChansConnected();
  const size_t numChannelsExternalOut = (size_t)NOutChansConnected();
  const size_t numChannelsInternal = kNumChannelsInternal;
  const size_t numFrames = (size_t)nFrames;
  const double sampleRate = GetSampleRate();

  std::fenv_t fe_state;
  std::feholdexcept(&fe_state);
  disable_denormals();

  _PrepareBuffers(numChannelsInternal, numFrames);
  _ProcessInput(inputs, numFrames, numChannelsExternalIn, numChannelsInternal);
  _ApplyDSPStaging();
  const bool noiseGateActive = GetParam(kNoiseGateActive)->Value();
  const bool toneStackActive = GetParam(kEQActive)->Value();

  // Noise gate trigger
  sample** triggerOutput = mInputPointers;
  if (noiseGateActive)
  {
    const double time = 0.01;
    const double threshold = GetParam(kNoiseGateThreshold)->Value();
    const double ratio = 0.1;
    const double openTime = 0.005;
    const double holdTime = 0.01;
    const double closeTime = 0.05;
    const dsp::noise_gate::TriggerParams triggerParams(time, threshold, ratio, openTime, holdTime, closeTime);
    mNoiseGateTrigger.SetParams(triggerParams);
    mNoiseGateTrigger.SetSampleRate(sampleRate);
    triggerOutput = mNoiseGateTrigger.Process(mInputPointers, numChannelsInternal, numFrames);
  }

  // ===== DRIVE (après NG trigger, avant NAM) =====
  if (GetParam(kDriveActive)->Value())
  {
    mDrive.Process(triggerOutput[0], mDrivePointers[0], numFrames);
    triggerOutput = mDrivePointers;
  }
  // ==============================================

  if (mModel != nullptr)
  {
    mModel->process(triggerOutput, mOutputPointers, nFrames);
  }
  else
  {
    _FallbackDSP(triggerOutput, mOutputPointers, numChannelsInternal, numFrames);
  }

  // Noise gate gain
  sample** gateGainOutput =
    noiseGateActive ? mNoiseGateGain.Process(mOutputPointers, numChannelsInternal, numFrames) : mOutputPointers;

  sample** toneStackOutPointers = (toneStackActive && mToneStack != nullptr)
                                    ? mToneStack->Process(gateGainOutput, numChannelsInternal, nFrames)
                                    : gateGainOutput;

  sample** irPointers = toneStackOutPointers;
  if (mIR != nullptr && GetParam(kIRToggle)->Value())
    irPointers = mIR->Process(toneStackOutPointers, numChannelsInternal, numFrames);

  // HPF
  const double highPassCutoffFreq = kDCBlockerFrequency;
  const recursive_linear_filter::HighPassParams highPassParams(sampleRate, highPassCutoffFreq);
  mHighPass.SetParams(highPassParams);
  sample** hpfPointers = mHighPass.Process(irPointers, numChannelsInternal, numFrames);

  // ===== PASSAGE MONO -> STÉRÉO =====
  for (size_t s = 0; s < numFrames; s++)
  {
    mStereoArray[0][s] = hpfPointers[0][s];
    mStereoArray[1][s] = hpfPointers[0][s];
  }

  // ===== DELAY (stéréo) =====
  sample** delayPointers = mStereoPointers;
  if (GetParam(kDelayActive)->Value())
  {
    mDelay.Process(mStereoPointers, mDelayPointers, numFrames);
    delayPointers = mDelayPointers;
  }

  // ===== REVERB (stéréo) =====
  sample** reverbPointers = delayPointers;
  if (GetParam(kReverbActive)->Value())
  {
    mReverb.Process(delayPointers, mReverbPointers, numFrames);
    reverbPointers = mReverbPointers;
  }
  // ===========================

  std::feupdateenv(&fe_state);

  _ProcessOutput(reverbPointers, outputs, numFrames, 2, numChannelsExternalOut);
  _UpdateMeters(mInputPointers, outputs, numFrames, numChannelsInternal, numChannelsExternalOut);
}
void NeuralAmpModeler::OnReset()
{
  const auto sampleRate = GetSampleRate();
  const int maxBlockSize = GetBlockSize();

  const int tailCycles = 10;
  SetTailSize(tailCycles * (int)(sampleRate / kDCBlockerFrequency));
  mInputSender.Reset(sampleRate);
  mOutputSender.Reset(sampleRate);
  _ResetModelAndIR(sampleRate, GetBlockSize());
  mToneStack->Reset(sampleRate, maxBlockSize);

  // ===== NOUVEAUX DSP =====
  mDrive.Reset(sampleRate);
  mDelay.Reset(sampleRate);
  mReverb.Reset(sampleRate);
  // ========================

  _UpdateLatency();
}

void NeuralAmpModeler::OnIdle()
{
  mInputSender.TransmitData(*this);
  mOutputSender.TransmitData(*this);

  if (mModelPathNeedsUIRefresh.exchange(false))
  {
    if (GetUI())
    {
      WDL_String path;
      {
        std::lock_guard<std::mutex> lock(mModelListMutex);
        path = mNAMPath;
      }
      if (path.GetLength())
        SendControlMsgFromDelegate(kCtrlTagModelFileBrowser, kMsgTagLoadedModel, path.GetLength(), path.Get());
    }
  }

  if (mIRPathNeedsUIRefresh.exchange(false))
  {
    if (GetUI())
    {
      WDL_String path;
      {
        std::lock_guard<std::mutex> lock(mIRListMutex);
        path = mIRPath;
      }
      if (path.GetLength())
        SendControlMsgFromDelegate(kCtrlTagIRFileBrowser, kMsgTagLoadedIR, path.GetLength(), path.Get());
    }
  }

  if (mNewModelLoadedInDSP)
  {
    if (auto* pGraphics = GetUI())
    {
      _UpdateControlsFromModel();
      mNewModelLoadedInDSP = false;
    }
  }
  if (mModelCleared)
  {
    if (auto* pGraphics = GetUI())
    {
      static_cast<NAMSettingsPageControl*>(pGraphics->GetControlWithTag(kCtrlTagSettingsBox))->ClearModelInfo();
      if (auto* p = pGraphics->GetControlWithTag(kCtrlTagSlimmableIcon))
        p->Hide(true);
      if (auto* p = pGraphics->GetControlWithTag(kCtrlTagSlimOverlayBackdrop))
        p->Hide(true);
      if (auto* p = pGraphics->GetControlWithTag(kCtrlTagSlimKnob))
        p->Hide(true);
      pGraphics->SetAllControlsDirty();
      mModelCleared = false;
    }
  }
}

bool NeuralAmpModeler::SerializeState(IByteChunk& chunk) const
{
  WDL_String header("###NeuralAmpModeler###");
  chunk.PutStr(header.Get());
  WDL_String version(PLUG_VERSION_STR);
  chunk.PutStr(version.Get());
  {
    std::lock_guard<std::mutex> lock(mModelListMutex);
    chunk.PutStr(mNAMPath.Get());
  }
  {
    std::lock_guard<std::mutex> lock(mIRListMutex);
    chunk.PutStr(mIRPath.Get());
  }
  return SerializeParams(chunk);
}

int NeuralAmpModeler::UnserializeState(const IByteChunk& chunk, int startPos)
{
  WDL_String header;
  int pos = startPos;
  pos = chunk.GetStr(header, pos);

  const char* kExpectedHeader = "###NeuralAmpModeler###";
  if (strcmp(header.Get(), kExpectedHeader) == 0)
  {
    return _UnserializeStateWithKnownVersion(chunk, pos);
  }
  else
  {
    return _UnserializeStateWithUnknownVersion(chunk, startPos);
  }
}

void NeuralAmpModeler::OnUIOpen()
{
  Plugin::OnUIOpen();

  if (mNAMPath.GetLength())
  {
    SendControlMsgFromDelegate(kCtrlTagModelFileBrowser, kMsgTagLoadedModel, mNAMPath.GetLength(), mNAMPath.Get());
    if (mModel == nullptr && mStagedModel == nullptr)
      SendControlMsgFromDelegate(kCtrlTagModelFileBrowser, kMsgTagLoadFailed);
  }

  if (mIRPath.GetLength())
  {
    SendControlMsgFromDelegate(kCtrlTagIRFileBrowser, kMsgTagLoadedIR, mIRPath.GetLength(), mIRPath.Get());
    if (mIR == nullptr && mStagedIR == nullptr)
      SendControlMsgFromDelegate(kCtrlTagIRFileBrowser, kMsgTagLoadFailed);
  }

  if (mModel != nullptr)
  {
    _UpdateControlsFromModel();
  }
}

void NeuralAmpModeler::OnParamChange(int paramIdx)
{
  switch (paramIdx)
  {
    case kCalibrateInput:
    case kInputCalibrationLevel:
    case kInputLevel: _SetInputGain(); break;
    case kOutputLevel:
    case kOutputMode: _SetOutputGain(); break;
    case kToneBass: mToneStack->SetParam("bass", GetParam(paramIdx)->Value()); break;
    case kToneMid: mToneStack->SetParam("middle", GetParam(paramIdx)->Value()); break;
    case kToneTreble: mToneStack->SetParam("treble", GetParam(paramIdx)->Value()); break;
    case kSlim: _ApplySlimParamToLoadedNAMs(); break;

    // ===== NOUVEAUX PARAMS =====
    case kDriveGain:
    case kDriveVolume:
      mDrive.SetParams(GetParam(kDriveGain)->Value(), GetParam(kDriveVolume)->Value());
      break;
    case kDelayTime:
    case kDelayFeedback:
    case kDelayMix:
      mDelay.SetParams(GetParam(kDelayTime)->Value(),
                       GetParam(kDelayFeedback)->Value(),
                       GetParam(kDelayMix)->Value());
      break;
    case kReverbTime:
    case kReverbType:
    case kReverbMix:
      mReverb.SetParams(GetParam(kReverbTime)->Value(),
                        GetParam(kReverbMix)->Value(),
                        static_cast<dsp::reverb::Type>(GetParam(kReverbType)->Int()));
      break;
    // ===========================

    case kModelIndex: mWantedModelIndex.store(GetParam(kModelIndex)->Int()); break;
    case kIRIndex: mWantedIRIndex.store(GetParam(kIRIndex)->Int()); break;
    case kBankIndex: mWantedBankIndex.store(GetParam(kBankIndex)->Int()); break;
    case kIRBankIndex: mWantedIRBankIndex.store(GetParam(kIRBankIndex)->Int()); break;
    default: break;
  }
}

void NeuralAmpModeler::OnParamChangeUI(int paramIdx, EParamSource source)
{
  if (paramIdx == kBankIndex)
  {
    if (_ApplyModelBank(GetParam(kBankIndex)->Int()))
      InformHostOfParameterDetailsChange();
    return;
  }
  if (paramIdx == kIRBankIndex)
  {
    if (_ApplyIRBank(GetParam(kIRBankIndex)->Int()))
      InformHostOfParameterDetailsChange();
    return;
  }

  if (auto pGraphics = GetUI())
  {
    bool active = GetParam(paramIdx)->Bool();

    switch (paramIdx)
    {
      case kNoiseGateActive: pGraphics->GetControlWithParamIdx(kNoiseGateThreshold)->SetDisabled(!active); break;
      case kEQActive:
        pGraphics->ForControlInGroup("EQ_KNOBS", [active](IControl* pControl) { pControl->SetDisabled(!active); });
        break;
      case kIRToggle: pGraphics->GetControlWithTag(kCtrlTagIRFileBrowser)->SetDisabled(!active); break;

      // ===== NOUVEAUX BYPASS =====
      case kDriveActive:
        pGraphics->GetControlWithParamIdx(kDriveGain)->SetDisabled(!active);
        pGraphics->GetControlWithParamIdx(kDriveVolume)->SetDisabled(!active);
        break;
      case kDelayActive:
        pGraphics->GetControlWithParamIdx(kDelayTime)->SetDisabled(!active);
        pGraphics->GetControlWithParamIdx(kDelayFeedback)->SetDisabled(!active);
        pGraphics->GetControlWithParamIdx(kDelayMix)->SetDisabled(!active);
        break;
      case kReverbActive:
        pGraphics->GetControlWithParamIdx(kReverbTime)->SetDisabled(!active);
        pGraphics->GetControlWithParamIdx(kReverbType)->SetDisabled(!active);
        pGraphics->GetControlWithParamIdx(kReverbMix)->SetDisabled(!active);
        break;
      // ==========================

      default: break;
    }
  }
}

bool NeuralAmpModeler::OnMessage(int msgTag, int ctrlTag, int dataSize, const void* pData)
{
  switch (msgTag)
  {
    case kMsgTagClearModel: mShouldRemoveModel = true; return true;
    case kMsgTagClearIR: mShouldRemoveIR = true; return true;
    case kMsgTagHighlightColor:
    {
      mHighLightColor.Set((const char*)pData);
      if (GetUI())
      {
        GetUI()->ForStandardControlsFunc([&](IControl* pControl) {
          if (auto* pVectorBase = pControl->As<IVectorBase>())
          {
            IColor color = IColor::FromColorCodeStr(mHighLightColor.Get());
            pVectorBase->SetColor(kX1, color);
            pVectorBase->SetColor(kPR, color.WithOpacity(0.3f));
            pVectorBase->SetColor(kFR, color.WithOpacity(0.4f));
            pVectorBase->SetColor(kX3, color.WithContrast(0.1f));
          }
          pControl->GetUI()->SetAllControlsDirty();
        });
      }
      return true;
    }
    default: return false;
  }
}

// Private methods ============================================================

void NeuralAmpModeler::_AllocateIOPointers(const size_t nChans)
{
  if (mInputPointers != nullptr)
    throw std::runtime_error("Tried to re-allocate mInputPointers without freeing");
  mInputPointers = new sample*[nChans];
  if (mInputPointers == nullptr)
    throw std::runtime_error("Failed to allocate pointer to input buffer!\n");
  if (mOutputPointers != nullptr)
    throw std::runtime_error("Tried to re-allocate mOutputPointers without freeing");
  mOutputPointers = new sample*[nChans];
  if (mOutputPointers == nullptr)
    throw std::runtime_error("Failed to allocate pointer to output buffer!\n");
}

void NeuralAmpModeler::_ApplyDSPStaging()
{
  if (mShouldRemoveModel)
  {
    mModel = nullptr;
    mNAMPath.Set("");
    mShouldRemoveModel = false;
    mModelCleared = true;
    _UpdateLatency();
    _SetInputGain();
    _SetOutputGain();
  }
  if (mShouldRemoveIR)
  {
    mIR = nullptr;
    mIRPath.Set("");
    mShouldRemoveIR = false;
  }
  if (mStagedModel != nullptr)
  {
    mModel = std::move(mStagedModel);
    mStagedModel = nullptr;
    mNewModelLoadedInDSP = true;
    _UpdateLatency();
    _SetInputGain();
    _SetOutputGain();
  }
  if (mStagedIR != nullptr)
  {
    mIR = std::move(mStagedIR);
    mStagedIR = nullptr;
  }
}

void NeuralAmpModeler::_DeallocateIOPointers()
{
  if (mInputPointers != nullptr)
  {
    delete[] mInputPointers;
    mInputPointers = nullptr;
  }
  if (mOutputPointers != nullptr)
  {
    delete[] mOutputPointers;
    mOutputPointers = nullptr;
  }
  // ===== NOUVEAUX BUFFERS =====
  if (mDrivePointers != nullptr)
  {
    delete[] mDrivePointers;
    mDrivePointers = nullptr;
  }
  if (mStereoPointers != nullptr)
  {
    delete[] mStereoPointers;
    mStereoPointers = nullptr;
  }
  if (mDelayPointers != nullptr)
  {
    delete[] mDelayPointers;
    mDelayPointers = nullptr;
  }
  if (mReverbPointers != nullptr)
  {
    delete[] mReverbPointers;
    mReverbPointers = nullptr;
  }
  // ===========================
}

void NeuralAmpModeler::_FallbackDSP(iplug::sample** inputs, iplug::sample** outputs, const size_t numChannels,
                                    const size_t numFrames)
{
  for (auto c = 0; c < numChannels; c++)
    for (auto s = 0; s < numFrames; s++)
      mOutputArray[c][s] = mInputArray[c][s];
}

void NeuralAmpModeler::_ResetModelAndIR(const double sampleRate, const int maxBlockSize)
{
  if (mStagedModel != nullptr)
  {
    mStagedModel->Reset(sampleRate, maxBlockSize);
  }
  else if (mModel != nullptr)
  {
    mModel->Reset(sampleRate, maxBlockSize);
  }

  if (mStagedIR != nullptr)
  {
    const double irSampleRate = mStagedIR->GetSampleRate();
    if (irSampleRate != sampleRate)
    {
      const auto irData = mStagedIR->GetData();
      mStagedIR = std::make_unique<LongImpulseResponse>(irData, sampleRate);
    }
  }
  else if (mIR != nullptr)
  {
    const double irSampleRate = mIR->GetSampleRate();
    if (irSampleRate != sampleRate)
    {
      const auto irData = mIR->GetData();
      mStagedIR = std::make_unique<LongImpulseResponse>(irData, sampleRate);
    }
  }
}

void NeuralAmpModeler::_SetInputGain()
{
  iplug::sample inputGainDB = GetParam(kInputLevel)->Value();
  if ((mModel != nullptr) && (mModel->HasInputLevel()) && GetParam(kCalibrateInput)->Bool())
  {
    inputGainDB += GetParam(kInputCalibrationLevel)->Value() - mModel->GetInputLevel();
  }
  mInputGain = DBToAmp(inputGainDB);
}

void NeuralAmpModeler::_SetOutputGain()
{
  double gainDB = GetParam(kOutputLevel)->Value();
  if (mModel != nullptr)
  {
    const int outputMode = GetParam(kOutputMode)->Int();
    switch (outputMode)
    {
      case 1:
        if (mModel->HasLoudness())
        {
          const double loudness = mModel->GetLoudness();
          const double targetLoudness = -18.0;
          gainDB += (targetLoudness - loudness);
        }
        break;
      case 2:
        if (mModel->HasOutputLevel())
        {
          const double inputLevel = GetParam(kInputCalibrationLevel)->Value();
          const double outputLevel = mModel->GetOutputLevel();
          gainDB += (outputLevel - inputLevel);
        }
        break;
      case 0:
      default: break;
    }
  }
  mOutputGain = DBToAmp(gainDB);
}

void NeuralAmpModeler::_ApplySlimParamToLoadedNAMs()
{
  const double v = GetParam(kSlim)->Value();
  auto apply = [v](ResamplingNAM* p) {
    if (p == nullptr)
      return;
    if (nam::SlimmableModel* s = p->GetSlimmableModel())
      s->SetSlimmableSize(v);
  };
  apply(mModel.get());
  apply(mStagedModel.get());
}

std::string NeuralAmpModeler::_StageModel(const WDL_String& modelPath)
{
  WDL_String previousNAMPath = mNAMPath;
  try
  {
    auto dspPath = std::filesystem::u8path(modelPath.Get());
    std::unique_ptr<nam::DSP> model = nam::get_dsp(dspPath);

    if (model->NumInputChannels() != 1)
    {
      throw std::runtime_error("Model must have 1 input channel, but has " + std::to_string(model->NumInputChannels()));
    }
    if (model->NumOutputChannels() != 1)
    {
      throw std::runtime_error("Model must have 1 output channel, but has "
                               + std::to_string(model->NumOutputChannels()));
    }

    std::unique_ptr<ResamplingNAM> temp = std::make_unique<ResamplingNAM>(std::move(model), GetSampleRate());
    temp->Reset(GetSampleRate(), GetBlockSize());
    if (nam::SlimmableModel* slimmable = temp->GetSlimmableModel())
    {
      slimmable->SetSlimmableSize(GetParam(kSlim)->Value());
    }
    mStagedModel = std::move(temp);
    mNAMPath = modelPath;
    SendControlMsgFromDelegate(kCtrlTagModelFileBrowser, kMsgTagLoadedModel, mNAMPath.GetLength(), mNAMPath.Get());
  }
  catch (std::runtime_error& e)
  {
    SendControlMsgFromDelegate(kCtrlTagModelFileBrowser, kMsgTagLoadFailed);

    if (mStagedModel != nullptr)
    {
      mStagedModel = nullptr;
    }
    mNAMPath = previousNAMPath;
    std::cerr << "Failed to read DSP module" << std::endl;
    std::cerr << e.what() << std::endl;
    return e.what();
  }
  return "";
}

static void ListBankFolders(const std::filesystem::path& currentFolder, const std::string& extension, size_t maxCount,
                            std::vector<std::filesystem::path>& folders, std::vector<std::string>& names)
{
  folders.clear();
  names.clear();
  const std::filesystem::path root = currentFolder.parent_path();

  auto isHomeFolder = [&]() {
    for (const char* var : {"HOME", "USERPROFILE"})
    {
      const char* home = std::getenv(var);
      if (home != nullptr && *home != '\0')
      {
        std::error_code ec;
        const bool same = std::filesystem::equivalent(root, std::filesystem::path(home), ec);
        if (!ec && same)
          return true;
      }
    }
    return false;
  };
  auto hasFileWithExtension = [&](const std::filesystem::path& dir) {
    std::error_code ec;
    for (std::filesystem::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec))
    {
      std::error_code ec2;
      if (!it->is_regular_file(ec2))
        continue;
      const std::string fileName = it->path().filename().string();
      if (fileName.empty() || fileName[0] == '.')
        continue;
      std::string ext = it->path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return std::tolower(ch); });
      if (ext == extension)
        return true;
    }
    return false;
  };

  std::vector<std::filesystem::path> found;
  if (root.empty() || isHomeFolder())
  {
    found.push_back(currentFolder);
  }
  else
  {
    std::error_code ec;
    for (std::filesystem::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec))
    {
      std::error_code ec2;
      if (!it->is_directory(ec2))
        continue;
      const std::string dirName = it->path().filename().string();
      if (dirName.empty() || dirName[0] == '.')
        continue;
      if (it->path().filename() == currentFolder.filename() || hasFileWithExtension(it->path()))
        found.push_back(it->path());
    }
  }

  auto lowerName = [](const std::filesystem::path& p) {
    std::string s = p.filename().string();
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) { return std::tolower(ch); });
    return s;
  };
  std::sort(found.begin(), found.end(),
            [&](const std::filesystem::path& a, const std::filesystem::path& b) { return lowerName(a) < lowerName(b); });

  if (found.size() > maxCount)
  {
    const auto current = std::find_if(found.begin(), found.end(), [&](const std::filesystem::path& p) {
      return p.filename() == currentFolder.filename();
    });
    const bool currentIsKept = current != found.end() && static_cast<size_t>(current - found.begin()) < maxCount;
    const std::filesystem::path currentPath = current != found.end() ? *current : currentFolder;
    found.resize(maxCount);
    if (!currentIsKept)
      found.back() = currentPath;
  }

  for (const auto& p : found)
  {
    folders.push_back(p);
    names.push_back(p.filename().string());
  }
}

void NeuralAmpModeler::_RefreshModelBanks(const std::filesystem::path& currentFolder)
{
  std::vector<std::filesystem::path> folders;
  std::vector<std::string> names;
  ListBankFolders(currentFolder, ".nam", static_cast<size_t>(kMaxBanks), folders, names);

  int bankIndex = -1;
  for (size_t i = 0; i < folders.size(); i++)
  {
    if (folders[i].filename() == currentFolder.filename())
    {
      bankIndex = static_cast<int>(i);
      break;
    }
  }
  {
    std::lock_guard<std::mutex> lock(mModelListMutex);
    mBankFolders = std::move(folders);
    mBankNames = std::move(names);
  }
  if (bankIndex >= 0)
  {
    mCurrentBankIndex.store(bankIndex);
    mWantedBankIndex.store(bankIndex);
    GetParam(kBankIndex)->Set(bankIndex);
  }
}

void NeuralAmpModeler::_RefreshIRBanks(const std::filesystem::path& currentFolder)
{
  std::vector<std::filesystem::path> folders;
  std::vector<std::string> names;
  ListBankFolders(currentFolder, ".wav", static_cast<size_t>(kMaxBanks), folders, names);

  int bankIndex = -1;
  for (size_t i = 0; i < folders.size(); i++)
  {
    if (folders[i].filename() == currentFolder.filename())
    {
      bankIndex = static_cast<int>(i);
      break;
    }
  }
  {
    std::lock_guard<std::mutex> lock(mIRListMutex);
    mIRBankFolders = std::move(folders);
    mIRBankNames = std::move(names);
  }
  if (bankIndex >= 0)
  {
    mCurrentIRBankIndex.store(bankIndex);
    mWantedIRBankIndex.store(bankIndex);
    GetParam(kIRBankIndex)->Set(bankIndex);
  }
}

bool NeuralAmpModeler::_ApplyModelBank(int idx)
{
  std::filesystem::path folder;
  {
    std::lock_guard<std::mutex> lock(mModelListMutex);
    if (mBankFolders.empty())
      return false;
    idx = std::clamp(idx, 0, static_cast<int>(mBankFolders.size()) - 1);
    if (idx == mCurrentBankIndex.load())
      return false;
    folder = mBankFolders[static_cast<size_t>(idx)];
  }
  _ScanModelFolder(folder);
  mCurrentBankIndex.store(idx);
  mLoadedModelIndex.store(-1);
  return true;
}

bool NeuralAmpModeler::_ApplyIRBank(int idx)
{
  std::filesystem::path folder;
  {
    std::lock_guard<std::mutex> lock(mIRListMutex);
    if (mIRBankFolders.empty())
      return false;
    idx = std::clamp(idx, 0, static_cast<int>(mIRBankFolders.size()) - 1);
    if (idx == mCurrentIRBankIndex.load())
      return false;
    folder = mIRBankFolders[static_cast<size_t>(idx)];
  }
  _ScanIRFolder(folder);
  mCurrentIRBankIndex.store(idx);
  mLoadedIRIndex.store(-1);
  return true;
}

void NeuralAmpModeler::_ScanModelFolder(const std::filesystem::path& folder)
{
  std::vector<std::filesystem::path> files;
  std::error_code ec;
  for (std::filesystem::directory_iterator it(folder, ec), end; !ec && it != end; it.increment(ec))
  {
    std::error_code ec2;
    if (!it->is_regular_file(ec2))
      continue;
    const std::string fileName = it->path().filename().string();
    if (fileName.empty() || fileName[0] == '.')
      continue;
    std::string ext = it->path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return std::tolower(ch); });
    if (ext == ".nam")
      files.push_back(it->path());
  }

  auto lowerName = [](const std::filesystem::path& p) {
    std::string s = p.filename().string();
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) { return std::tolower(ch); });
    return s;
  };
  std::sort(files.begin(), files.end(),
            [&](const std::filesystem::path& a, const std::filesystem::path& b) { return lowerName(a) < lowerName(b); });
  if (files.size() > static_cast<size_t>(kMaxModels))
    files.resize(static_cast<size_t>(kMaxModels));

  std::vector<std::string> names;
  names.reserve(files.size());
  for (const auto& f : files)
    names.push_back(f.stem().string());

  std::lock_guard<std::mutex> lock(mModelListMutex);
  mModelFiles = std::move(files);
  mModelNames = std::move(names);
}

void NeuralAmpModeler::_SetModelFolderFromFile(const WDL_String& filePath, bool notifyHost)
{
  const std::filesystem::path file = std::filesystem::u8path(filePath.Get());
  _ScanModelFolder(file.parent_path());
  _RefreshModelBanks(file.parent_path());

  int index = -1;
  {
    std::lock_guard<std::mutex> lock(mModelListMutex);
    for (size_t i = 0; i < mModelFiles.size(); i++)
    {
      if (mModelFiles[i].filename() == file.filename())
      {
        index = static_cast<int>(i);
        break;
      }
    }
  }
  if (index < 0)
    return;

  mLoadedModelIndex.store(index);
  mWantedModelIndex.store(index);
  GetParam(kModelIndex)->Set(index);

  if (notifyHost)
  {
    InformHostOfParameterDetailsChange();
    const double normalized = GetParam(kModelIndex)->ToNormalized(index);
    BeginInformHostOfParamChangeFromUI(kModelIndex);
    SendParameterValueFromUI(kModelIndex, normalized);
    EndInformHostOfParamChangeFromUI(kModelIndex);
    const double bankNormalized = GetParam(kBankIndex)->ToNormalized(GetParam(kBankIndex)->Int());
    BeginInformHostOfParamChangeFromUI(kBankIndex);
    SendParameterValueFromUI(kBankIndex, bankNormalized);
    EndInformHostOfParamChangeFromUI(kBankIndex);
  }
}

void NeuralAmpModeler::_ModelWorkerLoop()
{
  int lastSeen = -1;
  int stableCount = 0;
  while (!mModelWorkerQuit.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    const int wantedBank = mWantedBankIndex.load();
    if (wantedBank >= 0 && wantedBank != mCurrentBankIndex.load())
      _ApplyModelBank(wantedBank);

    const int wanted = mWantedModelIndex.load();
    if (wanted < 0)
      continue;

    std::filesystem::path file;
    int effective = -1;
    {
      std::lock_guard<std::mutex> lock(mModelListMutex);
      if (!mModelFiles.empty())
      {
        effective = std::min(wanted, static_cast<int>(mModelFiles.size()) - 1);
        file = mModelFiles[static_cast<size_t>(effective)];
      }
    }
    if (effective < 0 || effective == mLoadedModelIndex.load())
    {
      lastSeen = effective;
      stableCount = 0;
      continue;
    }

    if (effective != lastSeen)
    {
      lastSeen = effective;
      stableCount = 0;
      continue;
    }
    if (++stableCount < 3)
      continue;

    const std::string msg = _StageModelInBackground(file);
    if (msg.size())
      std::cerr << "Failed to load model " << file.string() << ": " << msg << std::endl;
    mLoadedModelIndex.store(effective);
    stableCount = 0;
  }
}

std::string NeuralAmpModeler::_StageModelInBackground(const std::filesystem::path& path)
{
  try
  {
    std::unique_ptr<nam::DSP> model = nam::get_dsp(path);
    if (model->NumInputChannels() != 1)
      throw std::runtime_error("Model must have 1 input channel, but has " + std::to_string(model->NumInputChannels()));
    if (model->NumOutputChannels() != 1)
      throw std::runtime_error("Model must have 1 output channel, but has "
                               + std::to_string(model->NumOutputChannels()));

    std::unique_ptr<ResamplingNAM> temp = std::make_unique<ResamplingNAM>(std::move(model), GetSampleRate());
    temp->Reset(GetSampleRate(), GetBlockSize());
    if (nam::SlimmableModel* slimmable = temp->GetSlimmableModel())
      slimmable->SetSlimmableSize(GetParam(kSlim)->Value());
    mStagedModel = std::move(temp);
    {
      std::lock_guard<std::mutex> lock(mModelListMutex);
      mNAMPath.Set(path.string().c_str());
    }
    mModelPathNeedsUIRefresh = true;
  }
  catch (std::exception& e)
  {
    return e.what();
  }
  return "";
}

void NeuralAmpModeler::_ScanIRFolder(const std::filesystem::path& folder)
{
  std::vector<std::filesystem::path> files;
  std::error_code ec;
  for (std::filesystem::directory_iterator it(folder, ec), end; !ec && it != end; it.increment(ec))
  {
    std::error_code ec2;
    if (!it->is_regular_file(ec2))
      continue;
    const std::string fileName = it->path().filename().string();
    if (fileName.empty() || fileName[0] == '.')
      continue;
    std::string ext = it->path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return std::tolower(ch); });
    if (ext == ".wav")
      files.push_back(it->path());
  }

  auto lowerName = [](const std::filesystem::path& p) {
    std::string s = p.filename().string();
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) { return std::tolower(ch); });
    return s;
  };
  std::sort(files.begin(), files.end(),
            [&](const std::filesystem::path& a, const std::filesystem::path& b) { return lowerName(a) < lowerName(b); });
  if (files.size() > static_cast<size_t>(kMaxIRs))
    files.resize(static_cast<size_t>(kMaxIRs));

  std::vector<std::string> names;
  names.reserve(files.size());
  for (const auto& f : files)
    names.push_back(f.stem().string());

  std::lock_guard<std::mutex> lock(mIRListMutex);
  mIRFiles = std::move(files);
  mIRNames = std::move(names);
}

void NeuralAmpModeler::_SetIRFolderFromFile(const WDL_String& filePath, bool notifyHost)
{
  const std::filesystem::path file = std::filesystem::u8path(filePath.Get());
  _ScanIRFolder(file.parent_path());
  _RefreshIRBanks(file.parent_path());

  int index = -1;
  {
    std::lock_guard<std::mutex> lock(mIRListMutex);
    for (size_t i = 0; i < mIRFiles.size(); i++)
    {
      if (mIRFiles[i].filename() == file.filename())
      {
        index = static_cast<int>(i);
        break;
      }
    }
  }
  if (index < 0)
    return;

  mLoadedIRIndex.store(index);
  mWantedIRIndex.store(index);
  GetParam(kIRIndex)->Set(index);

  if (notifyHost)
  {
    InformHostOfParameterDetailsChange();
    const double normalized = GetParam(kIRIndex)->ToNormalized(index);
    BeginInformHostOfParamChangeFromUI(kIRIndex);
    SendParameterValueFromUI(kIRIndex, normalized);
    EndInformHostOfParamChangeFromUI(kIRIndex);
    const double bankNormalized = GetParam(kIRBankIndex)->ToNormalized(GetParam(kIRBankIndex)->Int());
    BeginInformHostOfParamChangeFromUI(kIRBankIndex);
    SendParameterValueFromUI(kIRBankIndex, bankNormalized);
    EndInformHostOfParamChangeFromUI(kIRBankIndex);
  }
}

void NeuralAmpModeler::_IRWorkerLoop()
{
  int lastSeen = -1;
  int stableCount = 0;
  while (!mIRWorkerQuit.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    const int wantedBank = mWantedIRBankIndex.load();
    if (wantedBank >= 0 && wantedBank != mCurrentIRBankIndex.load())
      _ApplyIRBank(wantedBank);

    const int wanted = mWantedIRIndex.load();
    if (wanted < 0)
      continue;

    std::filesystem::path file;
    int effective = -1;
    {
      std::lock_guard<std::mutex> lock(mIRListMutex);
      if (!mIRFiles.empty())
      {
        effective = std::min(wanted, static_cast<int>(mIRFiles.size()) - 1);
        file = mIRFiles[static_cast<size_t>(effective)];
      }
    }
    if (effective < 0 || effective == mLoadedIRIndex.load())
    {
      lastSeen = effective;
      stableCount = 0;
      continue;
    }

    if (effective != lastSeen)
    {
      lastSeen = effective;
      stableCount = 0;
      continue;
    }
    if (++stableCount < 3)
      continue;

    const std::string msg = _StageIRInBackground(file);
    if (msg.size())
      std::cerr << "Failed to load IR " << file.string() << ": " << msg << std::endl;
    mLoadedIRIndex.store(effective);
    stableCount = 0;
  }
}

std::string NeuralAmpModeler::_StageIRInBackground(const std::filesystem
