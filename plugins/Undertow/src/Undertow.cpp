// Undertow.cpp - an "altered delay" for the MPC by Fullban, inspired by the character of
// the DigiTech Obscura pedal (analogue / tape / lo-fi / reverse delay, degrading repeats,
// runaway feedback, tails). Original DSP, no code or samples from any product.
//
// Signal flow (per channel):
//
//   in --> [+ feedback] --> delay line --> tap --> MODE colouring --> wet
//                                          ^                                          |
//                                          +---------- repeats (feedback) <----------+
//   out = dry + wet * Mix
//
//   Modes:
//     Analogue : darkening two-pole tone filter + soft drive (bucket-brigade feel), hiss with Degrade
//     Tape     : slow wow + fast flutter on the delay time, tape saturation, hiss (all grow with Degrade)
//     Lo-Fi    : sample-rate reduction + bit crushing inside the loop, so every repeat gets dirtier
//     Reverse  : two overlapping reversed grains of length Time (Hann cross-faded)
//   Repeats go past 100% into self-oscillation; a soft limiter in the loop keeps it bounded.
//   Tails: On = repeats ring out after the input stops. Off = the delay is cut shortly after the input goes silent.
//
// Parameters (order is permanent once users have projects):
//   0 Mix  1 Mode  2 Time  3 Sync  4 Repeats  5 Tone  6 Degrade  7 Spread  8 Tails

#include "mpc_plugin.h"
#include "dsp/math.h"
#include "dsp/delay_line.h"
#include "dsp/onepole.h"
#include "dsp/smoother.h"
#include "dsp/tempo.h"
#include "dsp/noise.h"

#include <cmath>

using namespace mpc::dsp;

class Undertow : public mpc::Plugin
{
public:
    enum ParamIndex { kMix = 0, kMode, kTime, kSync, kRepeats, kTone, kDegrade, kSpread, kTails, kNumParams };
    enum Mode { kAnalogue = 0, kTape, kLoFi, kReverse, kNumModes };

    static mpc::PluginInfo makeInfo()
    {
        mpc::PluginInfo i;
        i.name = "Undertow";
        i.vendor = "Fullban";
        i.uniqueId = MPC_FOURCC('F', 'E', 'Y', 'Y');   // == "uid" in plugin.json
        i.version = 1050;
        i.isInstrument = false;
        i.numInputs = 2;
        i.numOutputs = 2;
        i.tailSamples = 0;
        return i;
    }

    Undertow() : mpc::Plugin(makeInfo())
    {
        using namespace mpc;
        addParam(floatParam ("mix",     "Mix",     0.0f, 100.0f, 40.0f, "%", 0));        // 0: harmless on host nudge
        addParam(choiceParam("mode",    "Mode",    {"Analogue", "Tape", "Lo-Fi", "Reverse"}, kTape));
        addParam(logParam   ("time",    "Time",    20.0f, 2000.0f, 380.0f, "ms", 0));
        std::vector<std::string> syncNames; syncNames.push_back("Free");
        for (const std::string& s : divisionNames()) syncNames.push_back(s);
        addParam(choiceParam("sync",    "Sync",    syncNames, 0));
        addParam(floatParam ("repeats", "Repeats", 0.0f, 100.0f, 45.0f, "%", 0));
        addParam(floatParam ("tone",    "Tone",    0.0f, 100.0f, 45.0f, "%", 0));
        addParam(floatParam ("degrade", "Degrade", 0.0f, 100.0f, 40.0f, "%", 0));
        addParam(floatParam ("spread",  "Spread",  0.0f, 100.0f, 25.0f, "%", 0));
        addParam(boolParam  ("tails",   "Tails",   true));
        if (numParams() != kNumParams) mpc::log("Undertow: parameter count mismatch");

        //                              mix mode  time  sync rep tone deg spread tails
        addProgram("Dark Tape",      { 38, kTape,     380, 0, 45, 45, 40, 25, 1 });
        addProgram("Analogue Echo",  { 38, kAnalogue, 320, 0, 40, 55, 25, 20, 1 });
        addProgram("Lo-Fi Bounce",   { 40, kLoFi,     400, 8, 50, 55, 55, 30, 1 });
        addProgram("Backwards",      { 42, kReverse,  600, 0, 35, 55, 25, 30, 1 });
        addProgram("Slapback",       { 32, kAnalogue,  90, 0, 14, 60, 20, 10, 1 });
        addProgram("Murky Dub",      { 40, kAnalogue, 500, 11, 66, 25, 50, 50, 1 });
        addProgram("Runaway Tape",   { 40, kTape,     450, 0, 96, 40, 60, 40, 1 });
        addProgram("Reverse Sync",   { 40, kReverse,  500, 11, 40, 60, 30, 40, 1 });
        addProgram("Crushed Echo",   { 36, kLoFi,     250, 0, 55, 50, 80, 25, 1 });
    }

    void prepare(double sampleRate, int /*maxBlockSize*/) override
    {
        sr_ = (float)sampleRate;
        // Reverse needs up to twice the longest delay (2 s) => 4 s of memory.
        const int maxSamples = (int)(sampleRate * 4.0) + 8;
        dl_[0].prepare(maxSamples);
        dl_[1].prepare(maxSamples);
        time_.setTime(140.0f, sampleRate);       // glide: tape-style pitch bend when Time changes
        mix_.setTime(25.0f, sampleRate);
        fb_.setTime(25.0f, sampleRate);
        deg_.setTime(40.0f, sampleRate);
        tone_.setTime(40.0f, sampleRate);
        spread_.setTime(40.0f, sampleRate);
        gate_.setTime(40.0f, sampleRate);
        reset();
    }

    void reset() override
    {
        dl_[0].reset(); dl_[1].reset();
        for (int c = 0; c < 2; ++c)
        {
            lp1_[c].reset(); lp2_[c].reset(); hissLp_[c].reset();
            held_[c] = 0.0f; wetPrev_[c] = 0.0f;
        }
        holdCount_ = 0; silent_ = 0; cleared_ = false; gate_.snap(1.0f);
        wowPh_ = 0.0f; flutPh_ = 0.31f; revPh_ = 0.0f;
        time_.snap(time_.target());
        mix_.snap(get(kMix) * 0.01f);
        fb_.snap(0.0f);
        deg_.snap(get(kDegrade) * 0.01f);
        tone_.snap(get(kTone) * 0.01f);
        spread_.snap(get(kSpread) * 0.01f);
    }

    void process(mpc::ProcessContext& ctx) override
    {
        const float sr = (float)ctx.sampleRate;

        // ----- delay time (free ms, or tempo division) -----------------------------------
        float ms;
        const int syncIdx = getInt(kSync);
        if (syncIdx > 0) ms = beatsToSeconds(divisionBeats(syncIdx - 1), ctx.transport.bpm) * 1000.0f;
        else             ms = get(kTime);
        while (ms > 2000.0f) ms *= 0.5f;   // very long divisions at slow tempos: fold down
        if (ms < 5.0f) ms = 5.0f;
        time_.setTarget(msToSamples(ms, sr));

        const int   mode   = getInt(kMode);
        const float repeats = get(kRepeats) * 0.01f;
        const bool  tails  = getBool(kTails);
        const int   silentLimit = (int)(0.20f * sr), clearLimit = (int)(0.60f * sr);
        mix_.setTarget(get(kMix) * 0.01f);
        deg_.setTarget(get(kDegrade) * 0.01f);
        tone_.setTarget(get(kTone) * 0.01f);
        spread_.setTarget(get(kSpread) * 0.01f);
        // Repeats 0..100% -> feedback 0..1.06 (past 1.0 = self-oscillation, kept bounded by the limiter)
        fb_.setTarget(repeats * 1.06f);

        const float* inL = ctx.inputs[0];
        const float* inR = ctx.numInputs > 1 ? ctx.inputs[1] : ctx.inputs[0];
        float* outL = ctx.outputs[0];
        float* outR = ctx.numOutputs > 1 ? ctx.outputs[1] : ctx.outputs[0];

        const float wowInc  = 0.55f / sr;     // slow wobble ~0.55 Hz
        const float flutInc = 6.7f / sr;      // flutter ~6.7 Hz

        for (int i = 0; i < ctx.numFrames; ++i)
        {
            const float d    = time_.process();
            const float mix  = mix_.process();
            const float deg  = deg_.process();
            const float tone = tone_.process();
            const float spr  = spread_.process();
            const float sprE = spr;
            float fb = fb_.process();

            wowPh_  += wowInc;  if (wowPh_  >= 1.0f) wowPh_  -= 1.0f;
            flutPh_ += flutInc; if (flutPh_ >= 1.0f) flutPh_ -= 1.0f;

            // Tone: feedback/wet low-pass cutoff, darker with Degrade
            float cutoff = 400.0f * std::pow(10.0f, 1.7f * tone) * (1.0f - 0.45f * deg);
            cutoff = clamp(cutoff, 200.0f, 0.45f * sr);
            const float a = 1.0f - std::exp(-6.2831853f * cutoff / sr);

            // Modulation (Tape: wow + flutter, Analogue: a touch of clock drift)
            float mod = 0.0f;   // in samples
            if (mode == kTape)
                mod = msToSamples((0.25f + 2.4f * deg) * fastSin01(wowPh_) + (0.04f + 0.30f * deg) * fastSin01(flutPh_), sr);
            else if (mode == kAnalogue)
                mod = msToSamples(0.10f * (0.3f + deg) * fastSin01(wowPh_), sr);

            // Reverse grain phase (chunk length = delay time)
            const float chunk = clamp(d, 64.0f, sr * 2.0f);
            if (mode == kReverse) { revPh_ += 1.0f / chunk; if (revPh_ >= 1.0f) revPh_ -= 1.0f; }

            float dry[2] = { inL[i], inR[i] };
            float tap[2], wet[2], wo[2];

            for (int c = 0; c < 2; ++c)
            {
                // R channel gets a slightly longer time as Spread rises
                const float dc = d * (1.0f + (c == 1 ? 0.14f * sprE : 0.0f));

                if (mode == kReverse)
                {
                    const float ph0 = revPh_, ph1 = (revPh_ >= 0.5f) ? revPh_ - 0.5f : revPh_ + 0.5f;
                    const float L = chunk * (1.0f + (c == 1 ? 0.14f * sprE : 0.0f));
                    const float s0 = fastSin01(ph0 * 0.5f), s1 = fastSin01(ph1 * 0.5f);   // sin(pi*ph)
                    // fastSin01 is sin(2*pi*x): x = ph/2 gives sin(pi*ph); square for a Hann window
                    tap[c] = dl_[c].readLinear(1.0f + 2.0f * L * ph0) * (s0 * s0)
                           + dl_[c].readLinear(1.0f + 2.0f * L * ph1) * (s1 * s1);
                }
                else
                    tap[c] = dl_[c].readLinear(dc + mod);

                // ----- mode colouring (this is what each repeat goes through) -----
                float x = tap[c];
                if (mode == kLoFi)
                {
                    const int period = 1 + (int)(deg * 10.0f);              // 1..11 -> 48k .. ~4.4k
                    if (holdCount_ == 0) held_[c] = x;                      // sample & hold
                    const float levels = std::pow(2.0f, 12.0f - 7.0f * deg); // 12 -> 5 bits
                    const float q = std::floor(held_[c] * levels + 0.5f) / levels;
                    x = q;
                    if (c == 1) { if (++holdCount_ >= period) holdCount_ = 0; }
                }
                else if (mode == kTape || mode == kAnalogue)
                {
                    // Saturation with unity small-signal gain (so it never adds loop gain); makeup is applied to the output only
                    const float drive = 1.0f + deg * (mode == kTape ? 2.2f : 1.6f);
                    x = fastTanh(x * drive) / drive;
                }
                // hiss grows with Degrade (not in Lo-Fi)
                if (mode != kLoFi)
                    x += hissLp_[c].process(noise_.process()) * (0.0006f + 0.006f * deg * deg);

                x = lp1_[c].process(x);
                lp1_[c].setCoeff(a);
                if (mode == kAnalogue || mode == kTape) { x = lp2_[c].process(x); lp2_[c].setCoeff(a); }

                // Soft limiter in the loop: unity at low level, bounded at +/-0.8
                const float lim = 0.8f * fastTanh(x * 1.25f);
                wet[c] = lim;
                // makeup for the saturation so Degrade does not make the echoes quieter (output only)
                wo[c] = wet[c] * (1.0f + (mode == kTape ? 0.9f : (mode == kAnalogue ? 0.6f : 0.0f)) * deg);
            }

            // Cross-feed (ping-pong) with Spread, input to R reduced so repeats alternate
            const float cx = 0.5f * sprE;
            const float fL = (1.0f - cx) * wet[0] + cx * wet[1];
            const float fR = (1.0f - cx) * wet[1] + cx * wet[0];
            const float inGain = 1.0f;
            const float inRg   = inGain * (1.0f - 0.6f * spr);
            dl_[0].write(flushDenormal(sanitize(dry[0] * inGain + fL * fb)));
            dl_[1].write(flushDenormal(sanitize(dry[1] * inRg   + fR * fb)));

            // Tails Off: close the wet path ~0.2 s after the input goes quiet, wipe the delay after ~0.6 s
            const float inLevel = std::fabs(dry[0]) > std::fabs(dry[1]) ? std::fabs(dry[0]) : std::fabs(dry[1]);
            if (inLevel > 0.0005f) { silent_ = 0; cleared_ = false; }
            else if (silent_ < 1000000) ++silent_;
            const bool closed = !tails && silent_ > silentLimit;
            gate_.setTarget(closed ? 0.0f : 1.0f);
            const float gg = gate_.process();
            if (!tails && !cleared_ && silent_ > clearLimit) { dl_[0].reset(); dl_[1].reset(); cleared_ = true; }

            const float wg = mix * 1.2f * gg;
            outL[i] = dry[0] + wo[0] * wg;
            outR[i] = dry[1] + wo[1] * wg;
        }
    }

private:
    float sr_ = 44100.0f;
    DelayLine dl_[2];
    OnePoleLP lp1_[2], lp2_[2], hissLp_[2];
    Smoother time_, mix_, fb_, deg_, tone_, spread_, gate_;
    int silent_ = 0;
    bool cleared_ = false;
    WhiteNoise noise_;
    float held_[2] = {0, 0}, wetPrev_[2] = {0, 0};
    int holdCount_ = 0;
    float wowPh_ = 0.0f, flutPh_ = 0.31f, revPh_ = 0.0f;
};

MPC_PLUGIN_ENTRY(Undertow)
