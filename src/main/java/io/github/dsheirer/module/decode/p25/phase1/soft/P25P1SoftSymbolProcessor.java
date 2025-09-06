/*
 * *****************************************************************************
 * Copyright (C) 2014-2025 Dennis Sheirer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 * ****************************************************************************
 */

package io.github.dsheirer.module.decode.p25.phase1.soft;

import io.github.dsheirer.bits.CorrectedBinaryMessage;
import io.github.dsheirer.bits.IntField;
import io.github.dsheirer.dsp.filter.interpolator.LinearInterpolator;
import io.github.dsheirer.dsp.symbol.Dibit;
import io.github.dsheirer.dsp.symbol.DibitDelayLine;
import io.github.dsheirer.dsp.symbol.DibitToByteBufferAssembler;
import io.github.dsheirer.edac.bch.BCH_63_16_23_P25;
import io.github.dsheirer.log.LoggingSuppressor;
import io.github.dsheirer.module.decode.dmr.sync.visualizer.SyncResultsViewer;
import io.github.dsheirer.module.decode.p25.phase1.Modulation;
import io.github.dsheirer.module.decode.p25.phase1.P25P1DataUnitID;
import io.github.dsheirer.module.decode.p25.phase1.sync.P25P1SoftSyncDetector;
import io.github.dsheirer.module.decode.p25.phase1.sync.P25P1SoftSyncDetectorFactory;
import io.github.dsheirer.module.decode.p25.phase1.sync.P25P1SyncDetector;
import io.github.dsheirer.sample.Listener;
import java.nio.ByteBuffer;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.concurrent.CountDownLatch;
import org.apache.commons.math3.stat.descriptive.moment.StandardDeviation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class P25P1SoftSymbolProcessor
{
    private static final Logger LOGGER = LoggerFactory.getLogger(P25P1SoftSymbolProcessor.class);
    private static final LoggingSuppressor LOGGING_SUPPRESSOR = new LoggingSuppressor(LOGGER);
    private static final int DIBIT_LENGTH_NID = 33; //32 dibits (64 bits) +1 status
    private static final int DIBIT_LENGTH_SYNC = 24;
    private static final Correction INVALID_SYNC_DETECTION = new Correction(Double.MAX_VALUE, 0f, 0f);
    private static final float SOFT_SYMBOL_MAX_POSITIVE_PHASE = 3.5f;
    private static final float SOFT_SYMBOL_MAX_NEGATIVE_PHASE = -3.5f;
    private static final float SOFT_SYMBOL_QUADRANT_BOUNDARY = (float)(Math.PI / 2.0);
    private static final float SYNC_THRESHOLD_DETECTION = 60;
    private static final float SYNC_THRESHOLD_OPTIMIZED = 80;
    private static final float EQUALIZER_LOOP_GAIN = 0.25f;
    private static final float EQUALIZER_MAXIMUM_BALANCE = (float)(Math.PI / 4.0);
    private static final float EQUALIZER_RECALIBRATE_THRESHOLD = (float)(Math.PI / 8.0);
    private static final float EQUALIZER_MAXIMUM_GAIN = 1.35f;
    private static final float SAMPLES_PER_SYMBOL_ALLOWABLE_DEVIATION = 0.005f; //.5%
    private static final float TWO_PI = (float)(Math.PI * 2.0);
    private static final float[] SYNC_PATTERN_SYMBOLS = P25P1SyncDetector.syncPatternToSymbols();
    private static final Dibit[] SYNC_PATTERN_DIBITS = P25P1SyncDetector.syncPatternToDibits();
    private P25P1SoftSyncDetector mSyncDetector = P25P1SoftSyncDetectorFactory.getDetector();
    private P25P1SoftSyncDetector mSyncDetectorLag1 = P25P1SoftSyncDetectorFactory.getDetector();
    private P25P1SoftSyncDetector mSyncDetectorLag2 = P25P1SoftSyncDetectorFactory.getDetector();
    private DibitToByteBufferAssembler mDibitAssembler = new DibitToByteBufferAssembler(300);
    private P25P1SoftMessageFramer mMessageFramer;
    private DibitDelayLine mSymbolDelayLine = new DibitDelayLine(DIBIT_LENGTH_SYNC);
    private boolean mFineSync = false;
    private float mLaggingSyncOffset1;
    private float mLaggingSyncOffset2;
    private double mSamplesPerSymbol;
    private double mObservedSamplesPerSymbol;
    private double mSyncOffset;
    private int mSyncOffsetForReload;
    private double mSamplePoint;
    private float[] mBuffer;
    private int mBufferLoadPointer;
    private int mBufferReloadThreshold;
    private int mBufferPointer;
    private int mBufferWorkspaceLength = 1024;
    private int mPreviousMessageSymbolLength;
    private P25P1DataUnitID mPreviousDataUnitID;
    private int mSymbolsSinceLastSync = DIBIT_LENGTH_NID; //Set to NID length (72) to prevent false initial NID calculation.
    private final BCH_63_16_23_P25 mBCHDecoder = new BCH_63_16_23_P25();
    public static final IntField NAC_FIELD = IntField.length12(0);
    public static final IntField DUID_FIELD = IntField.length4(12);
    private NACTracker mNACTracker = new NACTracker();
    private int mDebugSymbolCount;
    private long mDebugSampleCount = 0;
    private long mDebugSymbolCountLastDetect = 0;
    private P25P1DataUnitID mCurrentDataUnitID = P25P1DataUnitID.UNKNOWN;
    private double mNoiseStandardDeviationThreshold;
    private float mOptimizeFineIncrement;
    private SyncResultsViewer mSyncResultsViewer;
    private static final DecimalFormat DF = new DecimalFormat(" 0.000000;-0.000000");
    private Modulation mModulation;
    private Equalizer mEqualizer;
    private boolean mSyncDetectionMode = true;

    /**
     * Constructs an instance
     * @param messageFramer to receive symbol decisions (dibits) and sync notifications.
     */
    public P25P1SoftSymbolProcessor(P25P1SoftMessageFramer messageFramer, Modulation modulation)
    {
        mMessageFramer = messageFramer;
        mModulation = modulation;
        mEqualizer = (mModulation == Modulation.C4FM ? new C4FMEqualizer() : new CQPSKEqualizer());
    }

    /**
     * Primary input method for receiving a stream of demodulated samples to process into symbols.
     * @param samples to process
     */
    public boolean receive(float[] samples)
    {
        boolean debugReturn = false;
        int samplesPointer = 0;
        float softSymbol;
        Dibit delayedSymbol;
        Correction correctionCandidate;

        while(samplesPointer < samples.length)
        {
            //Note: buffer pointer can become greater than reload threshold during timing optimization at sync detect
            if(mBufferPointer >= mBufferReloadThreshold)
            {
                //Do reload
                int copyLength = Math.min(mBufferWorkspaceLength, samples.length - samplesPointer);
                System.arraycopy(mBuffer, copyLength, mBuffer, 0, mBuffer.length - copyLength);
                System.arraycopy(samples, samplesPointer, mBuffer, mBuffer.length - copyLength, copyLength);
                samplesPointer += copyLength;
                mBufferPointer -= copyLength;
                mEqualizer.process(copyLength); //Equalize the newly added samples and optionally unwrap phases.
            }

            while(mBufferPointer < mBufferReloadThreshold)
            {
                mBufferPointer++;
                mSamplePoint--;
                mDebugSampleCount++;

                if(mSamplePoint < 1)
                {
                    mSymbolsSinceLastSync++;
                    mDebugSymbolCount++;

                    //Toggle assembling vs sync detection mode once the message framer stops assembling a message
//                    if(mSyncDetectionMode ^ mMessageFramer.isAssembling())
//                    {
//                        mSyncDetectionMode = !mSyncDetectionMode;
//
//                        if(!mSyncDetectionMode)
//                        {
//                            System.out.println("TODO: enable the sync detection resets ...********************");
////                            mSyncDetector.reset();
////                            mSyncDetectorLag1.reset();
////                            mSyncDetectorLag2.reset();
//                        }
//                    }

                    if(mSyncDetectionMode)
                    {
                        softSymbol = LinearInterpolator.calculate(mBuffer[mBufferPointer], mBuffer[mBufferPointer + 1], mSamplePoint);
                        float scorePrimary = mSyncDetector.process(softSymbol);

                        correctionCandidate = INVALID_SYNC_DETECTION;

                        if(mFineSync)
                        {
                            if(scorePrimary > SYNC_THRESHOLD_DETECTION)
                            {
                                correctionCandidate = optimize(0);
                            }
                        }
                        else
                        {
                            //Check for lagging sync pattern
                            float lag1 = (float)(mBufferPointer + mSamplePoint - mLaggingSyncOffset1);
                            float lag2 = (float)(mBufferPointer + mSamplePoint - mLaggingSyncOffset2);
                            int lagIntegral1 = (int)Math.floor(lag1);
                            int lagIntegral2 = (int)Math.floor(lag2);
                            float softSymbolLag1 = LinearInterpolator.calculate(mBuffer[lagIntegral1], mBuffer[lagIntegral1 + 1], lag1 - lagIntegral1);
                            float softSymbolLag2 = LinearInterpolator.calculate(mBuffer[lagIntegral2], mBuffer[lagIntegral2 + 1], lag2 - lagIntegral2);
                            float scoreLag1 = mSyncDetectorLag1.process(softSymbolLag1);
                            float scoreLag2 = mSyncDetectorLag2.process(softSymbolLag2);

                            if(mSymbolsSinceLastSync > 1)
                            {
                                if(scorePrimary > SYNC_THRESHOLD_DETECTION && scorePrimary > scoreLag1 && scorePrimary > scoreLag2)
                                {
                                    correctionCandidate = optimize(0.0f);
                                }

                                if(correctionCandidate == INVALID_SYNC_DETECTION && scoreLag1 > SYNC_THRESHOLD_DETECTION && scoreLag1 > scoreLag2)
                                {
                                    correctionCandidate = optimize(-mLaggingSyncOffset1);
                                }

                                if(correctionCandidate == INVALID_SYNC_DETECTION && scoreLag2 > SYNC_THRESHOLD_DETECTION)
                                {
                                    correctionCandidate = optimize(-mLaggingSyncOffset2);
                                }
                            }
                        }

                        if(correctionCandidate.hasValidTiming())
                        {
                            validateNID(correctionCandidate);

                            int debugThreshold = 7_982_000;

                            if(correctionCandidate.hasValidNID())
                            {
                                //Apply candidate correction values to timing and equalizer.
                                apply(correctionCandidate);

                                //Overwrite the sync symbols in the symbol delay line
                                for(Dibit syncSymbol: SYNC_PATTERN_DIBITS)
                                {
                                    mSymbolDelayLine.insert(syncSymbol);
                                }

                                System.out.println(correctionCandidate + " Samples [" + mDebugSampleCount +
                                        "] Symbols [" + mDebugSymbolCount +
                                        "] NAC [" + correctionCandidate.getNAC() +
                                        "] DUID [" + correctionCandidate.getDataUnitID() + "]");

                                if(mDebugSampleCount > debugThreshold)
                                {
                                    visualizeSyncDetect(0, true, "valid NID");
                                }

                                mFineSync = true;
                                mMessageFramer.syncDetected(correctionCandidate.getNAC(), correctionCandidate.getDataUnitID());
                            }
                            else
                            {
                                System.out.println(correctionCandidate + " Samples [" + mDebugSampleCount +
                                        "] Symbols [" + mDebugSymbolCount + "]");

                                if(mDebugSampleCount > debugThreshold)
                                {
                                    visualizeSyncDetect(0, true, "*** JUNK NID DETECTED ***");
                                }
                            }

                            mPreviousMessageSymbolLength = mSymbolsSinceLastSync;
                            mPreviousDataUnitID = correctionCandidate.getDataUnitID();
                            mSymbolsSinceLastSync = 33;
                        }
                    }

                    if(mFineSync)
                    {
                        if(mSymbolsSinceLastSync > 180 && mMessageFramer.isAssembling() && mCurrentDataUnitID != mMessageFramer.getAssemblingDUID())
                        {
                            //                        System.out.println("*** MESSAGE ASSEMBLER PIVOT TO [" + mMessageFramer.getAssemblingDUID() + "] at Elapsed [" + mSymbolsSinceLastSync + "]");
                            mCurrentDataUnitID = mMessageFramer.getAssemblingDUID();
                        }

                        if(mSymbolsSinceLastSync > (mCurrentDataUnitID.getElapsedDibitLength()))
                        {
                            if(mMessageFramer.isAssembling())
                            {
                                //                            System.out.println("Elapsed [" + mSymbolsSinceLastSync + "] symbols exceeds expected [" +
                                //                                    mCurrentDataUnitID.getElapsedDibitLength() + "] current [" + mCurrentDataUnitID +
                                //                                    "] changing to [" + mMessageFramer.getAssemblingDUID() +
                                //                                    "] new elapsed [" + mMessageFramer.getAssemblingDUID().getElapsedDibitLength() + "]");
                                mCurrentDataUnitID = mMessageFramer.getAssemblingDUID();
                            }
                            else if(mSymbolsSinceLastSync > (mCurrentDataUnitID.getElapsedDibitLength()))
                            {
                                //                            System.out.println("Elapsed [" + mSymbolsSinceLastSync + "] symbols exceeds expected [" +
                                //                                    mCurrentDataUnitID.getElapsedDibitLength() + "] for [" + mCurrentDataUnitID +
                                //                                    "] message framer is no longer assembling #### SETTING FINE SYNC TO FALSE #### At Symbol [" + mDebugSymbolCount + "]");
                                mFineSync = false;
                            }
                        }
                    }

                    //Calculate next symbol and get dibit from the delay line and feed the message framer and dibit assembler.
                    delayedSymbol = mSymbolDelayLine.insert(toSymbol(getProjectedSoftSymbol()));
                    mMessageFramer.receive(delayedSymbol);
                    mDibitAssembler.receive(delayedSymbol);

                    //Add another symbol's worth of samples to the counter
                    mSamplePoint += mObservedSamplesPerSymbol;
                }
            }
        }

        return debugReturn;
    }

    /**
     * Applies the symbol timing and equalizer correction settings.
     * @param correction to apply
     */
    private void apply(Correction correction)
    {
        mSamplePoint += correction.getTiming();

        while(mSamplePoint < 0)
        {
            mSamplePoint++;
            mBufferPointer--;
        }

        while(mSamplePoint > 1)
        {
            mSamplePoint--;
            mBufferPointer++;
        }

        mEqualizer.apply(correction);
    }

    /**
     * Calculate the maximum delay soft symbol to feed the message framer and dibit assembler.
     * @return soft symbol.
     */
    private float getProjectedSoftSymbol()
    {
        //Calculate the framer symbol 33 dibits ahead of the sync pointer that represents the NID symbol
        double offset = mBufferPointer + mSamplePoint + mSyncOffset;
        int integral = (int)Math.floor(offset);
        offset -= integral;
        return LinearInterpolator.calculate(mBuffer[integral], mBuffer[integral + 1], offset);
    }

    /**
     * Indicates if the samples in the sample buffer that contain a detected sync pattern have a standard deviation
     * that is lower than the expected sample to sample deviation for a modulated signal.  False detects against
     * noise tend to have a standard deviation that is 2x the expected value.
     * @param offset to the sample representing the final symbol in the detected sync pattern.
     * @return true if the standard deviation is more than expected.
     */
    private boolean isNoisy(double offset)
    {
        StandardDeviation standardDeviation = new StandardDeviation();
        int start = (int)Math.floor(offset - (23 * mObservedSamplesPerSymbol));
        int end = (int)Math.ceil(offset);
        end = Math.min(end, mBuffer.length - 1);

        for(int i = start; i < end; i++)
        {
            standardDeviation.increment(mBuffer[i] - mBuffer[i + 1]);
        }

        return standardDeviation.getResult() > mNoiseStandardDeviationThreshold;
    }

    /**
     * On sync detection, calculates the optimal timing adjustment that achieves the highest sync correlation score or
     * returns an INVALID_SYNC_DETECTION sentinel value if the correlation score doesn't exceed the threshold.
     * @param additionalOffset from current mBufferPointer and mSamplePoint.  This can be zero offset for the primary
     * sync detector or a lagging offset for the lagging sync detectors.
     * @return optimized timing adjustment or NO_OPTIMIZATION sentinel value.
     */
    private Correction optimize(double additionalOffset)
    {
        //Offset is the start of the first sample of the first symbol of the sync pattern calculated from the current
        //buffer pointer and sample point which should be the final sample of the final symbol of the detected sync.
        double offset = mBufferPointer + mSamplePoint + additionalOffset;

        //Reject any sync detections where the sample:sample standard deviation exceeds the noise threshold.
        if(isNoisy(offset))
        {
            return INVALID_SYNC_DETECTION;
        }

        //Find the optimal symbol timing
        double stepSize = mSamplesPerSymbol / (mFineSync ? 16.0 : 8.0); //Start at 1/8th for coarse & 1/16th for fine
        double stepSizeMin = mOptimizeFineIncrement;
        double adjustment = 0.0;

        //In coarse sync mode, constrain max adjustment to half a symbol period so that a lagging sync detect doesn't
        //preempt a primary sync detect prematurely and cause a significant timing correction.  So, we constrain the
        //max correction to a half symbol period for coarse sync mode.
        double adjustmentMax = mFineSync ? mSamplesPerSymbol : (mSamplesPerSymbol / 2.0);
        double candidate = offset;

        float scoreCenter = score(candidate, mObservedSamplesPerSymbol);

        candidate = offset - stepSize;
        float scoreLeft = score(candidate, mObservedSamplesPerSymbol);

        candidate = offset + stepSize;
        float scoreRight = score(candidate, mObservedSamplesPerSymbol);

        while(stepSize > stepSizeMin && Math.abs(adjustment) <= adjustmentMax)
        {
            if(scoreLeft > scoreRight && scoreLeft > scoreCenter)
            {
                adjustment -= stepSize;
                scoreRight = scoreCenter;
                scoreCenter = scoreLeft;

                candidate = offset + adjustment - stepSize;
                scoreLeft = score(candidate, mObservedSamplesPerSymbol);
            }
            else if(scoreRight > scoreLeft && scoreRight > scoreCenter)
            {
                adjustment += stepSize;
                scoreLeft = scoreCenter;
                scoreCenter = scoreRight;

                candidate = offset + adjustment + stepSize;
                scoreRight = score(candidate, mObservedSamplesPerSymbol);
            }
            else
            {
                stepSize *= 0.5f;

                if(stepSize > stepSizeMin)
                {
                    candidate = offset + adjustment - stepSize;
                    scoreLeft = score(candidate, mObservedSamplesPerSymbol);

                    candidate = offset + adjustment + stepSize;
                    scoreRight = score(candidate, mObservedSamplesPerSymbol);
                }
            }
        }

        //If we didn't find an optimal correlation score above the 95 threshold, return a false sync.
        if(scoreCenter < SYNC_THRESHOLD_OPTIMIZED)
        {
            return INVALID_SYNC_DETECTION;
        }

//        System.out.println("\t\t$$$ Adjustment: " + adjustment + " Additional: " + additionalOffset + " Total: " + (adjustment + additionalOffset));

        adjustment += additionalOffset;

        return mEqualizer.getCorrection(adjustment);
    }

    /**
     * Calculates the sync correlation score at the specified offset and samples per symbol interval.
     * @param offset to the final symbol in the soft symbol buffer
     * @param samplesPerSymbol spacing to test for.
     * @return correlation score.
     */
    public float score(double offset, double samplesPerSymbol)
    {
        int maxPointer = mBuffer.length - 1;
        float softSymbol;

        double pointer = offset - (samplesPerSymbol * 23.0);
        int bufferPointer = (int)Math.floor(pointer);
        double fractional = pointer - bufferPointer;

        float score = 0;

        for(int x = 0; x < 24; x++)
        {
            if(bufferPointer < maxPointer)
            {
                softSymbol = LinearInterpolator.calculate(mBuffer[bufferPointer], mBuffer[bufferPointer + 1], fractional);
            }
            else
            {
                softSymbol = 0.0f;
            }

            score += softSymbol * SYNC_PATTERN_SYMBOLS[x];
            pointer += samplesPerSymbol;
            bufferPointer = (int)Math.floor(pointer);
            fractional = pointer - bufferPointer;
        }

        return score;
    }

    /**
     * Debug method to visualize the contents of the sync detection for the samples and symbols and symbol timing.
     * @param score from the sync detector
     * @param primary sync detector (true) or secondary (false)
     */
    public void visualizeSyncDetect(float score, boolean primary, String tag)
    {
        //This will block until the viewer is constructed and showing.
        if(mSyncResultsViewer == null)
        {
            mSyncResultsViewer = new SyncResultsViewer();
        }

        int offset = 3;
        int length = (int)Math.ceil(mObservedSamplesPerSymbol * 23) + (2 * offset);
        int end = mBufferPointer + offset;
        int start = end - length;
        float[] symbols = new float[24];
        float[] samples = Arrays.copyOfRange(mBuffer, start, end);

        float[] intervals = new float[24];

        int adjust = mBufferPointer - length + offset;
        double pointer = mBufferPointer + mSamplePoint - adjust;

        double symbolPointer = mBufferPointer + mSamplePoint - (mObservedSamplesPerSymbol * 23);
        int symbolIntegral = (int)Math.floor(symbolPointer);
        double mu = symbolPointer - symbolIntegral;

        for(int x = 0; x < 24; x++)
        {
            symbols[x] = LinearInterpolator.calculate(mBuffer[symbolIntegral], mBuffer[symbolIntegral + 1], mu);
            symbolPointer += mObservedSamplesPerSymbol;
            symbolIntegral = (int)Math.floor(symbolPointer);
            mu = symbolPointer - symbolIntegral;
        }

        for(int x = 23; x >= 0; x--)
        {
            intervals[x] = (float)pointer;
            pointer -= mObservedSamplesPerSymbol;
        }

        CountDownLatch countDownLatch = new CountDownLatch(1);
        mSyncResultsViewer.receive(symbols, SYNC_PATTERN_SYMBOLS, samples, intervals, mEqualizer.balance(), mEqualizer.gain(),
                "Score: " + score + (primary ? " PRIMARY " : " SECONDARY ") + (mFineSync ? "FINE " : "COARSE ") +
                        " EQ-B:" + mEqualizer.balance() + " EQ-G:" + mEqualizer.gain() + " " + tag, countDownLatch);

        try
        {
            countDownLatch.await();
        }
        catch(InterruptedException e)
        {
            throw new RuntimeException(e);
        }
    }

    /**
     * Registers the listener to receive demodulated bit stream buffers.
     * @param listener to register
     */
    public void setBufferListener(Listener<ByteBuffer> listener)
    {
        mDibitAssembler.setBufferListener(listener);
    }

    /**
     * Indicates if there is a registered buffer listener
     */
    public boolean hasBufferListener()
    {
        return mDibitAssembler.hasBufferListeners();
    }

    /**
     * Sets or updates the samples per symbol
     * @param samplesPerSymbol to apply.
     */
    public void setSamplesPerSymbol(float samplesPerSymbol)
    {
        mSamplesPerSymbol = samplesPerSymbol;
        mOptimizeFineIncrement = samplesPerSymbol * .004f; //Adjust at .4%
        mObservedSamplesPerSymbol = samplesPerSymbol;
        mSyncOffset = mObservedSamplesPerSymbol * DIBIT_LENGTH_NID;
        mSyncOffsetForReload = (int)Math.ceil(mSyncOffset);
        mNoiseStandardDeviationThreshold = Dibit.D01_PLUS_3.getIdealPhase() * 2 / mSamplesPerSymbol * 1.2; //120% of optimal
        mSamplePoint = samplesPerSymbol;
        mLaggingSyncOffset1 = samplesPerSymbol / 3;
        mLaggingSyncOffset2 = mLaggingSyncOffset1 * 2;
        int bufferLength = mBufferWorkspaceLength + (int)(Math.ceil((DIBIT_LENGTH_SYNC + DIBIT_LENGTH_NID + 2) * samplesPerSymbol));
        mBuffer = new float[bufferLength];
//        mBufferLoadPointer = (int)Math.ceil(DIBIT_LENGTH_PROTECTED_REGION * samplesPerSymbol);
        mBufferReloadThreshold = mBuffer.length - (int)Math.ceil(samplesPerSymbol * (DIBIT_LENGTH_NID + 1));
        mBufferPointer = mBufferReloadThreshold;
    }

    /**
     * Resamples the NID dibits using the supplied adjustment.  If the NID dibits pass error correction, notifies the
     * message framer with the extracted NAC and DUID values and overwrites the NID dibit delay buffer with the
     * static sync symbols and the resampled NID dibits so that the message framer and dibit assembler receive the
     * optimally sampled sync and NID dibit sequences.
     * @param correction (timing) to apply when resampling the NID.
     * @return true if a valid NID sequence was sampled to indicate that the correction value should be permanent
     */
    private Correction validateNID(Correction correction)
    {
        //Current sample point + adjustment is pointing to the final sync symbol.  Resample the NID from here.
        double pointer = mBufferPointer + mSamplePoint + correction.getTiming() + mObservedSamplesPerSymbol;
        double fractional;
        int integral;
        float softSymbol = 0;
        Dibit symbol;

        //Capture just the 63-bit BCH protected NID codeword including the 64th parity bit which we ignore.
        CorrectedBinaryMessage candidateNID = new CorrectedBinaryMessage(64);
        Dibit[] resampledNIDSymbols = new Dibit[33];

        for(int x = 0; x < DIBIT_LENGTH_NID; x++)
        {
            integral = (int) Math.floor(pointer);
            fractional = pointer - integral;
            softSymbol = LinearInterpolator.calculate(mBuffer[integral], mBuffer[integral + 1], fractional);

//            softSymbol = (softSymbol + correction.getBalance()) * correction.getGain();
            symbol = toSymbol(softSymbol);
            resampledNIDSymbols[x] = symbol;

            //Skip the status symbol at dibit 11
            if(x != 11)
            {
                candidateNID.add(symbol.getBit1(), symbol.getBit2());
            }

            pointer += mObservedSamplesPerSymbol;
        }

        int trackedNAC = mNACTracker.getTrackedNAC();
        mBCHDecoder.decode(candidateNID, trackedNAC);

        //If error correction fails, return the original correction candidate
        if(candidateNID.getCorrectedBitCount() < 0)
        {
            return correction;
        }

        int nac = candidateNID.getInt(NAC_FIELD);
        P25P1DataUnitID duid = P25P1DataUnitID.fromValue(candidateNID.getInt(DUID_FIELD));

        //The BCH decoder can over-correct the NID and produce an invalid NAC.  Compare it against the tracked NID to
        //flag it as invalid NID when this happens.  The NAC tracker will give us a value of 0 until it has enough
        //observations of a valid NID value.
        mNACTracker.track(nac);

        if(trackedNAC > 0 && trackedNAC != nac)
        {
            return correction;
        }

        correction.setNID(nac, duid);
        return correction;
    }

    private static void log(CorrectedBinaryMessage a, P25P1DataUnitID duid, int nac, boolean corrected)
    {
        StringBuilder sb = new StringBuilder();
        sb.append("\tNID: ").append(a);
        sb.append(" NAC:").append(nac);
        sb.append(" DUID:").append(duid);
        sb.append(" CORRECTED:").append(corrected);

        if(corrected)
        {
            sb.append(" BITS:").append(a.getCorrectedBitCount());
        }
        else
        {
            sb.append("\t\t** ERROR **");
        }

        System.out.println(sb);
    }

    /**
     * Decodes the sample value to determine the correct QPSK quadrant and maps the value to a Dibit symbol.
     * @param sample in radians.
     * @return symbol decision.
     */
    private static Dibit toSymbol(float sample)
    {
        if(sample > 0)
        {
            return sample > SOFT_SYMBOL_QUADRANT_BOUNDARY ? Dibit.D01_PLUS_3 : Dibit.D00_PLUS_1;
        }
        else
        {
            return sample < -SOFT_SYMBOL_QUADRANT_BOUNDARY ? Dibit.D11_MINUS_3 : Dibit.D10_MINUS_1;
        }
    }

    /**
     * Symbol timing and equalizer correction candidate settings
     */
    public static class Correction
    {
        private double mTiming;
        private float mBalance;
        private float mGain;
        private int mNAC;
        private P25P1DataUnitID mDataUnitID;

        /**
         * Constructs an instance
         * @param timing correction for symbol sampling
         * @param balance correction for equalizer
         * @param gain correction for equalizer
         */
        public Correction(double timing, float balance, float gain)
        {
            mTiming = timing;
            mBalance = balance;
            mGain = gain;
        }

        @Override
        public String toString()
        {
            StringBuilder sb = new StringBuilder();

            if(hasValidTiming())
            {
                sb.append("Correction - Timing [").append(mTiming);
                sb.append("] Eq Balance [").append(mBalance);
                sb.append("] Eq Gain [").append(mGain);

                if(hasValidNID())
                {
                    sb.append("] NAC [").append(mNAC);
                    sb.append("] DUID [").append(mDataUnitID.toString());
                }
                else
                {
                    sb.append("] INVALID NID");
                }
            }
            else
            {
                sb.append("Correction - INVALID TIMING");
            }
            return sb.toString();
        }

        /**
         * Timing correction value
         * @return correction
         */
        public double getTiming()
        {
            return mTiming;
        }

        /**
         * Equalizer balance correction value.
         * @return correction
         */
        public float getBalance()
        {
            return mBalance;
        }

        /**
         * Equalizer gain correction value
         * @return correction.
         */
        public float getGain()
        {
            return mGain;
        }

        /**
         * Sets the detected data unit ID.
         * @param dataUnitID that was detected.
         */
        public void setNID(int nac, P25P1DataUnitID dataUnitID)
        {
            mNAC = nac;
            mDataUnitID = dataUnitID;
        }

        /**
         * Detected data unit ID
         * @return data unit ID
         */
        public P25P1DataUnitID getDataUnitID()
        {
            return mDataUnitID;
        }

        public int getNAC()
        {
            return mNAC;
        }

        /**
         * Indicates if this correction has valid timing and equalizer information.
         * @return true if the timing correction is not the default (ie no timing) double max value.
         */
        public boolean hasValidTiming()
        {
            return getTiming() != Double.MAX_VALUE;
        }

        /**
         * Indicates if this correction is valid and has a non-null detected data unit ID from the NID.
         * @return true if valid.
         */
        public boolean hasValidNID()
        {
            return mDataUnitID != null;
        }
    };

    /**
     * Base equalizer class
     */
    abstract class Equalizer
    {
        protected boolean mInitialized = false;
        protected float mBalance = 0.0f;
        protected float mGain = 1.0f;

        /**
         * Process the samples buffer to correct the balance and gain of the (copyLength) newest samples in the buffer.
         * @param copyLength quantity of newly added samples in the buffer
         */
        public abstract void process(int copyLength);

        /**
         * Indicates if this equalizer is initialized, meaning it has processed at least 1x sync detection to establish
         * initial gain and balance values.
         * @return true if initialized.
         */
        public boolean isInitialized()
        {
            return mInitialized;
        }

        /**
         * Current balance value
         * @return balance value.
         */
        public float balance()
        {
            return mBalance;
        }

        /**
         * Current gain value.
         * @return gain value
         */
        public float gain()
        {
            return mGain;
        }

        /**
         * Applies equalizer correction settings.
         * @param correction settings to apply
         */
        public void apply(Correction correction)
        {
            //Re-initialize the equalizer any time the balance correction value exceeds the threshold.
            if(mInitialized && Math.abs(correction.getBalance()) > EQUALIZER_RECALIBRATE_THRESHOLD)
            {
                System.out.println("Equalizer - Excessive Balance Detected - Balance: " + DF.format(correction.getBalance()) + " Gain: " + DF.format(correction.getGain()));
                mInitialized = false;
            }

            //Limit equalizer adjustments using a control loop at each sync detect after the initial equalizer setup or
            //apply as new settings on initial sync detect or any time an excess balance correction is supplied.
            if(mInitialized)
            {
                mBalance += (correction.getBalance() * EQUALIZER_LOOP_GAIN);
                mGain += (correction.getGain() * EQUALIZER_LOOP_GAIN);
            }
            else
            {
                mBalance += correction.getBalance();
                mGain += correction.getGain();
            }

            //Constrain balance to +/- PI/4
            mBalance = Math.min(mBalance, EQUALIZER_MAXIMUM_BALANCE);
            mBalance = Math.max(mBalance, -EQUALIZER_MAXIMUM_BALANCE);

            //Constrain gain between 1.0f and 1.35f
            mGain = Math.min(mGain, EQUALIZER_MAXIMUM_GAIN);
            mGain = Math.max(mGain, 1.0f);

            if(!mInitialized)
            {
                //Apply the initial gain settings to the remaining buffer samples to affect subsequent sampling
                for(int x = 0; x < mBuffer.length; x++)
                {
                    mBuffer[x] = (mBuffer[x] + mBalance) * mGain;
                }

                mInitialized = true;
            }
        }

        /**
         * Calculates a correction update for the equalizer balance and gain when the sync pattern is detected in the
         * sample buffer, using the supplied timing correction argument.  Resamples the sync symbols and compares each
         * soft symbol to the ideal symbol phase to develop average error measurements for balance and gain.  On initial
         * sync detection, the equalizer settings are applied to the samples in the buffer allowing the symbols to be
         * resampled during coarse sync acquisition.
         */
        public Correction getCorrection(double timingCorrection)
        {
            double resampleStart = mBufferPointer + mSamplePoint + timingCorrection;
            int resampleStartIntegral = (int)Math.floor(resampleStart);
            float symbol = SYNC_PATTERN_SYMBOLS[23];
            float resampledSoftSymbol = LinearInterpolator.calculate(mBuffer[resampleStartIntegral],
                    mBuffer[resampleStartIntegral + 1], resampleStart - resampleStartIntegral);
            float balancePlus3Symbols = resampledSoftSymbol - symbol;
            float balanceMinus3Symbols = 0;
            float gainAccumulator = Math.abs(symbol) - Math.abs(resampledSoftSymbol);

            resampleStart -= (23 * mObservedSamplesPerSymbol);
            resampleStartIntegral = (int)Math.floor(resampleStart);

            for(int x = 0; x < 23; x++)
            {
                if(resampleStartIntegral >= 0)
                {
                    symbol = SYNC_PATTERN_SYMBOLS[x];
                    resampledSoftSymbol = LinearInterpolator.calculate(mBuffer[resampleStartIntegral],
                            mBuffer[resampleStartIntegral + 1], resampleStart - resampleStartIntegral);

                    if(symbol > 0)
                    {
                        balancePlus3Symbols += (resampledSoftSymbol - symbol);
                    }
                    else
                    {
                        balanceMinus3Symbols += (resampledSoftSymbol - symbol);
                    }

                    gainAccumulator += Math.abs(symbol) - Math.abs(resampledSoftSymbol);
                }

                resampleStart += mObservedSamplesPerSymbol;
                resampleStartIntegral = (int)Math.floor(resampleStart);
            }

            balancePlus3Symbols /= -11.0f; //There are 11x Plus 3 and 13x Minus 3 symbols in the sync pattern.
            balanceMinus3Symbols /= -13.0f;
            float balanceAverage = (balancePlus3Symbols + balanceMinus3Symbols) / 2f;
//            System.out.println("Balance [" + balanceAverage + "] Plus3 [" + balancePlus3Symbols + "] Minus3 [" + balanceMinus3Symbols + "]");
            gainAccumulator /= (24.0f * Dibit.D01_PLUS_3.getIdealPhase());

            return new Correction(timingCorrection, balanceAverage, gainAccumulator);
        }
    }

    /**
     * C4FM equalizer implementation
     */
    class C4FMEqualizer extends Equalizer
    {
        @Override
        public void process(int copyLength)
        {
            for(int x = mBuffer.length - copyLength; x < mBuffer.length; x++)
            {
                //Unwrap phases
                if(mBuffer[x - 1] > 1.5f && mBuffer[x] < -1.5f)
                {
                    mBuffer[x] += TWO_PI;
                }
                else if(mBuffer[x - 1] < -1.5f && mBuffer[x] > 1.5f)
                {
                    mBuffer[x] -= TWO_PI;
                }

                //Apply equalizer adjustments
                mBuffer[x] += mBalance;
                mBuffer[x] *= mGain;

                //Allow the equalized buffer samples to exceed PI (3.14) by a small factor (3.5) to ensure the optimize
                // and equalizer update functions work correctly.  The toSymbol() method will map any symbols that
                // exceed +/- PI into the correct quadrant.
                mBuffer[x] = Math.min(mBuffer[x], SOFT_SYMBOL_MAX_POSITIVE_PHASE);
                mBuffer[x] = Math.max(mBuffer[x], SOFT_SYMBOL_MAX_NEGATIVE_PHASE);
            }
        }
    }

    /**
     * CQPSK/LSM equalizer implementation
     */
    class CQPSKEqualizer extends Equalizer
    {
        @Override
        public void process(int copyLength)
        {
            for(int x = mBuffer.length - copyLength; x < mBuffer.length; x++)
            {
                //Apply equalizer adjustments
                mBuffer[x] += mBalance;
                mBuffer[x] *= mGain;

                //Allow the equalized buffer samples to exceed PI (3.14) by a small factor (3.5) to ensure the optimize
                // and equalizer update functions work correctly.  The toSymbol() method will map any symbols that
                // exceed +/- PI into the correct quadrant.
                mBuffer[x] = Math.min(mBuffer[x], SOFT_SYMBOL_MAX_POSITIVE_PHASE);
                mBuffer[x] = Math.max(mBuffer[x], SOFT_SYMBOL_MAX_NEGATIVE_PHASE);
            }
        }
    }
}
