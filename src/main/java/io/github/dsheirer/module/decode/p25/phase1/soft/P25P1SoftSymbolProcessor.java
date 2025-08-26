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
    private static final int DIBIT_LENGTH_PROTECTED_REGION = 26; //Sync (24) plus 2
    private static final int DIBIT_LENGTH_WORKSPACE_LENGTH = 25; //This can be adjusted for efficiency
    private static final int DIBIT_LENGTH_NID = 33; //32 dibits (64 bits) plus 1 status symbol dibit
    private static final int DIBIT_LENGTH_SYNC = 24;
    private static final int DIBIT_LENGTH_BUFFER = DIBIT_LENGTH_PROTECTED_REGION + DIBIT_LENGTH_WORKSPACE_LENGTH + DIBIT_LENGTH_NID;
    private static final int DIBIT_LENGTH_MIN_FOR_CHECK_SYNC = 38; //TDU is 63 dibits - 24 sync = 39 - 1 for possible overlap
    private static final double NO_OPTIMIZATION = Double.MAX_VALUE;
    private static final float SOFT_SYMBOL_MAX_POSITIVE = Dibit.D01_PLUS_3.getIdealPhase();
    private static final float SOFT_SYMBOL_MAX_NEGATIVE = Dibit.D11_MINUS_3.getIdealPhase();
    private static final float SOFT_SYMBOL_MAX_POSITIVE_PHASE = 3.5f;
    private static final float SOFT_SYMBOL_MAX_NEGATIVE_PHASE = -3.5f;
    private static final float SOFT_SYMBOL_QUADRANT_BOUNDARY = (float)(Math.PI / 2.0);
    private static final float SYNC_THRESHOLD_DETECTION = 60;
    private static final float SYNC_THRESHOLD_OPTIMIZED = 80;
    private static final float SYNC_THRESHOLD_EQUALIZED = 100;
    private static final float EQUALIZER_LOOP_GAIN = 0.1f;
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
    private DibitDelayLine mNidDelayLine = new DibitDelayLine(DIBIT_LENGTH_SYNC + DIBIT_LENGTH_NID);
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
    private int mBufferPointer;
    private int mBufferWorkspaceLength;
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

        while(samplesPointer < samples.length)
        {
            if(mBufferLoadPointer == mBuffer.length)
            {
                System.arraycopy(mBuffer, mBufferWorkspaceLength, mBuffer, 0, mBuffer.length - mBufferWorkspaceLength);
                mBufferLoadPointer -= mBufferWorkspaceLength;
                mBufferPointer -= mBufferWorkspaceLength;
            }

            int copyLength = Math.min(mBuffer.length - mBufferLoadPointer, samples.length - samplesPointer);
            System.arraycopy(samples, samplesPointer, mBuffer, mBufferLoadPointer, copyLength);
            samplesPointer += copyLength;
            mBufferLoadPointer += copyLength;

            //Equalize samples and optionally unwrap phases.
            mEqualizer.process(copyLength);

            float softSymbol;
            Dibit delayedSymbol;
            double adjustment;
            String tag = null;

            while(mBufferPointer < (mBufferLoadPointer - mSyncOffsetForReload))
            {
                mBufferPointer++;
                mSamplePoint--;
                mDebugSampleCount++;

                if(mDebugSampleCount > 3_064_000 && mDebugSymbolCount % 25 == 0)
                {
                    visualizeSyncDetect(0, false, "Symbols: " + mDebugSymbolCount + " Samples: " + mDebugSampleCount);
                }

                if(mSamplePoint < 1)
                {
                    mSymbolsSinceLastSync++;
                    mDebugSymbolCount++;

                    //Toggle assembling vs sync detection mode once the message framer stops assembling a message
                    if(mSyncDetectionMode ^ mMessageFramer.isAssembling())
                    {
                        mSyncDetectionMode = !mSyncDetectionMode;

                        if(!mSyncDetectionMode)
                        {
                            System.out.println("TODO: enable the sync detection resets ...********************");
//                            mSyncDetector.reset();
//                            mSyncDetectorLag1.reset();
//                            mSyncDetectorLag2.reset();
                        }
                    }

                    if(mSyncDetectionMode)
                    {
                        softSymbol = LinearInterpolator.calculate(mBuffer[mBufferPointer], mBuffer[mBufferPointer + 1], mSamplePoint);
                        float scorePrimary = mSyncDetector.process(softSymbol);

                        adjustment = NO_OPTIMIZATION;

                        if(mFineSync)
                        {
                            if(scorePrimary > SYNC_THRESHOLD_DETECTION)
                            {
                                tag = "PRI:" + scorePrimary;
                                adjustment = optimize(0, tag);
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
                            tag = "PRI:" + scorePrimary + " LAG1:" + scoreLag1 + " LAG2:" + scoreLag2;

                            if(mSymbolsSinceLastSync > 1)
                            {
                                if(scorePrimary > SYNC_THRESHOLD_DETECTION && scorePrimary > scoreLag1 && scorePrimary > scoreLag2)
                                {
                                    tag = "PRI:" + scorePrimary;
                                    adjustment = optimize(0.0f, tag);
                                }

                                if(adjustment == NO_OPTIMIZATION && scoreLag1 > SYNC_THRESHOLD_DETECTION && scoreLag1 > scoreLag2)
                                {
                                    tag = "LAG1:" + scoreLag1;
                                    adjustment = optimize(-mLaggingSyncOffset1, tag);
                                }

                                if(adjustment == NO_OPTIMIZATION && scoreLag2 > SYNC_THRESHOLD_DETECTION)
                                {
                                    tag = "LAG2:" + scoreLag2;
                                    adjustment = optimize(-mLaggingSyncOffset2, tag);
                                }
                            }
                        }

                        if(adjustment < NO_OPTIMIZATION)
                        {
                            //Apply the candidate adjustment if it produces a valid NID
                            boolean validNID = validateNID(adjustment);

                            if(validNID)
                            {
                                mSamplePoint += adjustment;

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

                                mFineSync = true;
                            }

                            mPreviousMessageSymbolLength = mSymbolsSinceLastSync;
                            mPreviousDataUnitID = mCurrentDataUnitID;

                            mSymbolsSinceLastSync = 33;
                            System.out.println("\nSYNC PRIMARY   FINE - ELAPSED [" +
                                    mPreviousMessageSymbolLength + "] FOR [" + mPreviousDataUnitID +
                                    "] SAMPLES [" + mDebugSampleCount + "] SYMBOLS [" + mDebugSymbolCount + "]");
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
                    delayedSymbol = mNidDelayLine.insert(toSymbol(getProjectedSoftSymbol()));
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
     * returns a NO_OPTIMIZATION sentinel value if the correlation score doesn't exceed the threshold.
     * @param additionalOffset from current mBufferPointer and mSamplePoint.  This can be zero offset for the primary
     * sync detector or a lagging offset for the lagging sync detectors.
     * @return optimized timing adjustment or NO_OPTIMIZATION sentinel value.
     */
    private Correction optimize(double additionalOffset, String tag)
    {
        //Offset is the start of the first sample of the first symbol of the sync pattern calculated from the current
        //buffer pointer and sample point which should be the final sample of the final symbol of the detected sync.
        double offset = mBufferPointer + mSamplePoint + additionalOffset;

        //Reject any sync detections where the sample:sample standard deviation exceeds the noise threshold.
        if(isNoisy(offset))
        {
            return NO_OPTIMIZATION;
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
            return NO_OPTIMIZATION;
        }


        System.out.println("\t\t$$$ Adjustment: " + adjustment + " Additional: " + additionalOffset + " Total: " + (adjustment + additionalOffset));

        adjustment += additionalOffset;

        return mEqualizer.update(adjustment);
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
        mBufferWorkspaceLength = (int)Math.ceil(DIBIT_LENGTH_WORKSPACE_LENGTH * samplesPerSymbol);
        int bufferLength = (int)(Math.ceil(DIBIT_LENGTH_BUFFER * samplesPerSymbol));
        mBuffer = new float[bufferLength];
        mBufferLoadPointer = (int)Math.ceil(DIBIT_LENGTH_PROTECTED_REGION * samplesPerSymbol);
        mBufferPointer = mBufferLoadPointer;
    }

    /**
     * Resamples the NID dibits using the supplied adjustment.  If the NID dibits pass error correction, notifies the
     * message framer with the extracted NAC and DUID values and overwrites the NID dibit delay buffer with the
     * static sync symbols and the resampled NID dibits so that the message framer and dibit assembler receive the
     * optimally sampled sync and NID dibit sequences.
     * @param adjustment to apply when resampling the NID.
     * @return true if a valid NID sequence was sampled to indicate that the correction value should be permanent
     */
    private boolean validateNID(double adjustment)
    {
        //Current sample point + adjustment is pointing to the final sync symbol.  Resample the NID from here.
        double pointer = mBufferPointer + mSamplePoint + adjustment + mObservedSamplesPerSymbol;
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

        //A negative corrected bit count indicates failed to correct the NID message.
        boolean validNID = candidateNID.getCorrectedBitCount() >= 0;

        int nac = 0;

        if(validNID)
        {
            nac = candidateNID.getInt(NAC_FIELD);
            //The BCH decoder can over-correct the NID and produce an invalid NAC.  Compare it against the tracked NID to
            //flag it as invalid NID when this happens.  The NAC tracker will give us a value of 0 until it has enough
            //observations of a valid NID value.
            if(trackedNAC > 0 && trackedNAC != nac)
            {
                validNID = false;
            }
            else
            {
                mNACTracker.track(nac);
            }
        }

        if(validNID)
        {
            //If there is an adjustment, update dibit delay line with actual sync symbols and resampled NID symbols.
            if(adjustment != 0)
            {
                for(Dibit syncSymbol: SYNC_PATTERN_DIBITS)
                {
                    mNidDelayLine.insert(syncSymbol);
                }

                //Insert all but the last resampled symbol.
                for(int x = 0; x < resampledNIDSymbols.length - 1; x++)
                {
                    mNidDelayLine.insert(resampledNIDSymbols[x]);
                }
            }

            mCurrentDataUnitID = P25P1DataUnitID.fromValue(candidateNID.getInt(DUID_FIELD));

            if(!mCurrentDataUnitID.isValidatedElapsedDibitLengthDUID())
            {
                LOGGER.warn("############ THIS DATA UNIT ID HAS NOT BEEN VALIDATED FOR ELAPSED DIBIT COUNT: " + mCurrentDataUnitID);
            }

            //        log(nidMessage, mCurrentDataUnitID, nac, validNID);
            System.out.println("\tValid NID Detected for NAC: " + nac + " DUID: " + mCurrentDataUnitID);
            //Update the NAC tracker with the observed, correctly decoded NAC value.
            mFineSync = true;
            mMessageFramer.syncDetected(nac, mCurrentDataUnitID);
        }
        else
        {
            //Set the DUID to TDU to cause disable of fine sync at earliest message length.
            mCurrentDataUnitID = P25P1DataUnitID.TERMINATOR_DATA_UNIT;
            mMessageFramer.syncDetectedInvalidNID(trackedNAC);
        }

        return validNID;
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
     * Correction candidate
     * @param timing correction for symbol timing
     * @param balance equalizer balance correction
     * @param gain equalizer gain correction.
     */
    public record Correction(double timing, float balance, float gain){};

    /**
     * Base equalizer class
     */
    abstract class Equalizer
    {
        protected boolean mInitialized = false;
        protected float mBalance = 0.0f;
        protected float mGain = 1.0f;
        protected int mEqualizerExcessBalanceDetectCount;
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
            if(Math.abs(correction.balance) > EQUALIZER_RECALIBRATE_THRESHOLD)
            {
                mEqualizerExcessBalanceDetectCount++;

                if(mEqualizerExcessBalanceDetectCount >= 2)
                {
                    System.out.println("*******************************\n\n EXCESS BALANCE DECTECTED \n\n****************************");
                    System.out.println("Equalizer Balance: " + DF.format(correction.balance) + " Gain: " + DF.format(correction.gain));
                    mInitialized = false;
                    mEqualizerExcessBalanceDetectCount = 0;
                }
            }
            else
            {
                mEqualizerExcessBalanceDetectCount = 0;
            }
             if(mInitialized)
            {
                //Limit equalizer adjustments at each sync after the initial equalizer setup.
                //                mBalance += (balanceAccumulator * EQUALIZER_LOOP_GAIN);
                mBalance += (correction.balance * EQUALIZER_LOOP_GAIN);
                mGain += (correction.gain * EQUALIZER_LOOP_GAIN);
                //            System.out.println("Balance: " + DF.format(mEqualizerBalance) + " Gain: " + DF.format(mEqualizerGain) +
                //                    " B-Acc: " + DF.format(balanceAccumulator) + " G-Acc: " + DF.format(gainAccumulator) +
                //                    " B-Chg: " + DF.format(balanceAccumulator * EQUALIZER_LOOP_GAIN) +
                //                    " G-Chg: " + DF.format(gainAccumulator * EQUALIZER_LOOP_GAIN));
            }
            else
            {
                mBalance += correction.balance;
                mGain += correction.gain;
                //            System.out.println("Balance: " + DF.format(mEqualizerBalance) + " Gain: " + DF.format(mEqualizerGain) + " B-Acc: " + DF.format(balanceAccumulator) + " G-Acc: " + DF.format(gainAccumulator));
            }

            //Constrain balance to +/- PI/4
            mBalance = Math.min(mBalance, EQUALIZER_MAXIMUM_BALANCE);
            mBalance = Math.max(mBalance, -EQUALIZER_MAXIMUM_BALANCE);

            //Constrain gain between 1.0f and 1.35f
            mGain = Math.min(mGain, EQUALIZER_MAXIMUM_GAIN);
            mGain = Math.max(mGain, 1.0f);

            if(!mInitialized)
            {
                //Apply the initial gain settings to the samples in the buffer so that the symbols can be resampled.
                for(int x = 0; x < mBufferPointer; x++)
                {
                    mBuffer[x] = (mBuffer[x] + mBalance) * mGain;
                }

                mInitialized = true;
            }
        }

        /**
         * Calculates an update to the equalizer balance and gain when the sync pattern is detected in the sample
         * buffer.  This method resamples the sync symbols and compares each soft symbol to the ideal symbol phase to
         * develop average error measurements for balance and gain.  On initial sync detection, the equalizer settings
         * are applied to the samples in the buffer allowing the symbols to be resampled during coarse sync acquisition.
         */
        public Correction update(double timingCorrection)
        {
            double resampleStart = mBufferPointer + mSamplePoint + timingCorrection;
            int resampleStartIntegral = (int)Math.floor(resampleStart);
            float symbol = SYNC_PATTERN_SYMBOLS[23];
            float resampledSoftSymbol = LinearInterpolator.calculate(mBuffer[resampleStartIntegral],
                    mBuffer[resampleStartIntegral + 1], resampleStart - resampleStartIntegral);
            float balanceAccumulator = resampledSoftSymbol - symbol;
            float balancePlus3 = resampledSoftSymbol - symbol;
            float balanceMinus3 = 0;
            float gainAccumulator = Math.abs(symbol) - Math.abs(resampledSoftSymbol);

            resampleStart -= (23 * mObservedSamplesPerSymbol); //Start at 89 (+ 1 current = 90) symbols
            resampleStartIntegral = (int)Math.floor(resampleStart);

            for(int x = 0; x < 23; x++)
            {
                if(resampleStartIntegral >= 0)
                {
                    symbol = SYNC_PATTERN_SYMBOLS[x];
                    resampledSoftSymbol = LinearInterpolator.calculate(mBuffer[resampleStartIntegral],
                            mBuffer[resampleStartIntegral + 1], resampleStart - resampleStartIntegral);
                    balanceAccumulator += resampledSoftSymbol - symbol;
                    if(symbol > 0)
                    {
                        balancePlus3 += (resampledSoftSymbol - symbol);
                    }
                    else
                    {
                        balanceMinus3 += (resampledSoftSymbol - symbol);
                    }

                    gainAccumulator += Math.abs(symbol) - Math.abs(resampledSoftSymbol);
                }

                resampleStart += mObservedSamplesPerSymbol;
                resampleStartIntegral = (int)Math.floor(resampleStart);
            }

            //Average the accumulated error over 24 sync symbols
            balanceAccumulator /= -24.0f;

            balancePlus3 /= -11.0f; //There are 11x Plus 3 and 13x Minus 3 symbols in the sync pattern.
            balanceMinus3 /= -13.0f;
            float balanceAverage = (balancePlus3 + balanceMinus3) / 2f;
            System.out.println("Balance [" + balanceAccumulator + "] Plus3 [" + balancePlus3 + "] Minus3 [" + balanceMinus3 + "] Average [" + balanceAverage + "]");
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
            for(int x = mBufferLoadPointer - copyLength; x < mBufferLoadPointer; x++)
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
            for(int x = mBufferLoadPointer - copyLength; x < mBufferLoadPointer; x++)
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

    static void main()
    {
        double[] s = new double[]{2.2646596, 2.376257, 2.4020972, 2.3868852, 2.3489807, 2.293533, 2.2039113, 1.914535, 0.32357034, 1.7023864, 2.197961, 2.2670896, 2.300842, 2.3264236, 2.3447568, 2.3503795, 2.3304927, 2.2351463, 1.3991193, 0.261054, 2.303139, 2.3558066, 2.342518, 2.3244324, 2.3102124, 2.3016934, 2.2970488, 2.2859178, 2.1841803, 0.026668489, 1.9531729, 2.2837346, 2.3302097, 2.3403783, 2.334687, 2.318292, 2.2930045, 2.255557, 2.1709278, 1.0924033, 0.55675375, 2.1811376, 2.280473, 2.3363507, 2.3927295, 2.4526892, 2.5105073, 2.5519621, 2.5401146, 2.267077, -0.2953683, -1.2436261, -2.0847025, -2.3790448, -2.4892468, -2.5463865, -2.5851789, -2.610502, -2.606251, -2.5115027, -2.105523, -2.5508661, 2.425252, 2.4801722, 2.4577172, 2.407239, 2.348917, 2.2928603, 2.2453144, 2.212419, 2.244208, -1.2654984, 2.5916674, 2.3814042, 2.396392, 2.4501975, 2.5217884, 2.6019921, 2.6833775, 2.7593985, 2.8404284, -2.2061656, -1.2051437, -1.9697014, -2.3342767, -2.4395044, -2.4648542, -2.4613245, -2.4431243, -2.408314, -2.3320532, -2.1102843, -1.5830822, -1.6340878, -2.0804286, -2.3050392, -2.4238448, -2.5083504, -2.5785418, -2.634445, -2.6595168, -2.6038218, -2.388157, -3.099742, 2.5971935, 2.5273414, 2.4710932, 2.4129627, 2.3580315, 2.3115516, 2.281592, 2.3014436, 2.7311323, -2.6483858, 2.7870836, 2.543695, 2.5041218, 2.5296564, 2.5909228, 2.6801958, 2.8003528, -3.3032951, -2.8920772, -1.8052965, -1.5979044, -2.0542157, -2.329488, -2.416012, -2.4345117, -2.429482, -2.4124885, -2.377544, -2.2951384, -2.0815763, -1.7188495, -1.8021612, -2.1538653, -2.3207576, -2.3808792, -2.3975399, -2.3922768, -2.3687527, -2.3146303, -2.182519, -1.8647058, -1.5685468, -1.8050231, -2.1176608, -2.274186, -2.3439224, -2.3707457, -2.371499, -2.350297, -2.2971125, -2.170418, -1.9331696, -1.8693593, -2.0717754, -2.2742288, -2.4370828, -2.583502, -2.7242517, -2.8573534, -2.9700365, -3.040701, -3.0833087, -3.269108, 2.752115, 2.6232533, 2.5944028, 2.6316366, 2.7238052, -3.4136977, -3.2044811, -2.8912184, -2.3961456, -1.9019454, -1.8328991, -2.0536032, -2.272393, -2.4322197, -2.5677028, -2.701907, -2.8407836, -2.9785936, -3.1010337, -3.1973162, -3.3058186, 2.8312902, 2.7224796, 2.6791313, 2.697456, 2.7748127, -3.3695638, -3.158972, -2.852725, -2.436537, -2.053236, -1.973826, -2.1374166, -2.2741237, -2.3301594, -2.346658, -2.3483891, -2.34122, -2.3181198, -2.259155, -2.1403153, -2.0284371, -2.0887396, -2.219416, -2.3012867, -2.3371215, -2.343586, -2.32661, -2.2810504, -2.1876824, -2.0136395, -1.783971, -1.7209476, -1.9227631, -2.1553013, -2.300848, -2.380974, -2.4212108, -2.4305947, -2.4049776, -2.3264973, -2.1672463, -1.9641122, -1.9347963, -2.1029239, -2.2338536, -2.2811224, -2.2847557, -2.2734122, -2.2606351, -2.2471707, -2.215411, -2.126466, -2.0072904, -2.0104525, -2.1323447, -2.0239475, -2.23187, -2.6735568};
        float[] samples = new float[s.length];
        for(int i = 0; i < s.length; i++)
        {
            samples[i] = (float)s[i];
        }

        double samplesPerSymbol = 50_000d / 4_800d;
    }
}
