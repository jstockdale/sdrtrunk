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
import io.github.dsheirer.dsp.symbol.Dibit;
import io.github.dsheirer.message.DroppedSamplesMessage;
import io.github.dsheirer.message.IMessage;
import io.github.dsheirer.message.SyncLossMessage;
import io.github.dsheirer.module.decode.p25.phase1.P25P1ChannelStatusProcessor;
import io.github.dsheirer.module.decode.p25.phase1.P25P1DataUnitID;
import io.github.dsheirer.module.decode.p25.phase1.message.P25MessageFactory;
import io.github.dsheirer.module.decode.p25.phase1.message.P25P1Message;
import io.github.dsheirer.module.decode.p25.phase1.message.pdu.PDUHeader;
import io.github.dsheirer.module.decode.p25.phase1.message.pdu.PDUMessageFactory;
import io.github.dsheirer.module.decode.p25.phase1.message.pdu.PDUSequence;
import io.github.dsheirer.module.decode.p25.phase1.message.tsbk.TSBKMessage;
import io.github.dsheirer.module.decode.p25.phase1.message.tsbk.TSBKMessageFactory;
import io.github.dsheirer.protocol.Protocol;
import io.github.dsheirer.sample.Listener;

/**
 * Provides message framing for the demodulated dibit stream.  This framer is notified by an external sync detection
 * process using the two syncDetected() methods below to indicate if the NID that follows the sync was correctly error
 * detected and corrected.  When the NID does not pass error correction, we use a PLACEHOLDER data unit ID to allow the
 * uncertain message to assemble and then we'll inspect before and after data unit IDs and the quantity of captured
 * dibits to make a best guess on what the assembled message represents.
 */
public class P25P1SoftMessageFramer implements Listener<Dibit>
{
    private static final double MILLISECONDS_PER_SYMBOL = 1.0 / 4800.0 / 1000.0;
    private Listener<IMessage> mMessageListener;
    private boolean mRunning = false;
    private boolean mMessageAssemblyRequired = false;
    private int mDibitCounter = 58; //Set to 1-greater than SYNC+NID to avoid triggering message assembly on startup
    private int mDibitSinceTimestampCounter = 0;
    private int mStatusSymbolDibitCounter = 36; //Set to 1-greater than the suppression trigger at 35 dibits
    private int mTrailingDibitsToSuppress = 0;
    private long mReferenceTimestamp = 0;
    private P25P1MessageAssembler mMessageAssembler;
    private P25P1DataUnitID mPreviousDataUnitID = P25P1DataUnitID.PLACEHOLDER;
    private P25P1DataUnitID mDetectedDataUnitID = P25P1DataUnitID.PLACEHOLDER;
    private int mDetectedNAC = 0;
    private int mDetectedBitErrors = 0;
    private P25P1ChannelStatusProcessor mChannelStatusProcessor = new P25P1ChannelStatusProcessor();
    private PDUSequence mPDUSequence;

    /**
     * Primary method that receives the demodulated dibit stream from the symbol processor.
     * @param dibit that was demodulated.
     */
    @Override
    public void receive(Dibit dibit)
    {
        mDibitSinceTimestampCounter++;

        //Strip status symbol after every 35 dibits/70 bits.  This counter is reset to zero on sync detect and runs
        //continuously even when we don't have a sync detect and not assembling a message.
        mStatusSymbolDibitCounter++;

        if(mStatusSymbolDibitCounter == 36)
        {
            if(mMessageAssemblyRequired || mMessageAssembler != null)
            {
                //Send status dibit to channel status processor to identify ISP or OSP channel
                mChannelStatusProcessor.receive(dibit);
            }

            mStatusSymbolDibitCounter = 0;
            return;
        }

        mDibitCounter ++;

        if(mMessageAssembler != null)
        {
            mMessageAssembler.receive(dibit);

            if(mMessageAssembler.isComplete())
            {
                dispatchMessage();
            }
        }
        //Start a message assembler after ignoring 24x Sync and 32x NID dibits. Don't feed the 56th dibit to the assembler.
        else if(mMessageAssemblyRequired && mDibitCounter == 56)
        {
            mMessageAssembler = new P25P1MessageAssembler(mDetectedNAC, mDetectedDataUnitID);
            mMessageAssemblyRequired = false;
        }
        else if(mDibitCounter >= 4800) //4800x (1-sec).
        {
            mDibitCounter -= 4800;
            broadcast(new SyncLossMessage(getTimestamp(), 9600, Protocol.APCO25));
        }
    }

    /**
     * Dispatch the message currently in the message assembler.
     */
    private void dispatchMessage()
    {
        //Note: the message assembler should have a valid DUID on it via the forceCompletion() method.  Capture the
        //current DUID as the previous, before the assembler is nullified.
        mPreviousDataUnitID = mMessageAssembler.getDataUnitID();

//TODO: add the detected bit error count from the SYNC and NAC to the dispatched message ...

        if(mMessageListener != null)
        {
            switch(mMessageAssembler.getDataUnitID())
            {
                case TRUNKING_SIGNALING_BLOCK_1:
                case TRUNKING_SIGNALING_BLOCK_2:
                case TRUNKING_SIGNALING_BLOCK_3:
                    dispatchTSBK();
                    break;
                case PACKET_DATA_UNIT:
                case PACKET_DATA_UNIT_BLOCK_1:
                case PACKET_DATA_UNIT_BLOCK_2:
                case PACKET_DATA_UNIT_BLOCK_3:
                case PACKET_DATA_UNIT_BLOCK_4:
                case PACKET_DATA_UNIT_BLOCK_5:
                    dispatchPDU();
                    break;
                case TERMINATOR_DATA_UNIT:
                    dispatchTDU();
                    break;
                case TERMINATOR_DATA_UNIT_LINK_CONTROL:
                    dispatchTDULC();
                    break;
                default:
                    dispatchOther();
                    break;
            }
        }
        else
        {
            mMessageAssembler = null;
        }
    }

    /**
     * Updates the dibit counter with the dibits collected on the current message before message assembler disposal.
     */
    private void adjustDibitCounterFromMessageAssembler()
    {
        if(mMessageAssembler != null)
        {
            mDibitCounter -= ((mMessageAssembler.getMessage().currentSize() / 2) + 56); //SYNC + NID + Message
        }
    }

    private void dispatchTDU()
    {
        adjustDibitCounterFromMessageAssembler();

        CorrectedBinaryMessage cbm = mMessageAssembler.getMessage();
        P25P1Message message = P25MessageFactory.create(mMessageAssembler.getDataUnitID(), mMessageAssembler.getNAC(),
                getTimestamp(), cbm);

        if(message != null)
        {
            mMessageListener.receive(message);
        }
        else
        {
            SyncLossMessage slm = new SyncLossMessage(getTimestamp(), cbm.currentSize(), Protocol.APCO25);
            mMessageListener.receive(slm);
        }

        mMessageAssembler = null;
    }

    private void dispatchTDULC()
    {
        //      TODO: dibit accounting ..
        //      adjustDibitCounterFromMessageAssembler();

        CorrectedBinaryMessage cbm = mMessageAssembler.getMessage();
        P25P1Message message = P25MessageFactory.create(mMessageAssembler.getDataUnitID(), mMessageAssembler.getNAC(),
                getTimestamp(), cbm);

        if(message != null)
        {
            mMessageListener.receive(message);
        }
        else
        {
            SyncLossMessage slm = new SyncLossMessage(getTimestamp(), cbm.currentSize(), Protocol.APCO25);
            mMessageListener.receive(slm);
        }

        mMessageAssembler = null;
    }

    /**
     * Dispatches the message currently in the message assembler when the DUID is not PDU or TSBK.
     */
    private void dispatchOther()
    {
        adjustDibitCounterFromMessageAssembler();

        CorrectedBinaryMessage cbm = mMessageAssembler.getMessage();
        P25P1Message message = P25MessageFactory.create(mMessageAssembler.getDataUnitID(), mMessageAssembler.getNAC(),
                getTimestamp(), cbm);

        if(message != null)
        {
            mMessageListener.receive(message);
        }
        else
        {
            SyncLossMessage slm = new SyncLossMessage(getTimestamp(), cbm.currentSize(), Protocol.APCO25);
            mMessageListener.receive(slm);
        }

        mMessageAssembler = null;
    }

    /**
     * Indicates if a message is being assembled.  Note: returns false when we are assembling to a PLACEHOLDER since
     * that is a non-specific message assembly.
     */
    public boolean isAssembling()
    {
        return mMessageAssembler != null || mMessageAssemblyRequired || mDibitCounter < 2;
    }

    public P25P1DataUnitID getAssemblingDUID()
    {
        if(isAssembling())
        {
            return mMessageAssembler.getDataUnitID();
        }

        return null;
    }

    /**
     * Dispatches the message currently in the message assembler when the DUID is TSBK1, TSBK2, or TSBK3.
     */
    private void dispatchTSBK()
    {
        switch(mMessageAssembler.getDataUnitID())
        {
            case TRUNKING_SIGNALING_BLOCK_1:
                CorrectedBinaryMessage message1 = mMessageAssembler.getMessage().getSubMessage(0, 195);
                TSBKMessage tsbk1 = TSBKMessageFactory.create(mChannelStatusProcessor.getDirection(), mMessageAssembler.getDataUnitID(), message1, mMessageAssembler.getNAC(), getTimestamp());

                if(tsbk1 != null)
                {
                    mMessageListener.receive(tsbk1);

                    if(tsbk1.isLastBlock())
                    {
                        adjustDibitCounterFromMessageAssembler();
                        mMessageAssembler = null;
                    }
                    else //Setup to capture TSBK2
                    {
                        mMessageAssembler.reconfigure(P25P1DataUnitID.TRUNKING_SIGNALING_BLOCK_2);
                    }
                }
                else
                {
                    adjustDibitCounterFromMessageAssembler();
                    mMessageAssembler = null;
                }
                break;
            case TRUNKING_SIGNALING_BLOCK_2:
                CorrectedBinaryMessage message2 = mMessageAssembler.getMessage().getSubMessage(196, 391);
                TSBKMessage tsbk2 = TSBKMessageFactory.create(mChannelStatusProcessor.getDirection(), mMessageAssembler.getDataUnitID(), message2, mMessageAssembler.getNAC(), getTimestamp());

                if(tsbk2 != null)
                {
                    mMessageListener.receive(tsbk2);

                    if(tsbk2.isLastBlock())
                    {
                        adjustDibitCounterFromMessageAssembler();
                        mMessageAssembler = null;
                    }
                    else //Setup to capture TSBK2
                    {
                        mMessageAssembler.reconfigure(P25P1DataUnitID.TRUNKING_SIGNALING_BLOCK_3);
                    }
                }
                else
                {
                    adjustDibitCounterFromMessageAssembler();
                    mMessageAssembler = null;
                }
                break;
            case TRUNKING_SIGNALING_BLOCK_3:
                CorrectedBinaryMessage message3 = mMessageAssembler.getMessage().getSubMessage(392, 588);
                TSBKMessage tsbk3 = TSBKMessageFactory.create(mChannelStatusProcessor.getDirection(), mMessageAssembler.getDataUnitID(), message3, mMessageAssembler.getNAC(), getTimestamp());

                if(tsbk3 != null)
                {
                    mMessageListener.receive(tsbk3);
                }

                adjustDibitCounterFromMessageAssembler();
                mMessageAssembler = null;
                break;
            default:
                System.out.println("Unexpected TSBK DUID: " +  mMessageAssembler.getDataUnitID());
        }
    }

    /**
     * Dispatches a sync loss message to account for lost bits.
     * @param bitCount that was lost.
     */
    private void dispatchSyncLoss(int bitCount)
    {
        if(mMessageListener != null && bitCount > 0)
        {
            mMessageListener.receive(new SyncLossMessage(getTimestamp(), bitCount, Protocol.APCO25));
        }
    }

    /**
     * Dispatches the message currently in the message assembler when the DUID is PDU or PDU1
     */
    private void dispatchPDU()
    {
        switch(mMessageAssembler.getDataUnitID())
        {
            case PACKET_DATA_UNIT:
                CorrectedBinaryMessage message = mMessageAssembler.getMessage().getSubMessage(0, 195);
                PDUHeader header = PDUMessageFactory.createHeader(message);

                if(header != null)
                {
                    mPDUSequence = new PDUSequence(header, getTimestamp(), mMessageAssembler.getNAC());

                    if(mPDUSequence.getHeader().isValid() && mPDUSequence.getHeader().getBlocksToFollowCount() > 0)
                    {
                        //Setup to catch the sequence of data blocks that follow the header
                        mMessageAssembler.reconfigure(P25P1DataUnitID.PACKET_DATA_UNIT_BLOCK_1);
                    }
                    else
                    {
                        adjustDibitCounterFromMessageAssembler();
                        mMessageListener.receive(PDUMessageFactory.create(mPDUSequence, mMessageAssembler.getNAC(),
                                getTimestamp()));
                        mPDUSequence = null;
                        mMessageAssembler = null;
                    }
                }
                else
                {
                    adjustDibitCounterFromMessageAssembler();
                    mMessageAssembler = null;
                    mPDUSequence = null;
                }
                break;
            case PACKET_DATA_UNIT_BLOCK_1:
                if(mPDUSequence != null)
                {
                    CorrectedBinaryMessage messageB1 = mMessageAssembler.getMessage().getSubMessage(196, 391);

                    if(mPDUSequence.getHeader().isConfirmationRequired())
                    {
                        mPDUSequence.addDataBlock(PDUMessageFactory.createConfirmedDataBlock(messageB1));
                    }
                    else
                    {
                        mPDUSequence.addDataBlock(PDUMessageFactory.createUnconfirmedDataBlock(messageB1));
                    }

                    if(mPDUSequence.isComplete())
                    {
                        adjustDibitCounterFromMessageAssembler();
                        mMessageListener.receive(PDUMessageFactory.create(mPDUSequence, mMessageAssembler.getNAC(),
                                getTimestamp()));
                        mMessageAssembler = null;
                    }
                    else
                    {
                        //Setup to catch the next data block
                        mMessageAssembler.reconfigure(P25P1DataUnitID.PACKET_DATA_UNIT_BLOCK_2);
                    }
                }
                else
                {
                    adjustDibitCounterFromMessageAssembler();
                    mMessageAssembler = null;
                }
                break;
            case PACKET_DATA_UNIT_BLOCK_2:
                if(mPDUSequence != null)
                {
                    CorrectedBinaryMessage messageB2 = mMessageAssembler.getMessage().getSubMessage(392, 587);

                    if(mPDUSequence.getHeader().isConfirmationRequired())
                    {
                        mPDUSequence.addDataBlock(PDUMessageFactory.createConfirmedDataBlock(messageB2));
                    }
                    else
                    {
                        mPDUSequence.addDataBlock(PDUMessageFactory.createUnconfirmedDataBlock(messageB2));
                    }

                    if(mPDUSequence.isComplete())
                    {
                        adjustDibitCounterFromMessageAssembler();
                        mMessageListener.receive(PDUMessageFactory.create(mPDUSequence, mMessageAssembler.getNAC(),
                                getTimestamp()));
                        mMessageAssembler = null;
                        mPDUSequence = null;
                    }
                    else
                    {
                        //Setup to catch the next data block
                        mMessageAssembler.reconfigure(P25P1DataUnitID.PACKET_DATA_UNIT_BLOCK_3);
                    }
                }
                else
                {
                    adjustDibitCounterFromMessageAssembler();
                    mMessageAssembler = null;
                }
                break;
            case PACKET_DATA_UNIT_BLOCK_3:
                if(mPDUSequence != null)
                {
                    CorrectedBinaryMessage messageB3 = mMessageAssembler.getMessage().getSubMessage(588, 783);

                    if(mPDUSequence.getHeader().isConfirmationRequired())
                    {
                        mPDUSequence.addDataBlock(PDUMessageFactory.createConfirmedDataBlock(messageB3));
                    }
                    else
                    {
                        mPDUSequence.addDataBlock(PDUMessageFactory.createUnconfirmedDataBlock(messageB3));
                    }

                    if(mPDUSequence.isComplete())
                    {
                        adjustDibitCounterFromMessageAssembler();
                        mMessageListener.receive(PDUMessageFactory.create(mPDUSequence, mMessageAssembler.getNAC(), getTimestamp()));
                        mMessageAssembler = null;
                    }
                    else
                    {
                        //Setup to catch the next data block
                        mMessageAssembler.reconfigure(P25P1DataUnitID.PACKET_DATA_UNIT_BLOCK_4);
                    }
                }
                else
                {
                    adjustDibitCounterFromMessageAssembler();
                    mMessageAssembler = null;
                }
                break;
            case PACKET_DATA_UNIT_BLOCK_4:
                if(mPDUSequence != null)
                {
                    CorrectedBinaryMessage messageB4 = mMessageAssembler.getMessage().getSubMessage(784, 979);

                    if(mPDUSequence.getHeader().isConfirmationRequired())
                    {
                        mPDUSequence.addDataBlock(PDUMessageFactory.createConfirmedDataBlock(messageB4));
                    }
                    else
                    {
                        mPDUSequence.addDataBlock(PDUMessageFactory.createUnconfirmedDataBlock(messageB4));
                    }

                    if(mPDUSequence.isComplete())
                    {
                        adjustDibitCounterFromMessageAssembler();
                        mMessageListener.receive(PDUMessageFactory.create(mPDUSequence, mMessageAssembler.getNAC(), getTimestamp()));
                        mMessageAssembler = null;
                    }
                    else
                    {
                        //Setup to catch the last data block
                        mMessageAssembler.reconfigure(P25P1DataUnitID.PACKET_DATA_UNIT_BLOCK_5);
                    }
                }
                else
                {
                    adjustDibitCounterFromMessageAssembler();
                    mMessageAssembler = null;
                }
                break;
            case PACKET_DATA_UNIT_BLOCK_5:
                if(mPDUSequence != null)
                {
                    CorrectedBinaryMessage messageB5 = mMessageAssembler.getMessage().getSubMessage(980, 1176);

                    if(mPDUSequence.getHeader().isConfirmationRequired())
                    {
                        mPDUSequence.addDataBlock(PDUMessageFactory.createConfirmedDataBlock(messageB5));
                    }
                    else
                    {
                        mPDUSequence.addDataBlock(PDUMessageFactory.createUnconfirmedDataBlock(messageB5));
                    }

                    adjustDibitCounterFromMessageAssembler();

                    mMessageListener.receive(PDUMessageFactory.create(mPDUSequence, mMessageAssembler.getNAC(),
                            getTimestamp()));
                    mMessageAssembler = null;
                    mPDUSequence = null;
                }
                else
                {
                    adjustDibitCounterFromMessageAssembler();
                    mMessageAssembler = null;
                }
                break;
            default:
                System.out.println("Unexpected PDU DUID: " + mMessageAssembler.getMessage());
                mMessageAssembler = null;
                mPDUSequence = null;
        }
    }

    private void reset()
    {
        mPDUSequence = null;
        mStatusSymbolDibitCounter = 0;
    }

    /**
     * Broadcasts the assembled message to the registered listener.
     * @param message to broadcast - ignored if there is no registered listener.
     */
    private void broadcast(IMessage message)
    {
        if(mMessageListener != null && message != null)
        {
            mMessageListener.receive(message);
        }
    }

    /**
     * Externally provided trigger that a sync pattern is detected and the next arriving dibit is the first symbol of
     * that detected sync.  This method is triggered when sync is detected and either:
     * a) a valid NID is decoded from the look-ahead sample buffer or,
     * b) the sync optimization process produces a high-quality correlation score
     *
     * When the trigger is option b, the DUID will be the PLACEHOLDER.
     *
     * @param nac value decoded from the NID.
     * @param dataUnitID decoded from the NID
     * @param detectedBitErrors across the SYNc and NID
     */
    public void syncDetected(int nac, P25P1DataUnitID dataUnitID, int detectedBitErrors)
    {
        mDetectedNAC = nac;
        mDetectedDataUnitID = dataUnitID;
        mDetectedBitErrors = detectedBitErrors;

        //If there is a message assembler (still) active, force it to complete
        if(mMessageAssembler != null)
        {
            if(mMessageAssembler.getDataUnitID() == P25P1DataUnitID.PLACEHOLDER)
            {
                //Don't send dropped samples message for placeholder since we had a bad NID decode.
                mMessageAssembler.forceCompletion(mPreviousDataUnitID, mDetectedDataUnitID);
                dispatchMessage();
            }
            else
            {
                int droppedBits = mMessageAssembler.forceCompletion(mPreviousDataUnitID, mDetectedDataUnitID);
                broadcast(new DroppedSamplesMessage(getTimestamp(), droppedBits, Protocol.APCO25, 0));
                dispatchMessage();
            }
        }

        if(mDibitCounter > 0)
        {
            dispatchSyncLoss(mDibitCounter * 2);
        }

        //Set dibit counter to 0 -- we'll start a message assembler once we skip the SYNC and NID dibits at dibit count=57
        mMessageAssemblyRequired = true;
        mDibitCounter = 0;
        mStatusSymbolDibitCounter = 0;
    }

    /**
     * Starts this framer dispatching messages
     */
    public void start()
    {
        mRunning = true;
    }

    /**
     * Stops this framer from dispatching messages
     */
    public void stop()
    {
        mRunning = false;
    }

    /**
     * Sets the listener to receive framed DMR messages.
     * @param listener for messages.
     */
    public void setListener(Listener<IMessage> listener)
    {
        mMessageListener = listener;
    }

    /**
     * Sets or updates the current dibit stream time from an incoming sample buffer.
     * @param time to use as a reference timestamp.
     */
    public void setTimestamp(long time)
    {
        mReferenceTimestamp = time;
        mDibitSinceTimestampCounter = 0;
    }

    /**
     * Calculates the timestamp accurate to the currently received dibit.
     * @return timestamp in milliseconds.
     */
    private long getTimestamp()
    {
        if(mReferenceTimestamp > 0)
        {
            return mReferenceTimestamp + (long)(1000.0 * mDibitSinceTimestampCounter / 4800);
        }
        else
        {
            mDibitSinceTimestampCounter = 0;
            return System.currentTimeMillis();
        }
    }
}
