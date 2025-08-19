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
import io.github.dsheirer.dsp.symbol.DibitDelayLine;

/**
 * Circular buffer for storing and accessing dibits and extract the NID.
 */
public class NidDibitDelayLine extends DibitDelayLine
{
    private static final int DIBIT_LENGTH_NID = 33; //32 dibits (64 bits) plus 1 status symbol dibit

    /**
     * Constructs a dibit delay buffer
     */
    public NidDibitDelayLine()
    {
        super(DIBIT_LENGTH_NID);
    }

    /**
     * Extracts the NID codeword from the dibit delay buffer.  The delay buffer is sized to 33 dibits which is 32
     * dibits for the NID and an extra dibit for the status symbol.  We extract the NID from the buffer once there
     * has been 33 dibits since the sync pattern detection.  The buffer pointer should be pointing to the first
     * dibit of the NID, since this is a circular buffer.
     *
     * @return message bits containing the NID.
     */
    public CorrectedBinaryMessage getNIDMessage(int offset)
    {
        //Capture just the 63-bit BCH protected NID codeword including the 64th parity bit which we ignore.
        CorrectedBinaryMessage nid = new CorrectedBinaryMessage(64);

        int delayLinePointer = mPointer + offset;

        if(delayLinePointer >= mDelayLine.length)
        {
            delayLinePointer -= mDelayLine.length;
        }
        else if(delayLinePointer < 0)
        {
            delayLinePointer += mDelayLine.length;
        }

        Dibit dibit = null;

        for(int x = 0; x < 33; x++)
        {
            if(x == 11)
            {
                delayLinePointer++; //Skip the status symbol that's in the middle of the NID
            }
            else
            {
                dibit = mDelayLine[delayLinePointer++];
                nid.add(dibit.getBit1(), dibit.getBit2());
            }

            if(delayLinePointer >= mDelayLine.length)
            {
                delayLinePointer = 0;
            }
        }

        return nid;
    }
}
