/*******************************************************************************
 * sdr-trunk
 * Copyright (C) 2014-2020 Dennis Sheirer
 *
 * This program is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,  
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License  
 * along with this program.
 * If not, see <http://www.gnu.org/licenses/>
 *
 ******************************************************************************/
package io.github.dsheirer.dsp.gain;

import io.github.dsheirer.sample.buffer.ReusableBufferQueue;
import io.github.dsheirer.sample.buffer.ReusableComplexBuffer;
import io.github.dsheirer.sample.buffer.ReusableComplexBufferQueue;
import io.github.dsheirer.sample.complex.Complex;
import org.apache.commons.math3.util.FastMath;
import java.util.concurrent.atomic.AtomicBoolean;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Performs automatic squelch on baseband I/Q samples to produce a mute/unmute output.
 */
public class ComplexAGCSquelchControl
{
    private final static Logger mLog = LoggerFactory.getLogger(ComplexAGCSquelchControl.class);
    private ReusableComplexBufferQueue mReusableComplexBufferQueue =
            new ReusableComplexBufferQueue("ComplexAGCSquelchControl");
    private int mOutputBufferPointer;

    private static final double SQUELCH_DEFAULT_BW 		= 0.25;

    public static final int SQUELCH_DISABLED    		= 0;
    public static final int SQUELCH_ENABLED     		= 1;
    public static final int SQUELCH_RISE        		= 2;
    public static final int SQUELCH_SIGNAL_HIGH 		= 3;
    public static final int SQUELCH_FALL        		= 4;
    public static final int SQUELCH_SIGNAL_LOW  		= 5;
    public static final int SQUELCH_TIMEOUT     		= 6;
    public static final int SQUELCH_UNKNOWN     		= 99;

    public static final int SQUELCH_OK                  =  1;
    public static final int SQUELCH_ERROR               =  0;
    public static final int SQUELCH_BANDWITH_LOW_ERROR  = -10;
    public static final int SQUELCH_BANDWITH_HIGH_ERROR = -11;

    // gain variables
    private double mGain  = 1.0;   				// current gain value

    // gain control loop filter parameters
    private double mBandwidth = 0.0;			// bandwidth-time constant
    private double mFeedbackGain = 0.0;			// feed-back gain

    private double mEstimatedSignal = 1.0;		// filtered output signal energy estimate
    private double mSquelchThreshold = -120.0; 	// squelch threshold

    private int    mSquelchMode = SQUELCH_DISABLED; // squelch mode state

    // squelch timeout
    private int    mSquelchTimeout;
    private int    mSquelchTimer;

    private AtomicBoolean mIsLocked = new AtomicBoolean( true );

    /**
     * Constructs this agc where the specified gain is applied to output samples.
     *
     * @param gain to apply to output samples.
     */
    public ComplexAGCSquelchControl(float gain, float squelch_threshold)
    {
        // initialize bandwidth
        setBandwidth( SQUELCH_DEFAULT_BW );

        // reset object
        reset();

        // squelch
//    	setSquelchDisable();
        setSquelchEnable();
        setSquelchThreshold( squelch_threshold );
        setSquelchTimeout( 100 );

        setGain(gain);
        setSignalLevel(1e-3);

    }

    /**
     * initialize internal gain on input array
     *
     * @param in : input data array, [size: n x 1]
     * @param size : number of input, output samples
     */
    private void init(Complex[] in,  int size)
    {
        // compute sum squares on input
        double s = 0.0;
        Complex sample;
        for (int i = 0; i < size; i++) {
            // compute output signal energy
            sample = Complex.multiply(in[i], in[i].conjugate());
            s += sample.real();
        }

        // compute RMS level and ensure result is positive
        double rms = FastMath.sqrt( s / size ) + 1e-16;

        // set internal gain based on estimated signal level
        setSignalLevel(rms);
    }

    /**
     * reset squelch object's internal state
     */
    private void reset()
    {
        // reset gain estimate
        mGain = 1.0;

        // reset signal level estimate
        mEstimatedSignal = 1.0;

        // unlock squelch control
        unlock();

        // reset squelch state
        mSquelchMode = ( mSquelchMode == SQUELCH_DISABLED ) ?
                SQUELCH_DISABLED : SQUELCH_ENABLED;
    }

    /**
     * execute automatic gain control loo
     *
     * @param in input sample.
     * @param out output sample.
     *
     * @return processed OK / Failed
     */
    private float agc(Complex sample)
    {

        // apply gain to input sample
        Complex s = Complex.multiply(sample, (float) mGain);

        // compute output signal energy
        s = Complex.multiply(s, s.conjugate());
        double out = s.real();

        // return if locked
        if ( mIsLocked.get() == true )
            return (float) out;

        // smooth energy estimate using single-pole low-pass filter
        mEstimatedSignal = ( 1.0 - mFeedbackGain ) * mEstimatedSignal + mFeedbackGain * out;

        // update gain according to output energy
        if ( mEstimatedSignal > 1e-6 )
            mGain = mGain * FastMath.exp( -0.5 * mFeedbackGain * FastMath.log10(mEstimatedSignal) );

        // clamp to 120 dB gain
        if ( mGain > 1e6 )
            mGain = 1e6;

        // udpate squelch mode appropriately
        updateSquelchMode();

        return (float) out;
    }

    /**
     * lock agc
     */
    public void lock()
    {
        mIsLocked.set( true );
    }

    /**
     * unlock agc
     */
    public void unlock()
    {
        mIsLocked.set( false );
    }

    /**
     * get loop bandwidth
     *
     * @return bandwidth.
     */
    public double getBandwidth()
    {
        return mBandwidth;
    }

    /**
     * set loop bandwidth
     *
     * @param bandwidth between 0 to 1.0
     */
    public void setBandwidth(double bandwidth)
    {
        // set internal bandwidth
        mBandwidth = bandwidth;

        // compute filter coefficient based on bandwidth
        mFeedbackGain = mBandwidth;
    }

    /**
     * get estimated signal level (linear)
     *
     * @return estimated signal level.
     */
    public double getSignalLevel()
    {
        return 1.0 / mGain;
    }

    /**
     * check to ensure signal level is reasonable
     *
     * @param signal_level : estimated signal level (linear)
     */
    public void setSignalLevel(double signal_level)
    {
        // set internal gain appropriately
        mGain = 1.0 / signal_level;

        // reset internal output signal level
        mEstimatedSignal = 1.0;
    }

    /**
     * get rssi (dB)
     *
     * @return rssi (dB).
     */
    public double getRssi()
    {
        return -20.0 * FastMath.log10(mGain);
    }

    /**
     * set rssi (dB)
     *
     * @param rssi : estimated signal level
     */
    public void setRssi(double rssi)
    {
        // set internal gain appropriately
        mGain = FastMath.pow( 10.0, -rssi / 20.0 );

        // ensure resulting gain is not arbitrarily low
        if ( mGain < 1e-16 )
            mGain = 1e-16;

        // reset internal output signal level
        mEstimatedSignal = 1.0;
    }

    /**
     * get internal gain
     *
     * @return internal gain.
     */
    public double getGain()
    {
        return mGain;
    }

    /**
     * set internal gain
     *
     * @param gain : set the gain
     */
    public void setGain(double gain)
    {
        // set internal gain appropriately
        mGain = gain;
    }

    /**
     * enable squelch mode
     */
    public void setSquelchEnable()
    {
        mSquelchMode = SQUELCH_ENABLED;
    }

    /**
     * disable squelch mode
     */
    public void setSquelchDisable()
    {
        mSquelchMode = SQUELCH_DISABLED;
    }

    /**
     * is squelch enabled?
     *
     * @return squelch enabled true/false.
     */
    public boolean isSquelchEnabled()
    {
        return mSquelchMode == SQUELCH_DISABLED ? false : true;
    }

    /**
     * get squelch mode
     *
     * @return squelch mode.
     */
    public int getSquelchMode()
    {
        return mSquelchMode;
    }

    /**
     * set squelch threshold
     *
     * @param threshold : threshold for enabling squelch [dB].
     */
    public void setSquelchThreshold(float threshold)
    {
        mSquelchThreshold = threshold;
    }

    /**
     * get squelch threshold
     *
     * @return squelch threshold.
     */
    public double getSquelchThreshold()
    {
        return mSquelchThreshold;
    }

    /**
     *
     * set squelch timeout
     *
     * @param timeout : timeout before enabling squelch [samples]
     */
    public void setSquelchTimeout(int timeout)
    {
        mSquelchTimeout = timeout;
    }

    /**
     *
     * get squelch timeout [samples]
     *
     * @return squelch timeout [samples].
     */
    public int getSquelchTimeout()
    {
        return mSquelchTimeout;
    }

    /**
     * Performes signal analysis to the reusable complex sample buffer and 
     re	 * turns a rssi float buffer
     *
     * @param complexBuffer to demodulate
     * @return demodulated audio buffer.
     */
    public ReusableComplexBuffer process(ReusableComplexBuffer complexBuffer)
    {
        ReusableComplexBuffer reusableComplexBuffer = mReusableComplexBufferQueue.getBuffer(complexBuffer.getSampleCount());
        float[] input = complexBuffer.getSamples();
        float[] output = reusableComplexBuffer.getSamples();
        mOutputBufferPointer = 0;
        Complex sample;

        for(int x= 0; x < complexBuffer.getSamples().length; x += 2)
        {
            sample = new Complex(input[x], input[x + 1]);
            output[mOutputBufferPointer++] = agc( sample );
        }

        complexBuffer.decrementUserCount();

        return reusableComplexBuffer;
    }

    /**
     * private methods
     */

    /**
     *
     * update squelch mode appropriately
     *
     */
    private void updateSquelchMode()
    {
        boolean threshold_exceeded = getRssi() > mSquelchThreshold;

        // update state
        switch (mSquelchMode) {
            case SQUELCH_ENABLED:
                mSquelchMode = threshold_exceeded ? SQUELCH_RISE : SQUELCH_ENABLED;
                break;
            case SQUELCH_RISE:
                mSquelchMode = threshold_exceeded ? SQUELCH_SIGNAL_HIGH : SQUELCH_FALL;
                break;
            case SQUELCH_SIGNAL_HIGH:
                mSquelchMode = threshold_exceeded ? SQUELCH_SIGNAL_HIGH : SQUELCH_FALL;
                break;
            case SQUELCH_FALL:
                mSquelchMode = threshold_exceeded ? SQUELCH_SIGNAL_HIGH : SQUELCH_SIGNAL_LOW;
                mSquelchTimer = mSquelchTimeout;
                break;
            case SQUELCH_SIGNAL_LOW:
                mSquelchTimer--;
                if (mSquelchTimer == 0)
                    mSquelchMode = SQUELCH_TIMEOUT;
                else if (threshold_exceeded)
                    mSquelchMode = SQUELCH_SIGNAL_HIGH;
                break;
            case SQUELCH_TIMEOUT:
                mSquelchMode = SQUELCH_ENABLED;
                break;
            case SQUELCH_DISABLED:
                break;
            case SQUELCH_UNKNOWN:
            default:
                mLog.info("warning: Invalid squelch mode:" + mSquelchMode);
        }
    }
}
