/*******************************************************************************
 * sdr-trunk
 * Copyright (C) 2014-2018 Dennis Sheirer
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by  the Free Software Foundation, either version 3 of the License, or  (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; without even the implied
 * warranty of  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License  along with this program.
 * If not, see <http://www.gnu.org/licenses/>
 *
 ******************************************************************************/
package io.github.dsheirer.source.tuner.manager;

import io.github.dsheirer.sample.Broadcaster;
import io.github.dsheirer.sample.Listener;
import io.github.dsheirer.sample.complex.ComplexBuffer;
import io.github.dsheirer.sample.complex.IComplexBufferProvider;
import io.github.dsheirer.source.ISourceEventProcessor;
import io.github.dsheirer.source.SourceEvent;
import io.github.dsheirer.source.tuner.channel.TunerChannel;
import io.github.dsheirer.source.tuner.channel.TunerChannelSource;

import java.util.SortedSet;

/**
 * Interface to define the functionality of a source manager for handling tuner channel management, complex buffer
 * listeners, and source event listeners.
 */
public abstract class AbstractSourceManager implements IComplexBufferProvider, ISourceEventProcessor
{
    private Broadcaster<ComplexBuffer> mComplexBufferBroadcaster = new Broadcaster<>();
    private Broadcaster<SourceEvent> mSourceEventBroadcaster = new Broadcaster<>();

    /**
     * Sorted set of tuner channels being sourced by this source manager.  Set is ordered by frequency lowest to highest
     */
    public abstract SortedSet<TunerChannel> getTunerChannels();

    /**
     * Count of tuner channels being sourced by this source manager.
     * @return
     */
    public abstract int getTunerChannelCount();

    /**
     * Obtains a source for the tuner channel or returns null if the channel cannot be sourced by this tuner.
     *
     * Note: you MUST invoke start() on the obtained source to start the sample flow and invoke stop() to release all
     * resources allocated for the tuner channel source.
     *
     * @param tunerChannel for requested source
     * @return tuner channel source or null
     */
    public abstract TunerChannelSource getSource(TunerChannel tunerChannel);

    /**
     * Adds the listener to receive complex buffer samples
     */
    @Override
    public void addComplexBufferListener(Listener<ComplexBuffer> listener)
    {
        mComplexBufferBroadcaster.addListener(listener);
    }

    /**
     * Removes the listener from receiving complex buffer samples
     */
    @Override
    public void removeComplexBufferListener(Listener<ComplexBuffer> listener)
    {
        mComplexBufferBroadcaster.removeListener(listener);
    }

    /**
     * Indicates if there are any complex buffer listeners registered with this source manager
     */
    @Override
    public boolean hasComplexBufferListeners()
    {
        return mComplexBufferBroadcaster.hasListeners();
    }

    /**
     * Broadcasts the buffer to any registered listeners
     */
    protected void broadcast(ComplexBuffer complexBuffer)
    {
        mComplexBufferBroadcaster.broadcast(complexBuffer);
    }

    /**
     * Adds a listener to receive source events
     */
    public void addSourceEventListener(Listener<SourceEvent> listener)
    {
        mSourceEventBroadcaster.addListener(listener);
    }

    /**
     * Remove the listener from receiving source events
     */
    public void removeSourceEventListener(Listener<SourceEvent> listener)
    {
        mSourceEventBroadcaster.removeListener(listener);
    }

    /**
     * Broadcasts the source event to any registered listeners
     */
    protected void broadcast(SourceEvent sourceEvent)
    {
        mSourceEventBroadcaster.broadcast(sourceEvent);
    }
}
