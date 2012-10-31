/*
* 
* Copyright (c) 2012, Ban the Rewind
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

// Include header
#include "AudioInput.h"

using namespace ci;
using namespace std;

// Creates pointer to instance
template<typename T> 
std::shared_ptr<AudioInputT<T> > AudioInputT<T>::create( int32_t sampleRate, int32_t channelCount, int32_t bufferLength )
{
	return std::shared_ptr<AudioInputT<T> >( new AudioInputT<T>( sampleRate, channelCount, bufferLength ) );
}

// Constructor
// NOTE: Make this public to build instance
// directly on the stack (advanced)
template<typename T> 
AudioInputT<T>::AudioInputT( int32_t sampleRate, int32_t channelCount, int32_t bufferLength )
{

	// Initialize flag
	mReceiving = false;

	// Set buffer length
	mBufferLength = bufferLength;

	// Set parameters
	mBitsPerSample = sizeof( T ) * 8;
	mChannelCount = channelCount;
	mSampleRate = sampleRate;

	// Initialize device list
	mDeviceId = 0;
	mDeviceCount = -1;
	mLocale = std::locale( "" ); // Uses system's default language for UTF encoding

	// Initialize device list
	getDeviceList();

}

// Destructor
template<typename T> 
AudioInputT<T>::~AudioInputT()
{
	// Stop
	if ( mReceiving ) {
		stop();
	}

	// Disconnect signals
	for ( CallbackList::iterator callbackIt = mCallbacks.begin(); callbackIt != mCallbacks.end(); ++callbackIt ) {
		if ( callbackIt->second->connected() ) {
			callbackIt->second->disconnect();
		}
	}

	// Clear vectors
	mCallbacks.clear();
	mHeaderBuffers.clear();
	mInputBuffers.clear();
}

// Check for and print error
template<typename T> 
bool AudioInputT<T>::error()
{

	// Error occurred
	if ( mResultHnd ) {

		// Report error
		memset( mError, 0, MESSAGE_BUFFER_SIZE );
		::waveInGetErrorTextA( mResultHnd, mError, MESSAGE_BUFFER_SIZE );
		OutputDebugStringA( mError );
		OutputDebugStringA( "\n" );
		mResultHnd = 0;
		return true;

	}

	// No error
	return false;

}

// Retrieve device list
template<typename T> 
DeviceList AudioInputT<T>::getDeviceList()
{

	// Get device count
	int32_t deviceCount = (int32_t)::waveInGetNumDevs();

	// Skip routine if device count hasn't changed
	if ( mDeviceCount != deviceCount ) {

		// Update device count
		mDeviceCount = deviceCount;

		// Build new list
		mDeviceList.clear();
		for ( int32_t i = 0; i < mDeviceCount; i++ ) {

			// Get device
			WAVEINCAPS* device = new WAVEINCAPS();
			::waveInGetDevCaps( (UINT_PTR)i, device, sizeof( WAVEINCAPS ) );

			// Get device name
			memset( mDeviceName, 0, sizeof( mDeviceName ) );
			use_facet<ctype<wchar_t> >( mLocale ).narrow( device->szPname, device->szPname + wcslen( device->szPname ), 'X', & mDeviceName[ 0 ] );

			// Add device to list
			mDeviceList.insert( std::make_pair( i, string( mDeviceName ) ) );

			// Clean up
			delete device;

		}

	}

	// Return list
	return mDeviceList;

}

// Normalize audio buffer
template<typename T> 
float* AudioInputT<T>::getNormalBuffer()
{

	// Bail if there's no data to convert
	if ( mBuffer == 0 ) {
		return 0;
	}

	// Create buffer, if needed
	if ( mNormalBuffer == 0 ) {
		mNormalBuffer = new float[ mBufferSize ];
	}

	// Normalize data for analysis
	switch ( mBitsPerSample ) {
	case 8: 
		Concurrency::parallel_for( 0, mBufferSize, [ = ]( int32_t i ) {
			mNormalBuffer[ i ] = (float)( mBuffer[ i ] / ( 1.0 * 0x80 ) - 1.0 );
		} );
		break;
	case 16: 
		Concurrency::parallel_for( 0, mBufferSize, [ = ]( int32_t i ) {
			mNormalBuffer[ i ] = (float)( mBuffer[ i ] / ( 1.0 * 0x8000 ) );
		} );
		break;
	case 32: 
		Concurrency::parallel_for( 0, mBufferSize, [ = ]( int32_t i ) {
			mNormalBuffer[ i ] = (float)( mBuffer[ i ] / (8.0 * 0x10000000 ) );
		} );
		break;
	}

	// Return normalized buffer
	return mNormalBuffer;

}

// Receives buffer from multimedia API
template<typename T> 
void AudioInputT<T>::receiveMessage( tagMSG message )
{

	// Read message
	switch ( message.message ) {
	case MM_WIM_DATA:

		// Check receiving flag
		if ( mReceiving ) {

			// Check for pointer to wave header
			if ( ( (wavehdr_tag*)message.lParam )->dwBytesRecorded ) {

				// Update buffer
				mBufferSize = (int32_t)( (unsigned long)( (wavehdr_tag*)message.lParam )->dwBytesRecorded / sizeof( T ) );
				mBuffer = (T*)( (wavehdr_tag*)message.lParam )->lpData;

				// Execute callbacks
				mSignal( getNormalBuffer(), mBufferSize );

			}

			// Re-use buffer
			::waveInAddBuffer( mDeviceHnd, ( (wavehdr_tag*)message.lParam ), sizeof( wavehdr_tag ) );

		} else {

			// Mark buffer complete
			++mBuffersComplete;

		}

		break;
	case MM_WIM_OPEN:

		// Reset complete count if main thread is opening the device
		mBuffersComplete = 0;

		break;

	}

}

// Remove callback by ID
template<typename T> 
void AudioInputT<T>::removeCallback( int32_t callbackID )
{

	// Disconnect the callback connection
	mCallbacks.find( callbackID )->second->disconnect();

	// Remove the callback from the list
	mCallbacks.erase( callbackID ); 

}

// Select device
template<typename T> 
void AudioInputT<T>::setDevice( int32_t deviceID )
{

	// New device ID must be less than the number of devices
	// and different from current ID
	if ( deviceID >= 0 && deviceID < mDeviceCount && deviceID != mDeviceId ) {

		// Stop input if we're currently receiving
		bool receiving = mReceiving;
		if ( receiving ) { 
			stop();
		}

		// Switch device ID 
		mDeviceId = deviceID;

		// Restart if we were receiving audio
		if ( receiving ) { 
			start();
		}

	}

}

// Start audio input
template<typename T> 
void AudioInputT<T>::start()
{

	// Stop playback if we're already receiving
	if ( mReceiving ) {
		stop();
	}

	// Check receiving flag
	if ( !mReceiving ) {

		// Set up PCM format
		mWavFormat.wFormatTag		= WAVE_FORMAT_PCM;
		mWavFormat.nChannels		= mChannelCount;
		mWavFormat.nSamplesPerSec	= mSampleRate;
		mWavFormat.nAvgBytesPerSec	= mSampleRate * mChannelCount * sizeof( T );
		mWavFormat.nBlockAlign		= mChannelCount * sizeof( T );
		mWavFormat.wBitsPerSample	= mBitsPerSample;
		mWavFormat.cbSize = 0;

		// Set flag
		mReceiving		= true;

		// Initialize buffers
		mBuffer			= 0;
		mNormalBuffer	= 0;

		// Start callback thread
		unsigned long threadID;
		mWaveInThread = CreateThread( 0, 0, (LPTHREAD_START_ROUTINE)AudioInputT<T>::waveInProc, (PVOID)this, 0, & threadID );
		if ( !mWaveInThread ) { 
			return;
		}
		CloseHandle( mWaveInThread );

		// Open input device
		mResultHnd = ::waveInOpen( & mDeviceHnd, mDeviceId, & mWavFormat, threadID, 0, CALLBACK_THREAD );
		if ( error() ) { 
			return;
		}

		// Prepare buffers
		mBuffersComplete = 0;
		for ( int32_t i = 0; i < BUFFER_COUNT; i++ )  {

			// Create buffers
			mHeaderBuffers.push_back( new T[ mBufferLength * sizeof( T ) ] );
			mInputBuffers.push_back( new wavehdr_tag() );

			// Set up header
			mInputBuffers[ i ]->dwBufferLength	= mBufferLength * sizeof( T );
			mInputBuffers[ i ]->lpData			= (LPSTR)mHeaderBuffers[ i ];
			mInputBuffers[ i ]->dwBytesRecorded	= 0;
			mInputBuffers[ i ]->dwUser			= 0L;
			mInputBuffers[ i ]->dwFlags			= 0L;
			mInputBuffers[ i ]->dwLoops			= 0L;

			// Add buffer to input device
			mResultHnd = ::waveInPrepareHeader( mDeviceHnd, mInputBuffers[ i ], sizeof( wavehdr_tag ) );
			if ( error() ) { 
				return;
			}
			mResultHnd = ::waveInAddBuffer( mDeviceHnd, mInputBuffers[ i ], sizeof( wavehdr_tag ) );
			if ( error() ) { 
				return;
			}

		}

		// Start input
		mResultHnd = ::waveInStart( mDeviceHnd );
		if ( error() ) { 
			mReceiving = false;
		}

	}

}

// Stop audio input
template<typename T> 
void AudioInputT<T>::stop()
{

	// Check receiving flag
	if ( mReceiving ) {

		// Turn off flags
		mReceiving = false;

		// Release buffers
		for ( int32_t i = 0; i < BUFFER_COUNT; i++ ) {
			mResultHnd = ::waveInUnprepareHeader( (HWAVEIN)mDeviceId, (LPWAVEHDR)(& ( mInputBuffers[ i ] ) ), sizeof( wavehdr_tag ) );
		}

		// Close input device
		mResultHnd = ::waveInReset( (HWAVEIN)mDeviceId );
		if ( error() ) { 
			return;
		}
		mResultHnd = ::waveInClose( (HWAVEIN)mDeviceId );
		if ( error() ) { 
			return;
		}

	}

}

// Multimedia API callback
template<typename T> 
unsigned long __stdcall AudioInputT<T>::waveInProc( void far *arg )
{

	// Get instance
	AudioInputT<T>* instance = (AudioInputT<T>*) arg;

	// Get message from thread
	tagMSG message;

	// Bail if instance not available
	if ( instance == 0 ) { 
		return 0;
	}

	// Get message from thread
	while ( GetMessage( & message, 0, 0, 0 ) ) { 
		instance->receiveMessage( message );
	}

	// Return
	return 0;

}

// Template implementations
template class AudioInputT<uint8_t>;
template class AudioInputT<int16_t>;
template class AudioInputT<uint32_t>;
