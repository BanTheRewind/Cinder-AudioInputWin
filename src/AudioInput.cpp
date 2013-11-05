/*
* 
* Copyright (c) 2013, Ban the Rewind
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

#include "AudioInput.h"

#include <ppl.h>

using namespace std;

template<typename T> 
std::shared_ptr<AudioInputT<T> > AudioInputT<T>::create( int32_t sampleRate, int32_t channelCount, int32_t bufferLength )
{
	return std::shared_ptr<AudioInputT<T> >( new AudioInputT<T>( sampleRate, channelCount, bufferLength ) );
}

template<typename T> 
AudioInputT<T>::AudioInputT( int32_t sampleRate, int32_t channelCount, int32_t bufferLength )
: mBitsPerSample( sizeof(T)* 8 ), mBuffer( 0 ), mBufferLength( bufferLength ), 
mBufferSize( 0 ), mBuffersRead( false ), mChannelCount( channelCount ), mDevice( 0 ), 
mDeviceCount( 0 ), mDeviceIndex( 0 ), mEventHandler( nullptr ), mLocale( std::locale( "" ) ), 
mNormalBuffer( 0 ), mReceiving( false ), mSampleRate( sampleRate ), mThread( 0 )
{
	getDeviceList();
}

template<typename T> 
AudioInputT<T>::~AudioInputT()
{
	stop();
	for ( size_t i = 0; i < mBuffers.size(); ++i ) {
		if ( mBuffers[ i ] != 0 ) {
			delete [] mBuffers[ i ];
			mBuffers[ i ] = 0;
		}
	}
	for ( size_t i = 0; i < mHeaders.size(); ++i ) {
		if ( mHeaders[ i ] != 0 ) {
			delete mHeaders[ i ];
			mHeaders[ i ] = 0;
		}
	}
	mBuffers.clear();
	mHeaders.clear();

	if ( mNormalBuffer != 0 ) {
		delete [] mNormalBuffer;
		mNormalBuffer = 0;
	}
}

template<typename T> 
int32_t AudioInputT<T>::getBitsPerSample() const
{ 
	return mBitsPerSample; 
}

template<typename T> 
int32_t AudioInputT<T>::getBufferLength() const
{ 
	return mBufferLength; 
}

template<typename T> 
int32_t AudioInputT<T>::getChannelCount() const
{ 
	return mChannelCount; 
}

template<typename T> 
T* AudioInputT<T>::getData() const
{ 
	return mBuffer; 
}

template<typename T> 
int32_t AudioInputT<T>::getDataSize() const
{ 
	return mHeadersize; 
}

template<typename T> 
int32_t AudioInputT<T>::getDeviceCount() const
{ 
	return mDeviceCount; 
}

template<typename T> 
const vector<string>& AudioInputT<T>::getDeviceNames() const
{
	size_t deviceCount = waveInGetNumDevs();
	if ( mDeviceCount != deviceCount ) {
		mDeviceCount = deviceCount;
		mDeviceNames.clear();
		for ( int32_t i = 0; i < deviceCount; ++i ) {
			WAVEINCAPS* device = new WAVEINCAPS();
			waveInGetDevCaps( (UINT_PTR)i, device, sizeof( WAVEINCAPS ) );
			int_fast8_t deviceName[ 32 ];
			memset( deviceName, 0, sizeof( deviceName ) );
			use_facet<ctype<wchar_t> >( mLocale ).narrow( device->szPname, device->szPname + wcslen( device->szPname ), 'X', &deviceName[ 0 ] );
			mDeviceNames.push_back( deviceName ) );
			delete device;
		}
	}
	return mDeviceList;
}

template<typename T> 
float* AudioInputT<T>::getNormalizedData() const
{ 
	return mNormalBuffer; 
}

template<typename T> 
int32_t AudioInputT<T>::getSampleRate() const
{ 
	return mSampleRate; 
}

template<typename T> 
bool AudioInputT<T>::isReceiving() const
{ 
	return mReceiving; 
}

template<typename T> 
void AudioInputT<T>::receiveMessage( tagMSG message )
{
	switch ( message.message ) {
	case MM_WIM_DATA:
		if ( mReceiving ) {
			if ( ( (wavehdr_tag*)message.lParam )->dwBytesRecorded ) {
				mBufferSize	= (size_t)( (unsigned long)( (wavehdr_tag*)message.lParam )->dwBytesRecorded / sizeof( T ) );
				mBuffer		= (T*)( (wavehdr_tag*)message.lParam )->lpData;
				
				if ( mBuffer != 0 ) {
					if ( mNormalBuffer == 0 ) {
						mNormalBuffer = new float[ mBufferSize ];
					}
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
							mNormalBuffer[ i ] = (float)( mBuffer[ i ] / ( 8.0 * 0x10000000 ) );
						} );
						break;
					}

					if ( mEventHandler != nullptr ) {
						mEventHandler( mNormalBuffer, mBufferSize );
					}
				}
			}
			::waveInAddBuffer( mDevice, ( (wavehdr_tag*)message.lParam ), sizeof( wavehdr_tag ) );
		} else {
			++mBuffersRead;
		}
		break;
	case MM_WIM_OPEN:
		mBuffersRead = 0;
		break;
	}
}

template<typename T> 
void AudioInputT<T>::setDevice( size_t index )
{
	if ( index < mDeviceCount && index != mDeviceIndex ) {
		bool receiving = mReceiving;
		if ( receiving ) { 
			stop();
		}
		mDeviceIndex = index;
		if ( receiving ) { 
			start();
		}
	}
}

template<typename T> 
void AudioInputT<T>::start()
{
	stop();
	if ( !mReceiving && mDeviceCount > 0 ) {
		tWAVEFORMATEX waveFormat;
		waveFormat.wFormatTag		= WAVE_FORMAT_PCM;
		waveFormat.nChannels		= mChannelCount;
		waveFormat.nSamplesPerSec	= mSampleRate;
		waveFormat.nAvgBytesPerSec	= mSampleRate * mChannelCount * sizeof( T );
		waveFormat.nBlockAlign		= mChannelCount * sizeof( T );
		waveFormat.wBitsPerSample	= mBitsPerSample;
		waveFormat.cbSize			= 0;

		mBuffer			= 0;
		mNormalBuffer	= 0;
		mReceiving		= true;

		unsigned long threadId;
		mThread = CreateThread( 0, 0, (LPTHREAD_START_ROUTINE)AudioInputT<T>::waveInProc, (void*)this, 0, &threadId );
		if ( !mThread ) { 
			return;
		}
		CloseHandle( mThread );

		if ( !success( waveInOpen( &mDevice, mDeviceIndex, &waveFormat, threadId, 0, CALLBACK_THREAD ) ) ) { 
			return;
		}

		mBuffersRead = 0;
		for ( int32_t i = 0; i < BUFFER_COUNT; ++i )  {
			mBuffers.push_back( new T[ mBufferLength * sizeof( T ) ] ) );
			mHeaders.push_back( new wavehdr_tag() );
			
			mHeaders[ i ]->dwBufferLength	= mBufferLength * sizeof( T );
			mHeaders[ i ]->lpData			= (LPSTR)mBuffers[ i ].mData;
			mHeaders[ i ]->dwBytesRecorded	= 0;
			mHeaders[ i ]->dwUser			= 0L;
			mHeaders[ i ]->dwFlags			= 0L;
			mHeaders[ i ]->dwLoops			= 0L;

			if ( !success( waveInPrepareHeader( mDevice, mHeaders[ i ], sizeof( wavehdr_tag ) ) ) ) { 
				return;
			}
			if ( !success( waveInAddBuffer( mDevice, mHeaders[ i ], sizeof( wavehdr_tag ) ) ) { 
				return;
			}
		}
		if ( !success( waveInStart( mDevice ) ) ) { 
			mReceiving = false;
		}
	}
}

template<typename T> 
void AudioInputT<T>::stop()
{
	if ( mReceiving ) {
		mReceiving = false;
		for ( int32_t i = 0; i < BUFFER_COUNT; ++i ) {
			waveInUnprepareHeader( (HWAVEIN)mDeviceIndex, (LPWAVEHDR)( &mHeaders[ i ] ), sizeof( wavehdr_tag ) );
		}
		if ( !success( waveInReset( (HWAVEIN)mDeviceIndex ) ) ) { 
			return;
		}
		if ( !success( waveInClose( (HWAVEIN)mDeviceIndex ) ) ) { 
			return;
		}
	}
}

template<typename T> 
bool AudioInputT<T>::success( MMRESULT hr )
{
	if ( hr ) {
		int_fast8_t err[ MESSAGE_BUFFER_SIZE ];
		err[ MESSAGE_BUFFER_SIZE - 1 ] = 0;
		::waveInGetErrorTextA( hr, err, MESSAGE_BUFFER_SIZE );
		OutputDebugStringA( err );
		OutputDebugStringA( "\n" );
		return false;
	}
	return true;
}

template<typename T> 
unsigned long __stdcall AudioInputT<T>::waveInProc( void far *arg )
{
	AudioInputT<T>* instance = (AudioInputT<T>*) arg;
	tagMSG message;
	if ( instance == 0 ) { 
		return 0;
	}
	while ( GetMessage( &message, 0, 0, 0 ) ) { 
		instance->receiveMessage( message );
	}
	return 0;
}

template class AudioInputT<uint8_t>;
template class AudioInputT<int16_t>;
template class AudioInputT<uint32_t>;
