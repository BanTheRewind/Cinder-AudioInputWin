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

#pragma once

#include <functional>
#include <map>
#include "mmsystem.h"
#include <string>
#include <vector>
#include <utility>
#include "Windows.h"

#ifndef DRV_RESERVED
#define DRV_RESERVED					0×0800
#endif
#ifndef DRV_QUERYDEVICEINTERFACE
#define DRV_QUERYDEVICEINTERFACE		( DRV_RESERVED + 12 )
#endif
#ifndef DRV_QUERYDEVICEINTERFACESIZE
#define DRV_QUERYDEVICEINTERFACESIZE	( DRV_RESERVED + 13 )
#endif

template<typename T> 
class AudioInputT
{
public:
	static const size_t BUFFER_COUNT = 32;

	//! Header for WAV file
	typedef struct
	{
		int_fast8_t							RIFF[ 4 ];
		unsigned long						bytes;
		int_fast8_t							WAVE[ 4 ];
		int_fast8_t							fmt[ 4 ];
		int32_t								siz_wf;
		uint16_t							wFormatTag;
		uint16_t							nChannels;
		unsigned long						nSamplesPerSec;
		unsigned long						nAvgBytesPerSec;
		uint16_t							nBlockAlign;
		uint16_t							wBitsPerSample;
		int_fast8_t							data[ 4 ];
		unsigned long						pcmbytes;
	} WAVFILEHEADER;

	//! Creates audio input instance.
	static std::shared_ptr<AudioInputT<T> >	create( size_t sampleRate = 44100, size_t channelCount = 2, 
													size_t bufferLength = BUFFER_LENGTH );
	~AudioInputT();

	//! Starts audio input.
	void									start();
	//! Stops audio input.
	void									stop();

	/*! Sets buffer event handler. \a eventHandler has the signature \a void(float*, size_t). 
        \a obj is the instance receiving the event. */
	template<typename T, typename Y> 
    inline void								connectEventHandler( T eventHandler, Y* obj )
    {
            connectEventHandler( std::bind( eventHandler, obj, std::placeholders::_1, std::placeholders::_2 ) );
    }

	//! Sets buffer event callback to \a eventHandler.
	void									connectEventHandler( const std::function<void( float*, size_t )>& eventHandler );

	size_t									getBitsPerSample() const;
	size_t									getBufferLength() const;
	size_t									getChannelCount() const;
	T*										getData() const;
	size_t									getDataSize() const;
	size_t									getDeviceCount() const;
	const std::vector<std::string>&			getDeviceNames() const;
	float*									getNormalizedData() const;
	size_t									getSampleRate() const;
	bool									isReceiving() const;

	void									setDevice( size_t index );

	// Multimedia API callback
	static unsigned long __stdcall			waveInProc( void far* arg );
protected:
	static const size_t						BUFFER_LENGTH		= 1024;
	static const size_t						MESSAGE_BUFFER_SIZE	= 256;

	AudioInputT( size_t sampleRate = 44100, size_t channelCount = 2, size_t bufferLength = BUFFER_LENGTH );

	size_t									mBitsPerSample;
	T*										mBuffer;
	size_t									mBufferLength;
	std::vector<T*>							mBuffers;
	size_t									mBufferSize;
	uint_least8_t							mBuffersRead;
	size_t									mChannelCount;
	HWAVEIN									mDevice;
	size_t									mDeviceCount;
	size_t									mDeviceIndex;
	std::vector<std::string>				mDeviceNames;
	std::function<void( float*, size_t )>	mEventHandler;
	std::locale								mLocale;
	std::vector<wavehdr_tag*>				mHeaders;
	float*									mNormalBuffer;
	bool									mReceiving;
	size_t									mSampleRate;
	void*									mThread;

	void									receiveMessage( tagMSG message );
	bool									success( MMRESULT hr );
};

typedef AudioInputT<uint8_t>				AudioInput8;
typedef AudioInputT<int16_t>				AudioInput16;
typedef AudioInputT<uint32_t>				AudioInput32;
typedef AudioInput16						AudioInput;
typedef std::shared_ptr<AudioInput8>        AudioInputRef8;
typedef std::shared_ptr<AudioInput16>		AudioInputRef16;
typedef std::shared_ptr<AudioInput32>		AudioInputRef32;
typedef AudioInputRef16						AudioInputRef;
