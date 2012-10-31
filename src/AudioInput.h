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

#pragma once

// Includes
#include <algorithm>
#include "boost/signals2.hpp"
#include "cinder/app/App.h"
#include "MMSystem.h"
#include "ppl.h"
#include <utility>
#include <vector>
#include "windows.h"

// Define device query functions if WDK is not present
#ifndef DRV_RESERVED
#define DRV_RESERVED					0×0800
#endif
#ifndef DRV_QUERYDEVICEINTERFACE
#define DRV_QUERYDEVICEINTERFACE		( DRV_RESERVED + 12 )
#endif
#ifndef DRV_QUERYDEVICEINTERFACESIZE
#define DRV_QUERYDEVICEINTERFACESIZE	( DRV_RESERVED + 13 )
#endif

// Device map alias
typedef std::map<int32_t, std::string> DeviceList;

// Audio input for Windows
template<typename T> 
class AudioInputT
{

public:

	// Constants
	static const uint32_t BUFFER_COUNT = 32;

	// Header for WAV file
	typedef struct
	{
		int_fast8_t		RIFF[ 4 ];
		unsigned long	bytes;
		int_fast8_t		WAVE[ 4 ];
		int_fast8_t		fmt[ 4 ];
		int32_t			siz_wf;
		uint16_t		wFormatTag;
		uint16_t		nChannels;
		unsigned long	nSamplesPerSec;
		unsigned long	nAvgBytesPerSec;
		uint16_t		nBlockAlign;
		uint16_t		wBitsPerSample;
		int_fast8_t		data[ 4 ];
		unsigned long	pcmbytes;
	} WAVFILEHEADER;

	// Creates pointer to instance
	static std::shared_ptr<AudioInputT<T> > create( int32_t sampleRate = 44100, int32_t channelCount = 2, 
													int32_t bufferLength = BUFFER_LENGTH );

	// Destructor
	~AudioInputT();

	// Start audio input
	void start();
	void stop();

	// Callbacks
	template<typename T, typename Y> 
	inline uint32_t	addCallback( T callback, Y *callbackObject )
	{
		uint32_t id = mCallbacks.empty() ? 0 : mCallbacks.rbegin()->first + 1;
		mCallbacks.insert( std::make_pair( id, CallbackRef( new Callback( mSignal.connect( std::bind( callback, callbackObject, std::placeholders::_1, std::placeholders::_2 ) ) ) ) ) );
		return id;
	}
	void		removeCallback( int32_t callbackID );

	// Check for and print error
	bool		error();

	// Getters
	int32_t		getBitsPerSample() 
	{ 
		return mBitsPerSample; 
	}
	int32_t		getBufferLength() 
	{ 
		return mBufferLength; 
	}
	int32_t		getChannelCount() 
	{ 
		return mChannelCount; 
	}
	T*			getData() 
	{ 
		return mBuffer; 
	}
	int32_t		getDataSize() 
	{ 
		return mBufferSize; 
	}
	int32_t		getDeviceCount() 
	{ 
		return mDeviceCount; 
	}
	DeviceList	getDeviceList();
	float*		getNormalizedData() 
	{ 
		return mNormalBuffer; 
	}
	int32_t		getSampleRate() 
	{ 
		return mSampleRate; 
	}
	bool	isReceiving() 
	{ 
		return mReceiving; 
	}

	// Setters
	void	setDevice( int32_t deviceId );

	// Multimedia API callback
	static unsigned long __stdcall waveInProc( void far *arg );

private:

	// Constants
	static const uint32_t BUFFER_LENGTH			= 1024;
	static const uint32_t MESSAGE_BUFFER_SIZE	= 256;

	// Callback aliases
	typedef boost::signals2::connection			Callback;
	typedef std::shared_ptr<Callback>			CallbackRef;
	typedef std::map<int32_t, CallbackRef>		CallbackList;

	// Constructor
	// NOTE: Make this public to build instance
	// directly on the stack (advanced)
	AudioInputT( int32_t sampleRate = 44100, int32_t channelCount = 2, int32_t bufferLength = BUFFER_LENGTH );

	// Flags
	bool			mReceiving;

	// The current audio buffer
	int32_t			mBufferLength;
	T				*mBuffer;
	uint_least8_t	mBuffersComplete;
	int32_t			mBufferSize;
	void			receiveMessage( tagMSG message );

	// Buffer for analyzing audio
	float			*getNormalBuffer();
	float			*mNormalBuffer;

	// Windows multimedia API
	std::vector<T*>				mHeaderBuffers;
	std::vector<wavehdr_tag*>	mInputBuffers;
	MMRESULT					mResultHnd;
	void						*mWaveInThread;
	tWAVEFORMATEX				mWavFormat;

	// WAV format
	int32_t						mBitsPerSample;
	int32_t						mChannelCount;
	int32_t						mSampleRate;

	// Callback list
	boost::signals2::signal<void ( float*, int32_t )>	mSignal;
	CallbackList										mCallbacks;

	// Device list
	int32_t						mDeviceCount;
	HWAVEIN						mDeviceHnd;
	int32_t						mDeviceId;
	DeviceList					mDeviceList;
	int_fast8_t					mDeviceName[ 32 ];
	std::locale					mLocale;

	// Error message buffer
	int_fast8_t					mError[ MESSAGE_BUFFER_SIZE ];

};

// Aliases
typedef AudioInputT<uint8_t>			AudioInput8;
typedef AudioInputT<int16_t>			AudioInput16;
typedef AudioInputT<uint32_t>			AudioInput32;
typedef AudioInput16					AudioInput;
typedef std::shared_ptr<AudioInput8>	AudioInputRef8;
typedef std::shared_ptr<AudioInput16>	AudioInputRef16;
typedef std::shared_ptr<AudioInput32>	AudioInputRef32;
typedef AudioInputRef16					AudioInputRef;
