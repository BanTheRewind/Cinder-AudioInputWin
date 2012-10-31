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

// Includes
#include "AudioInput.h"
#include "cinder/app/AppBasic.h"
#include "cinder/gl/TextureFont.h"
#include <fstream>
#include <iostream>

// Main application
class WavWriterSampleApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void draw();
	void keyUp( ci::app::KeyEvent event );
	void setup();
	void shutdown();
	
	// Receives input
	void onData( float *data, int32_t size );

	// Writes PCM buffer to file
	void writeData();

private:

	// Audio input
	float						*mData;
	AudioInputRef				mInput;

	// File writing
	std::ofstream				mFile;
	AudioInput::WAVFILEHEADER	mFileHeader;
	int16_t						*mPcmBuffer;
	int32_t						mPcmBufferPosition;
	int32_t						mPcmBufferSize;
	uint32_t					mPcmTotalSize;
	bool						mRecording;

	// WAV format
	int32_t						mBitsPerSample;
	int32_t						mChannelCount;
	int32_t						mSampleRate;

	// Text
	ci::gl::TextureFontRef		mFont;

};

// Imports
using namespace ci;
using namespace ci::app;
using namespace std;

// Draw
void WavWriterSampleApp::draw()
{

	// Clear screen
	gl::setMatricesWindow( getWindowSize() );
	gl::clear( Color::black() );

	// We have data
	if ( mData != 0 ) {

		// Get size of data
		int32_t dataSize = mInput->getDataSize();
		float dataSizef = (float)dataSize;

		// Get dimensions
		float scale = ( (float)getWindowWidth() - 20.0f ) / dataSizef;
		float windowHeight = (float)getWindowHeight();

		// Use polyline to depict audio
		PolyLine<Vec2f> line;

		// Set line color
		if ( mRecording ) {
			gl::color( ColorAf( 1.0f, 0.0f, 0.0f, 1.0f ) );
		} else {
			gl::color( ColorAf::white() );
		}

		// Iterate through data and populate line
		for ( int32_t i = 0; i < dataSize; i++ ) { 
			line.push_back( Vec2f( (float)i * scale + 10.0f, mData[ i ] * ( windowHeight - 20.0f ) * 0.25f + ( windowHeight * 0.5f + 10.0f ) ) );
		}

		// Draw signal
		gl::draw( line );

	}

	// Write instructions (scaling to improve text quality)
	gl::pushMatrices();
	gl::scale( 0.25f, 0.25f, 1.0f );
	mFont->drawString( "Press SPACE to start/stop recording", Vec2f( 20.0f, 580.0f ) * 4.0f );
	gl::popMatrices();

}

// Handles key press
void WavWriterSampleApp::keyUp( KeyEvent event )
{

	// Toggle recording with space bar
	if ( event.getCode() == KeyEvent::KEY_SPACE ) {

		// Currently recording
		if ( mRecording ) {

			// Save remaining data
			if ( mPcmBufferSize > 0 ) {
				writeData();
			}

			// Close file
			if ( mFile.is_open() ) {
				mFile.flush();
				mFile.close();
			}

		}
		else
		{

			// Set size values
			mPcmBufferPosition	= 0;
			mPcmTotalSize		= 0;

			// Open file for streaming
			mFile.open( getAppPath().generic_string() + "output.wav", ios::binary | ios::trunc );

		}

		// Toggle recording flag
		mRecording = !mRecording;

	}

}

// Called when buffer is full
void WavWriterSampleApp::onData( float *data, int32_t size )
{

	// Get float data
	mData = data;

	// Check recording flag and buffer size
	if ( mRecording && size <= mInput->getBufferLength() ) {

		// Save the PCM data to file and reset the array if we 
		// don't have room for this buffer
		if ( mPcmBufferPosition + size >= mPcmBufferSize ) {
			writeData();
		}

		// Copy PCM data to file buffer
		copy( mInput->getData(), mInput->getData() + size, mPcmBuffer + mPcmBufferPosition );

		// Update PCM position
		mPcmBufferPosition += size;

	}

}

// Set up
void WavWriterSampleApp::setup()
{

	// Set up window
	setFrameRate( 60 );
	setWindowSize( 600, 600 );

	// Set up line and text rendering
	gl::enable( GL_LINE_SMOOTH );
	glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
	gl::enable( GL_POLYGON_SMOOTH );
	glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );

	// Create font (we'll be scaling down the 
	// oversized text so it looks cleaner)
	mFont = gl::TextureFont::create( Font( "Tahoma", 96 ) );

	// Create audio input
	mInput = AudioInput::create();

	// Bail if no devices present
	if ( mInput->getDeviceCount() <= 0 ) {
		return;
	}

	// Start receiving audio
	mInput->addCallback( &WavWriterSampleApp::onData, this );
	mInput->start();

	// List devices
	DeviceList devices = mInput->getDeviceList();
	for ( DeviceList::const_iterator deviceIt = devices.begin(); deviceIt != devices.end(); ++deviceIt ) {
		console() << deviceIt->second << std::endl;
	}

	// Define audio settings
	mBitsPerSample	= sizeof(int16_t) * 8;
	mChannelCount	= 2;
	mSampleRate		= 44100;

	// Set up file stream
	mRecording = false;
	mPcmBufferSize = ( AudioInput::BUFFER_COUNT * mInput->getBufferLength() ) / sizeof( int16_t );
	mPcmBuffer = new int16_t[ mPcmBufferSize ];
	memset( mPcmBuffer, (int16_t)0, mPcmBufferSize );

	// Set up output file header
	mFileHeader.siz_wf			= mBitsPerSample;
	mFileHeader.wFormatTag		= WAVE_FORMAT_PCM;
	mFileHeader.nChannels		= mChannelCount;
	mFileHeader.nSamplesPerSec	= mSampleRate;
	mFileHeader.nAvgBytesPerSec	= mSampleRate * mChannelCount * sizeof( int16_t );
	mFileHeader.nBlockAlign		= mChannelCount * sizeof( int16_t );
	mFileHeader.wBitsPerSample	= mBitsPerSample;
	memcpy( mFileHeader.RIFF, "RIFF", 4 );
	memcpy( mFileHeader.WAVE, "WAVE", 4 );
	memcpy( mFileHeader.fmt,  "fmt ", 4 );
	memcpy( mFileHeader.data, "data", 4 );

	// Initialize buffer
	mData = 0;

}

// Called on exit
void WavWriterSampleApp::shutdown()
{

	// Turn off recording flag
	mRecording = false;

	// Close file
	if ( mFile.is_open() ) {
		mFile.flush();
		mFile.close();
	}

	// Stop input
	mInput->stop();

	// Free resources
	if ( mData != 0 ) {
		delete [] mData;
	}
	if ( mPcmBuffer != 0 ) {
		delete [] mPcmBuffer;
	}

}

// Append file buffer to output WAV
void WavWriterSampleApp::writeData()
{

	// Update header with new PCM length
	mPcmBufferPosition		*= sizeof( int16_t );
	mPcmTotalSize			+= mPcmBufferPosition;
	mFileHeader.bytes		= mPcmTotalSize + sizeof( AudioInput::WAVFILEHEADER );
	mFileHeader.pcmbytes	= mPcmTotalSize;
	mFile.seekp( 0 );
	mFile.write( reinterpret_cast<int_fast8_t*>( &mFileHeader ), sizeof( mFileHeader ) );

	// Append PCM data
	if ( mPcmBufferPosition > 0 ) {
		mFile.seekp( mPcmTotalSize - mPcmBufferPosition + sizeof( AudioInput::WAVFILEHEADER ) );
		mFile.write( reinterpret_cast<int_fast8_t*>( mPcmBuffer ), mPcmBufferPosition );
	}

	// Reset file buffer position
	mPcmBufferPosition = 0;

}

// Start application
CINDER_APP_BASIC( WavWriterSampleApp, RendererGl )
