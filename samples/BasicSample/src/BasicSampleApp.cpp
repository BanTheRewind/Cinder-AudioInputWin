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

#include "cinder/app/AppBasic.h"

#include "AudioInput.h"

class BasicSampleApp : public ci::app::AppBasic 
{
public:
	void			draw();
	void			setup();
	void			shutdown();
private:
	float			*mData;
	size_t			mDataSize;
	AudioInputRef	mInput;
};

using namespace ci;
using namespace ci::app;
using namespace std;

void BasicSampleApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear( ColorAf::black() );
	gl::setMatricesWindow( getWindowSize() );

	if ( mData != 0 ) {
		float scale	= ( (float)getWindowWidth() - 20.0f ) / (float)mDataSize;
		float h		= (float)getWindowHeight();

		gl::begin( GL_LINE_STRIP );
		for ( int32_t i = 0; i < mDataSize; ++i ) {
			gl::vertex( Vec2f( i * scale + 10.0f, mData[ i ] * ( h - 20.0f ) * 0.25f + ( h * 0.5f + 10.0f ) ) );
		}
		gl::end();
	}
}

void BasicSampleApp::shutdown()
{
	mInput->stop();
	if ( mData != 0 ) {
		delete [] mData;
	}
}

void BasicSampleApp::setup()
{
	setFrameRate( 60.0f );
	setWindowSize( 600, 600 );

	gl::enable( GL_LINE_SMOOTH );
	glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
	gl::color( ColorAf::white() );

	mData		= 0;
	mDataSize	= 0;

	mInput = AudioInput::create();
	if ( mInput->getDeviceCount() > 0 ) {
		return;
		mInput->connectEventHandler( [ = ]( float* data, size_t dataSize )
		{
			mData		= data;
			mDataSize	= dataSize;
		} );
		mInput->start();

		vector<string> devices = mInput->getDeviceNames();
		for ( const string& device : devices ) {
			console() << device << endl;
		}
	}
}

CINDER_APP_BASIC( BasicSampleApp, RendererGl )
