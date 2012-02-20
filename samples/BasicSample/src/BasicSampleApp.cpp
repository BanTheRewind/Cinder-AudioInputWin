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
#include <AudioInput.h>
#include <cinder/app/AppBasic.h>

// Main application
class BasicSampleApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void draw();
	void quit();
	void setup();

	// Audio callback
	void onData(float * data, int32_t size);

private:

	// Audio input
	AudioInputRef mInput;
	float * mData;

};

// Imports
using namespace ci;
using namespace ci::app;

// Draw
void BasicSampleApp::draw()
{

	// Clear screen
	gl::clear(ColorAf::black());

	// We have audio data
	if (mData != 0)
	{

		// Get size of data
		int32_t mDataSize = mInput->getDataSize();

		// Get dimensions
		float mScale = ((float)getWindowWidth() - 20.0f) / (float)mDataSize;
		float mWindowHeight = (float)getWindowHeight();

		// Use polyline to depict audio
		PolyLine<Vec2f> mLine;

		// Iterate through data and populate line
		for (int32_t i = 0; i < mDataSize; i++) 
			mLine.push_back(Vec2f(i * mScale + 10.0f, mData[i] * (mWindowHeight - 20.0f) * 0.25f + (mWindowHeight * 0.5f + 10.0f)));

		// Draw signal
		gl::draw(mLine);

	}

}

// Called when buffer is full
void BasicSampleApp::onData(float * data, int32_t size)
{

	// Get data
	mData = data;

}

// Called on exit
void BasicSampleApp::quit()
{

	// Stop input
	mInput->stop();

	// Free resources
	if (mData != NULL)
		delete [] mData;

}

// Set up
void BasicSampleApp::setup()
{

	// Set up window
	setFrameRate(60.0f);
	setWindowSize(600, 600);

	// Set up line rendering
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	gl::color(ColorAf::white());

	// Initialize array
	mData = 0;

	// Start receiving audio
	mInput = AudioInput::create();
	mInput->addCallback<BasicSampleApp>(& BasicSampleApp::onData, this);
	mInput->start();

}

// Start application
CINDER_APP_BASIC(BasicSampleApp, RendererGl)
