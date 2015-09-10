// video2oni.cpp
// Multiple stream to ONI converter
// Part of MonoRFS
//
// Copyright (c) 2015, Angelo Falchetti
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * The names of its contributors may not be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ANGELO FALCHETTI BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifdef __linux__
	#define linux 1
#endif

#include <iostream>
#include <fstream>
#include <memory>
#include <cstring>
#include <map>
#include <cmath>
#include <cstdint>
#include <limits>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifdef __linux__
	#include <dirent.h>
#else
	#include <windows.h>
	#include <direct.h>
#endif

using namespace std;
using namespace cv;
using namespace xn;

string dirpath(string path)
{
#ifdef __linux__
	return path.substr(0, path.find_last_of('/') + 1);
#else
	return path.substr(0, path.find_last_of('\\') + 1);
#endif
}

int sizetype(int size)
{
	return (size == 8) ? CV_8UC1 : (size == 16) ? CV_16UC1 : (size == 24) ? CV_8UC3 : (size == 32) ? CV_8UC4 : -1;
}

void check(XnStatus retval, string message)
{
	if (retval != XN_STATUS_OK) {
		xnPrintError(retval, message.c_str());
	}
}

template<typename T>
class Stream
{
private:
	vector<T> data;
	int       width;
	int       height;
	int       length;
	double    timestamp;

public:
	class Iterator
	{
	private:
		Stream<T>* stream;
	public:
		Iterator(Stream<T>* stream) : stream(stream) {}

		bool       operator!=(const Iterator&) const { return !stream->finished();    }
		void       operator++()                const { stream->data = stream->next(); }
		Stream<T>* operator* ()                const { return stream;                 }
	};

	Stream()                                  : Stream(-1, -1, -1) {}
	Stream(int width, int height)             : Stream(width, height, -1) {}
	Stream(int width, int height, int length) : data(), width(width), height(height), length(length), timestamp(nan("")) {}

	const Iterator begin() { data = first(); return Iterator(this); }
	const Iterator end  () { return Iterator(this);                 }

	vector<T>    Data  ()    { return data;      }
	unsigned int Width ()    { return width;     }
	unsigned int Height()    { return height;    }
	unsigned int Length()    { return length;    }
	double       Timestamp() { return timestamp; }

protected:
	virtual vector<T> first   () = 0;
	virtual vector<T> next    () = 0;
	virtual bool      finished() = 0;
	
	void setWidth    (int    width)     { this->width     = width;     }
	void setHeight   (int    height)    { this->height    = height;    }
	void setLength   (int    length)    { this->length    = length;    }
	void setTimestamp(double timestamp) { this->timestamp = timestamp; }
};

using DepthStream = Stream<uint16_t>;
using RGBStream   = Stream<Vec3b>;

template<typename T>
class ImageStream final : Stream<T>
{
private:
	string         prefix;
	vector<string> filenames;
	vector<double> timestamps;
	int            i;
	int            type;

public:
	ImageStream(string path)
		: Stream<T>(), prefix(dirpath(path)), filenames(), timestamps(), i(0), type(0)
	{
		parsedeffile(path, filenames, timestamps);

		Mat firstimage = imread(prefix + filenames[0], CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

		this->setWidth (firstimage.cols);
		this->setHeight(firstimage.rows);
		this->setLength(filenames.size());
		type = sizetype(8 * sizeof (T));
	}

private:
	void parsedeffile(string defpath, vector<string>& filenames, vector<double>& timestamps)
	{
		ifstream deffile(defpath);
		filenames .clear();
		timestamps.clear();

		// map is always ordered by key; use it to sort filenames and timestamps
		map<double, string> entries;

		string line;
		while (getline(deffile, line)) {
			istringstream parser(line);
			double timestamp;
			string filename;

			if (!(parser >> timestamp >> filename)) {
				continue;  // ignore any malformed line, including comments
			}

			entries[timestamp] = filename;
		}

		for (auto entry : entries) {
			timestamps.push_back(entry.first);
			filenames .push_back(entry.second);
		}
	}

	vector<T> readFile(string path)
	{
		Mat image = imread(path, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

		if (image.cols != this->Width() || image.rows != this->Height() || image.type() != type) {
			CV_Error(1, "Incorrect image dimensions.");
		}

		if (type == CV_8UC3) {
			cvtColor(image, image, CV_BGR2RGB);
		}

		vector<T> data(image.rows * image.cols);
		Mat datamat(image.rows, image.cols, type, &data[0]);
		image.copyTo(datamat);

		return data;
	}

	virtual vector<T> first() override final 
	{
		i = 0;

		if (filenames.size() > 0) {
			this->setTimestamp(timestamps[i]);
			return readFile(prefix + filenames[i++]);
		}
		else {
			this->setTimestamp(nan(""));
			return vector<T>();
		}
	}

	virtual vector<T> next() override final 
	{
		if (i < (int) filenames.size()) {
			this->setTimestamp(timestamps[i]);
			return readFile(prefix + filenames[i++]);
		}
		else {
			this->setTimestamp(nan(""));
			return vector<T>();
		}
	}

	virtual bool finished() override final 
	{
		return !(i < (int) filenames.size());
	}
};

template<typename T>
class VideoStream final : Stream<T>
{
public:
	VideoStream(string path) : Stream<T>()
	{
	}

private:
	virtual vector<T> first() override final 
	{
		return vector<T>();
	}

	virtual vector<T> next() override final 
	{
		return vector<T>();
	}

	virtual bool finished() override final 
	{
		return true;
	}
};

template<typename T>
class MatrixStream final : Stream<T>
{
public:
	MatrixStream(string path) : Stream<T>()
	{
	}

private:
	virtual vector<T> first() override final 
	{
		return vector<T>();
	}

	virtual vector<T> next() override final 
	{
		return vector<T>();
	}

	virtual bool finished() override final 
	{
		return true;
	}
};

template<typename T>
class NullStream final : Stream<T>
{
public:
	NullStream() : Stream<T>() {}

private:
	virtual vector<T> first() override final 
	{
		return vector<T>();
	}

	virtual vector<T> next() override final 
	{
		return vector<T>();
	}

	virtual bool finished() override final 
	{
		return true;
	}
};

template<typename T>
unique_ptr<Stream<T>> readStream(char type, string path)
{
	unique_ptr<Stream<T>> stream;

	switch (type) {
	case 'i':
		cout << "reading image stream definition file " << path << endl;
		stream = unique_ptr<Stream<T>>((Stream<T>*) new ImageStream<T>(path));
		break;

	case 'v':
		stream = unique_ptr<Stream<T>>((Stream<T>*) new VideoStream<T>(path));
		break;

	case 'm':
		stream = unique_ptr<Stream<T>>((Stream<T>*) new MatrixStream<T>(path));
		break;

	default:
		stream = unique_ptr<Stream<T>>((Stream<T>*) new NullStream<T>());
	}

	return stream;
}

int main(int argc, char* argv[])
{
	if (argc != 4 && argc != 5) {
		cout << "usage: video2oni output.oni type=('i'|'v'|'m') depth.txt color.txt" << endl;
		return 1;
	}

	bool withrgb = (argc == 5);
	
	string outputfile(argv[1]);
	unique_ptr<DepthStream> depthstream = readStream<uint16_t>(argv[2][0], argv[3]);
	unique_ptr<RGBStream>   rgbstream   = readStream<Vec3b>(withrgb ? argv[2][0] : '\0', withrgb ? argv[4] : "");

	Context            context;
	Player             player;
	DepthGenerator     depth;
	ImageGenerator     rgb;
	MockDepthGenerator mockdepth;
	MockImageGenerator mockrgb;
	Recorder           recorder;
	XnStatus           ret;

	ret = context.Init();
	check(ret, "Init");

	ret = context.OpenFileRecording("./dummy.oni", player);
	check(ret, "OpenFileRecording");

	ret = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
	check(ret, "FindExistingNode Depth");

	ret = mockdepth.CreateBasedOn(depth);
	check(ret, "CreateBasedOn Depth");

	mockdepth.SetIntProperty(XN_PROP_DEVICE_MAX_DEPTH, numeric_limits<XnDepthPixel>::max());

	ret = context.FindExistingNode(XN_NODE_TYPE_IMAGE, rgb);
	check(ret, "FindExistingNode Color");

	ret = mockrgb.CreateBasedOn(rgb);
	check(ret, "CreateBasedOn Color");

	ret = recorder.Create(context, "oni");
	check(ret, "Create");

	NodeInfo nodeinfo = recorder.GetInfo();

	ret = context.CreateProductionTree(nodeinfo, recorder);
	check(ret, "CreateProductionTree");

	ret = recorder.SetDestination(XN_RECORD_MEDIUM_FILE, outputfile.c_str());
	check(ret, "SetDestination");
	
	ret = recorder.AddNodeToRecording(mockdepth);
	check(ret, "AddNodeToRecording Depth");

	if (withrgb) {
		ret = recorder.AddNodeToRecording(mockrgb);
		check(ret, "AddNodeToRecording Color");
	}

	unsigned int fps     = 30;
	int          frameid = 0;
	
	mockdepth.SetMapOutputMode(XnMapOutputMode { depthstream->Width(), depthstream->Height(), fps });
	mockrgb  .SetMapOutputMode(XnMapOutputMode { rgbstream  ->Width(), rgbstream  ->Height(), fps });
	
	auto depthit  = depthstream->begin(); auto depthend = depthstream->end();
	auto rgbit    = rgbstream  ->begin(); auto rgbend   = rgbstream  ->end();

	bool depthisnew = true;
	bool rgbisnew   = true;

	double firsttime = -1;

	while (depthit != depthend || rgbit != rgbend) {
		Stream<uint16_t>* depthframe = *depthit;
		Stream<Vec3b>*    rgbframe   = *rgbit;

		double depthtime = depthframe->Timestamp();
		double rgbtime   = rgbframe  ->Timestamp();
		double frametime = min(depthtime, rgbtime);

		// depth image extraction
		DepthMetaData    depthmetadata;
		vector<uint16_t> depthdata = depthframe->Data();
		ret = depthmetadata.ReAdjust(depthframe->Width(), depthframe->Height(), (XnDepthPixel*) (&depthdata[0]));
		check(ret, "ReAdjust Depth");

		depthmetadata.IsDataNew() = depthisnew;
		depthisnew = false;

		ret = depthmetadata.MakeDataWritable();
		check(ret, "MakeDataWritable Depth");

		ret = mockdepth.SetData(depthmetadata, frameid, (XnUInt64) (frametime * 1000000));
		check(ret, "SetData Depth");

		// rgb image extraction
		ImageMetaData rgbmetadata;
		vector<Vec3b> rgbdata = rgbframe->Data();
		ret = rgbmetadata.ReAdjust(rgbframe->Width(), rgbframe->Height(), XN_PIXEL_FORMAT_RGB24, (XnUInt8*) (&rgbdata[0]));
		check(ret, "ReAdjust Color");

		rgbmetadata.IsDataNew() = rgbisnew;
		rgbisnew = false;

		ret = rgbmetadata.MakeDataWritable();
		check(ret, "MakeDataWritable Color");

		ret = mockrgb.SetData(rgbmetadata, frameid, (XnUInt64) (frametime * 1000000));
		check(ret, "SetData Color");

		// recording
		ret = recorder.Record();
		check(ret, "Record");

		// sum of lengths is an approximate total; if they coincide anywhere, it won't be correct, but it's good enough
		cout << "frame " << (frameid + 1) << " / " << depthstream->Length() + rgbstream->Length() << endl;
		frameid++;

		if (depthtime <= rgbtime) {
			++depthit;
			depthisnew = true;
		}

		if (rgbtime <= depthtime) {
			++rgbit;
			rgbisnew = true;
		}
	}

	ret = recorder.RemoveNodeFromRecording(mockdepth);
	check(ret, "RemoveNodeFromRecording Depth");

	if (withrgb) {
		ret = recorder.RemoveNodeFromRecording(mockrgb);
		check(ret, "RemoveNodeFromRecording Color");
	}

	player   .Release();
	mockdepth.Release();
	mockrgb  .Release();
	recorder .Release();
	context  .Release();

	return 0;
}