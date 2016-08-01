// FileCaptureX.cs
// Extension methods for the FileCapture class
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

using System;
using System.IO;
using System.Linq;

using System.Runtime.Serialization;
using System.Reflection;

using DotImaging;

namespace monorfs
{
/// <summary>
/// Corrects the FileCapture constructor.
/// </summary>
public static class FileCaptureX
{
	private static readonly string[] supportedLocalFiles = new string[] { ".mp4", ".avi", ".divx", ".webm", ".wmv" };

	private static FileCapture EmptyConstructedFileCapture()
	{
		FileCapture fc = (FileCapture) FormatterServices.GetUninitializedObject(typeof (FileCapture));

		Console.WriteLine(fc);

		var baseconstructor = typeof (VideoCaptureBase).GetConstructor(BindingFlags.NonPublic | BindingFlags.Instance,
		                                                               null, new Type[] {}, null);
		baseconstructor.Invoke(fc, new object[] {});

		return fc;
	}

	/// <summary>
	/// Creates capture object from a local file.
	/// </summary>
	/// <returns>Capture object..</returns>
	/// <param name="fname">File name.</param>
	public static FileCapture CaptureLocalFile(string fname)
	{
		FileCapture fc = EmptyConstructedFileCapture();

		string fileExt = Path.GetExtension(fname);
		if (supportedLocalFiles.Any(x => x.Equals(fileExt.ToLower())) == false) {
			throw new UriFormatException(String.Format("File must be a supported video file ({0}).",
			                                           String.Join(", ", supportedLocalFiles)));
		}

		typeof (FileCapture).GetProperty("CanSeek").SetValue(fc, true);
		typeof (FileCapture).GetField("fileName", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(fc, fname);
		//fc.CanSeek = true;
		//fc.fileName = fname;

		fc.Open(); //to enable property change

		return fc;
	}
}
}
