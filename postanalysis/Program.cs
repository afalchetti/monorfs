// Program.cs
// Main entry to the plotting utility
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
using NDesk.Options;
using System.IO;

namespace postanalysis
{

class Program
{
	/// <summary>
	/// Show a description of the program and its parameters to the user.
	/// </summary>
	private static void ShowHelp(OptionSet options)
	{
		Console.WriteLine("MonoRFS");
		Console.WriteLine("Showcase a Rao-Blackwellized PHD SLAM navigator.");
		Console.WriteLine("If no options are specified a simulation of the full slam algorithm is run with one particle.");
		Console.WriteLine();
		Console.WriteLine("Options:");
		options.WriteOptionDescriptions(Console.Out);

	}

	public static void Main(string[] args)
	{
		string   file      = "data.zip";
		double   ospac     = 1;
		double   ospap     = 1;
		HistMode histmode  = HistMode.Timed;
		bool     showhelp  = false;

		OptionSet options = new OptionSet {
			{ "f|file=",    "Recording file.",                                               f          => file      = f },
			{ "c=",         "OSPA C parameter.",                                             (double c) => ospac     = c },
			{ "p=",         "OSPA P parameter",                                              (double p) => ospap     = p },
			{ "H|history=", "Trajectory history mode: either 'filter', 'smooth' or 'timed'", h          => histmode  = (h == "filter") ? HistMode.Filter : (h == "smooth") ? HistMode.Smooth : HistMode.Timed },
			{ "h|help",     "Show this message and exit",                                    h          => showhelp  = h != null }
		};

		try {
			options.Parse(args);
		}
		catch (OptionException e) {
			Console.Write("monorfs: ");
			Console.WriteLine(e.Message);
			Console.WriteLine("Try monorfs --help for more information.");
			Environment.Exit(1);
		}

		if (showhelp) {
			ShowHelp(options);
			return;
		}

		Plot plot = Plot.FromFiles(file, histmode, ospac, ospap);

		File.WriteAllText(file + ".loc.data",      plot.SerializedLocationError);
		File.WriteAllText(file + ".rot.data",      plot.SerializedRotationError);
		File.WriteAllText(file + ".odoloc.data",   plot.SerializedOdoLocationError);
		File.WriteAllText(file + ".odorot.data",   plot.SerializedOdoRotationError);
		File.WriteAllText(file + ".map.data",      plot.SerializedMapError);
		File.WriteAllText(file + ".size.data",     plot.SerializedExpectedSize);
		File.WriteAllText(file + ".realsize.data", plot.SerializedCorrectSize);
		File.WriteAllText(file + ".pathlen.data",  plot.SerializedPathLength);
		File.WriteAllText(file + ".tags.data",     plot.SerializedTags);
	}
}
}
