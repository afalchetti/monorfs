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
using System.IO.Compression;
using System.IO;
using System.Text;

using NDesk.Options;

using monorfs;
using System.Collections.Generic;

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

	/// <summary>
	/// The entry point of the program, where the program control starts and ends.
	/// </summary>
	/// <param name="args">The command-line arguments.</param>
	public static void Main(string[] args)
	{
		string   file      = "data.zip";
		double   ospac     = 1;
		double   ospap     = 1;
		double   reftime   = 0;
		HistMode histmode  = HistMode.Timed;
		bool     showhelp  = false;

		OptionSet options = new OptionSet {
			{ "f|file=",    "Recording file.",                                                f          => file      = f },
			{ "c=",         "OSPA C parameter.",                                              (double c) => ospac     = c },
			{ "p=",         "OSPA P parameter.",                                              (double p) => ospap     = p },
			{ "t|reftime=", "Reference time.",                                                (double t) => reftime   = t },
			{ "H|history=", "Trajectory history mode: either 'filter', 'smooth' or 'timed'.", h          => histmode  = (h == "filter") ? HistMode.Filter : (h == "smooth") ? HistMode.Smooth : HistMode.Timed },
			{ "h|help",     "Show this message and exit.",                                    h          => showhelp  = h != null }
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

		try {
			Config.FromRecordFile(file);
		}
		catch (IOException) {
			Console.WriteLine("Couldn't open the recording file.");
			Environment.Exit(2);
		}

		try {
		switch (Config.Model) {
		case DynamicsModel.Linear2D:
			Run(Plot<Linear2DMeasurer, LinearPose2D, LinearMeasurement2D>.
			        FromFiles(file, histmode, ospac, ospap, reftime), file);
			break;
		
		case DynamicsModel.PRM3D:
		default:
			Run(Plot<PRM3DMeasurer, Pose3D, PixelRangeMeasurement>.
			        FromFiles(file, histmode, ospac, ospap, reftime), file);
			break;
		}
		}
		catch (FileNotFoundException e) {
			Console.WriteLine("Error: File '" + e.FileName + "' not found.");
			Environment.Exit(3);
		}
	}

	/// <summary>
	/// Calculate all the relevant statistics and write them to files.
	/// </summary>
	/// <param name="plot">Plot object.</param>
	/// <param name="file">Input file name; output file names will be derived from this.</param>
	private static void Run<MeasurerT, PoseT, MeasurementT>(Plot<MeasurerT, PoseT, MeasurementT> plot, string file)
		where PoseT        : IPose<PoseT>, new()
		where MeasurementT : IMeasurement<MeasurementT>, new()
		where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
	{
		File.WriteAllText(file + ".loc.data",         plot.SerializedLocationError);
		File.WriteAllText(file + ".rot.data",         plot.SerializedRotationError);
		File.WriteAllText(file + ".odoloc.data",      plot.SerializedOdoLocationError);
		File.WriteAllText(file + ".odorot.data",      plot.SerializedOdoRotationError);
		File.WriteAllText(file + ".map.data",         plot.SerializedMapError);
		File.WriteAllText(file + "-spatial.map.data", plot.SerializedSpatialMapError);
		File.WriteAllText(file + ".size.data",        plot.SerializedExpectedSize);
		File.WriteAllText(file + ".realsize.data",    plot.SerializedCorrectSize);
		File.WriteAllText(file + ".pathlen.data",     plot.SerializedPathLength);
		File.WriteAllText(file + ".tags.data",        plot.SerializedTags);
	}
}
}
