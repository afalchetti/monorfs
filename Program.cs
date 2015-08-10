﻿// Program.cs
// Main entry point to the program
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
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Runtime.InteropServices;
using Accord.Extensions.Imaging;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using NDesk.Options;

namespace monorfs
{
/// <summary>
/// Main program class.
/// </summary>
public class Program
{
	/// <summary>
	/// Save a stream of image frames as an AVI video file.
	/// </summary>
	/// <param name="frames">Ordered list of frames at 30 fps.</param>
	/// <param name="file">Output filename.</param>
	public static void SaveAsAvi(List<Texture2D> frames, string file)
	{
		if (frames.Count == 0) { return; }

		int width  = frames[0].Width;
		int height = frames[0].Height;

		using (VideoWriter writer = new VideoWriter(file, new Accord.Extensions.Size(width, height),
		                                            30, true, VideoCodec.FromName("MJPG"))) {
			writer.Open();

			foreach (Texture2D frame in frames) {
				Bgr<byte>[,] bitmap = new Bgr<byte>[height, width];
				Color[]      linear = new Color[width * height];

				frame.GetData(linear);

				int h = 0; 
				for (int k = 0; k < height; k++) {
				for (int i = 0; i < width;  i++) {
					bitmap[k, i] = new Bgr<byte>(linear[h].B, linear[h].G, linear[h].R);
					h++;
				}
				}

				writer.Write(bitmap.Lock());
			}

			writer.Close();
		}
	}

	/// <summary>
	/// Show a description of the program and its parameters to the user.
	/// </summary>
	private static void ShowHelp(OptionSet options)
	{
		Console.WriteLine("Usage: monorfs ");
		Console.WriteLine("Showcase a Rao-Blackwellized PHD SLAM navigator.");
		Console.WriteLine("If no options are specified a simulation of the full slam algorithm is run with one particle.");
		Console.WriteLine();
		Console.WriteLine("Options:");
		options.WriteOptionDescriptions(Console.Out);

	}

	/// <summary>
	/// Primary entry point.
	/// </summary>
	/// <param name="args">Command line arguments.</param>
	//[STAThread]
	public static void Main(string[] args)
	{
		string recfile       = "data.zip";
		string scenefile     = "";
		string commandfile   = "";
		string configfile    = "";
		int    particlecount = 1;
		bool   onlymapping   = false;
		bool   simulate      = false;
		bool   realtime      = false;
		bool   viewer        = false;
		bool   headless      = false;
		bool   showhelp      = false;

		NavigationAlgorithm algorithm = NavigationAlgorithm.PHD;

		OptionSet options = new OptionSet() {
			{ "f|scene=",      "Scene description file. Simulated, recorded or device id.",    f       => scenefile     = f },
			{ "r|recfile=",    "Recording file. Saves State and events for reviewing.",        r       => recfile       = r },
			{ "c|command=",    "Auto-command file (simulates user input).",                    c       => commandfile   = c },
			{ "g|config=",     "Configuration file. Contains global constants",                g       => configfile    = g },
			{ "a|algorithm=",  "SLAM solver algorithm (phd or isam2)",                         a       => algorithm     = (a != "isam2") ? NavigationAlgorithm.PHD : NavigationAlgorithm.ISAM2 },
			{ "p|particles=",  "Number of particles used for the RB-PHD",                      (int p) => particlecount = p },
			{ "y|onlymap",     "Only do mapping, assuming known localization.",                y       => onlymapping   = y != null },
			{ "s|simulate",    "Generate an artificial simulation instead of using a sensor.", s       => simulate      = s != null },
			{ "R|realtime",    "Process the system in realtime, instead of a fixed step.",     R       => realtime      = R != null },
			{ "v|view",        "View a precorded session.",                                    v       => viewer        = v != null },
			{ "x|headless",    "Run headless, i.e. with no GUI",                               x       => headless      = x != null },
			{ "h|help",        "Show this message and exit",                                   h       => showhelp      = h != null }
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

		if (!KinectVehicle.Initialize()) {
			KinectVehicle.Shutdown();
			Environment.Exit(2);
		}

		if (!string.IsNullOrEmpty(configfile)) {
			Config.FromFile(configfile);
		}

		if (viewer) {
			string tmpdir;

			using (Viewer sim = Viewer.FromFiles(recfile, scenefile, out tmpdir)) {
				sim.Run();
				Directory.Delete(tmpdir, true);
			}
		}
		else {
			using (Simulation sim = Simulation.FromFiles(scenefile, commandfile, particlecount, onlymapping, simulate, realtime, algorithm)) {
				if (headless) {
					sim.RunHeadless();
				}
				else {
					sim.Run();
				}

				string tmp    = Util.TemporaryDir();
				string output = Path.Combine(tmp, "out");
				Directory.CreateDirectory(output);

				Console.WriteLine("writing output");

				// with artificial data, the scene file has useful information
				// with real sensors it isn't that useful and it's way too heavy
				if (simulate) {
					Console.WriteLine("  -- copying scene file");
					File.Copy(scenefile, Path.Combine(output, "scene.world"));
				}

				Console.WriteLine("  -- writing trajectory history");
				File.WriteAllText(Path.Combine(output, "trajectory.out"),   sim.SerializedTrajectory);
				Console.WriteLine("  -- writing estimate history");
				File.WriteAllText(Path.Combine(output, "estimate.out"),     sim.SerializedEstimate);
				Console.WriteLine("  -- writing map model history");
				File.WriteAllText(Path.Combine(output, "maps.out"),         sim.SerializedMaps);
				Console.WriteLine("  -- writing measurements history");
				File.WriteAllText(Path.Combine(output, "measurements.out"), sim.SerializedMeasurements);

				if (sim.Explorer.HasSidebar) {
					Console.WriteLine("  -- writing sidebar video");
					SaveAsAvi(sim.SidebarHistory, Path.Combine(output, "sidebar.avi"));
				}

				Console.WriteLine("  -- compressing");

				if (File.Exists(recfile)) {
					File.Delete(recfile);
				}

				ZipFile.CreateFromDirectory(output, recfile);

				Console.WriteLine("  -- cleaning up");
				Directory.Delete(tmp, true);
			}
		}

		KinectVehicle.Shutdown();
	}
}
}
