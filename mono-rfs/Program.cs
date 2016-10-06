// Program.cs
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
using System.IO;
using System.IO.Compression;
using System.Diagnostics;
using System.Threading;

using Mono.Unix;
using NDesk.Options;

namespace monorfs
{
/// <summary>
/// Main program class.
/// </summary>
public class Program
{
	/// <summary>
	/// Thread that listens to SIGINT for graceful termination.
	/// </summary>
	public static Thread signalthread;

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
	/// Set a handler for SIGINT, so they system can exit gracefully.
	/// </summary>
	public static void setSignalHandler<MeasurerT, PoseT, MeasurementT>(
	                        Manipulator<MeasurerT, PoseT, MeasurementT> manipulator)
		where PoseT        : IPose<PoseT>, new()
		where MeasurementT : IMeasurement<MeasurementT>, new()
		where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
	{
		UnixSignal sigint = new UnixSignal(Mono.Unix.Native.Signum.SIGINT);

		signalthread = new Thread(() => {
			while (true) {
				sigint.WaitOne();

				if (manipulator != null && !manipulator.Abort) {
					manipulator.Abort = true;
				}
				else {
					Environment.Exit(1);
				}
			}
		});

		signalthread.Start();
	}

	/// <summary>
	/// Primary entry point.
	/// </summary>
	/// <param name="args">Command line arguments.</param>
	//[STAThread]
	public static void Main(string[] args)
	{
		string recfile        = "data.zip";
		string scenefile      = "";
		string commandfile    = "";
		string configfile     = "";
		int    particlecount  = 1;
		bool   onlymapping    = false;
		bool   realtime       = false;
		bool   viewer         = false;
		bool   filterhistory  = false;
		bool   screenshotmode = false;
		bool   headless       = false;
		bool   noterminate    = false;
		bool   showhelp       = false;

		VehicleType         input     = VehicleType.Simulation;
		NavigationAlgorithm algorithm = NavigationAlgorithm.PHD;

		OptionSet options = new OptionSet {
			{ "f|scene=",      "Scene description file. Simulated, recorded or device id.",      f       => scenefile      = f },
			{ "r|recfile=",    "Recording file. Saves State and events for reviewing.",          r       => recfile        = r },
			{ "c|command=",    "Auto-command file (simulates user input).",                      c       => commandfile    = c },
			{ "g|config=",     "Configuration file. Contains global constants.",                 g       => configfile     = g },
			{ "a|algorithm=",  "SLAM solver algorithm ('odometry', 'phd', 'loopy' or 'isam2').", a       => algorithm      = (a == "isam2") ? NavigationAlgorithm.ISAM2 : (a == "odometry") ? NavigationAlgorithm.Odometry : (a == "loopy") ? NavigationAlgorithm.LoopyPHD : NavigationAlgorithm.PHD },
			{ "p|particles=",  "Number of particles used for the RB-PHD.",                       (int p) => particlecount  = p },
			{ "y|onlymap",     "Only do mapping, assuming known localization.",                  y       => onlymapping    = y != null },
			{ "i|input=",      "Vehicle input stream: 'kinect', 'simulation' or 'record'.",      i       => input          = (i == "kinect") ? VehicleType.Kinect : (i == "record") ? VehicleType.Record : VehicleType.Simulation },
			{ "R|realtime",    "Process the system in realtime, instead of a fixed step.",       R       => realtime       = R != null },
			{ "v|view",        "View a precorded session.",                                      v       => viewer         = v != null },
			{ "H|history=",    "Trajectory history mode: either 'filter' or 'smooth'.",          h       => filterhistory  = (h == "filter") },
			{ "s|screenshot",  "Screenshot mode: just take the screenshots in the tags.",        s       => screenshotmode = s != null },
			{ "x|headless",    "Run headless, i.e. with no GUI.",                                x       => headless       = x != null },
			{ "t|noterminate", "Skip simulation termination due command depletion o similars.",  t       => noterminate    = t != null },
			{ "h|help",        "Show this message and exit.",                                    h       => showhelp       = h != null }
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

		TimeSpan time = DateTime.Now.ToUniversalTime() - new DateTime(2010, 1, 1, 0, 0, 0, DateTimeKind.Utc);
		Util.SeedGenerators((int) time.TotalSeconds);

		if (!KinectVehicle.Initialize()) {
			KinectVehicle.Shutdown();
			Environment.Exit(2);
		}

		if (!string.IsNullOrEmpty(configfile)) {
			try {
				Config.FromFile(configfile);
			}
			catch (FileNotFoundException e) {
				Console.WriteLine("Error: Configuration file '" + e.FileName + "' not found.");
				Environment.Exit(3);
			}
		}

		try {
		switch (Config.Model) {
		case DynamicsModel.Linear2D:
			Run<Linear2DMeasurer, LinearPose2D, LinearMeasurement2D>(
			    recfile, scenefile, commandfile,
			    particlecount, onlymapping, realtime, viewer,
			    filterhistory, screenshotmode, headless, noterminate,
			    input, algorithm);
			break;
		case DynamicsModel.PRM3D:
		default:
			if (input == VehicleType.Kinect) {
			Run<KinectMeasurer, Pose3D, PixelRangeMeasurement>(
			    recfile, scenefile, commandfile,
			    particlecount, onlymapping, realtime, viewer,
			    filterhistory, screenshotmode, headless, noterminate,
				input, algorithm);
			}
			else {
			Run<PRM3DMeasurer, Pose3D, PixelRangeMeasurement>(
			    recfile, scenefile, commandfile,
			    particlecount, onlymapping, realtime, viewer,
			    filterhistory, screenshotmode, headless, noterminate,
				input, algorithm);
			}
			break;
		}
		}
		catch (FileNotFoundException e) {
			Console.WriteLine("Error: File '" + e.FileName + "' not found.");
			Environment.Exit(4);
		}

		KinectVehicle.Shutdown();
		signalthread.Abort();
	}

	/// <summary>
	/// Run the simulation or viewer.
	/// </summary>
	public static void Run<MeasurerT, PoseT, MeasurementT>(
	                       string recfile, string scenefile, string commandfile,
	                       int particlecount, bool onlymapping, bool realtime, bool viewer,
	                       bool filterhistory, bool screenshotmode, bool headless, bool noterminate,
	                       VehicleType input, NavigationAlgorithm algorithm)
		where PoseT        : IPose<PoseT>, new()
		where MeasurementT : IMeasurement<MeasurementT>, new()
		where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
	{

		if (viewer) {
			string tmpdir;

			using (var sim = Viewer<MeasurerT, PoseT, MeasurementT>.
			                     FromFiles(recfile, filterhistory, screenshotmode, out tmpdir)) {
				sim.ScreenshotPrefix = recfile + "-screenshot-";
				setSignalHandler(sim);

				if (sim.Estimate.Count > 0) {
					sim.Run();

					if (sim.TagChanged) {
						string datadir = Path.Combine(tmpdir, "data");
						string bkpfile = recfile + ".bk";

						Console.WriteLine("  -- tags were modified, rewriting");
						File.WriteAllText(Path.Combine(datadir, "tags.out"), sim.SerializedTags);

						Console.WriteLine("  -- old file was moved to " + bkpfile);

						if (File.Exists(bkpfile)) {
							File.Delete(bkpfile);
						}

						File.Move(recfile, bkpfile);
						
						Console.WriteLine("  -- compressing");
						ZipFile.CreateFromDirectory(datadir, recfile);
					}

					Directory.Delete(tmpdir, true);
				}
				else {
					Console.WriteLine("Nothing to show! Make sure there are appropriate frames " +
					                  "in the specified file. This can happen if screenshot mode is " +
					                  "used and there are no screenshot tags.");
				}
			}
		}
		else {
			using (var sim = Simulation<MeasurerT, PoseT, MeasurementT>.
			                     FromFiles(scenefile, commandfile, particlecount, input,
			                               algorithm, onlymapping, realtime, !headless, noterminate)) {
				sim.ScreenshotPrefix = recfile + "-screenshot-";
				sim.CheckpointFile   = recfile;
				setSignalHandler(sim);

				if (headless) {
					Stopwatch timer = new Stopwatch();

					Console.WriteLine("running headless");

					timer.Start();
					sim.RunHeadless();
					timer.Stop();

					Console.WriteLine("finished running (" + timer.Elapsed.TotalSeconds.ToString("F4") + " s)");
				}
				else {
					sim.Run();
				}

				sim.SaveToFile(recfile);
			}
		}
	}
}
}
