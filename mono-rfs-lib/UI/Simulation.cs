// Simulation.cs
// Vehicle navigation simulation
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
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Diagnostics;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

using FP = monorfs.FileParser;

using TimedState        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedArray        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMeasurements = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<double[]>>>;

namespace monorfs
{
/// <summary>
/// Navigation algorithm.
/// </summary>
public enum NavigationAlgorithm { Odometry, PHD, LoopyPHD, ISAM2 }

/// <summary>
/// Navigation algorithm.
/// </summary>
public enum DynamicsModel { PRM3D, Linear2D, Linear1D }

/// <summary>
/// Vehicle input type.
/// </summary>
public enum VehicleType { Simulation, Kinect, Record }

/// <summary>
/// Interactive manipulator for a simulation of vehicle navigation through a landmark map.
/// </summary>
public class Simulation<MeasurerT, PoseT, MeasurementT> : Manipulator<MeasurerT, PoseT, MeasurementT>
	where PoseT        : IPose<PoseT>, new()
	where MeasurementT : IMeasurement<MeasurementT>, new()
	where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
{
	/// <summary>
	/// Initial vehicle pass as argument with initial pose (for saving it at the end).
	/// </summary>
	public SimulatedVehicle<MeasurerT, PoseT, MeasurementT> InitVehicle;

	/// <summary>
	/// Automatic simulation command input.
	/// </summary>
	public List<double[]> Commands { get; private set; }

	/// <summary>
	/// Frame number for the command list.
	/// </summary>
	private int commandindex;

	/// <summary>
	/// True if all command frames have been played;
	/// this signals the system to shutdown.
	/// </summary>
	private bool commanddepleted;

	/// <summary>
	/// If true, the simulation will remain active after cues
	/// like command depletion or vehicle requirements ask to do so.
	/// </summary>
	private bool noterminate;

	/// <summary>
	/// Measure cycle period. Every this amount of
	/// time the SLAM solver is invoked.
	/// </summary>
	public static TimeSpan MeasureElapsed { get { return Config.MeasureElapsed; } }
	
	/// <summary>
	/// Last time the navigator update method was called.
	/// </summary>
	private GameTime lastnavigationupdate = new GameTime(new TimeSpan(-1, 0, 0), TimeSpan.Zero);

	/// <summary>
	/// Defines if the particles look at the odometry input as relevant information
	/// for the pose update. This may be set to false if there is no odometry
	/// or for performance comparisons of the system with and without it.
	/// </summary>
	public static bool UseOdometry { get { return Config.UseOdometry; } }

	/// <summary>
	/// If a valid string, file name to be used to checkpoint the progress.
	/// </summary>
	public string CheckpointFile { get; set; }

	/// <summary>
	/// Wait time between checkpoints, in seconds (approximate, it depends on the SLAM cycle).
	/// </summary>
	/// <value>The checkpoint.</value>
	public static int CheckpointCycleTime { get { return Config.CheckpointCycleTime; } }

	/// <summary>
	/// Timer to keep track of checkpoints.
	/// </summary>
	private Stopwatch timer;

	/// <summary>
	/// Every sidebar frame seen so far.
	/// </summary>
	public List<Color[]> SidebarHistory;

	/// <summary>
	/// Measurement history.
	/// </summary>
	public TimedMeasurements WayMeasurements { get; private set; }
	
	/// <summary>
	/// Get a string representation of the real trajectory of the vehicle.
	/// </summary>
	public string SerializedTrajectory
	{
		get
		{
			return SerializeWayPoints(Explorer.WayPoints);
		}
	}

	/// <summary>
	/// Get a string representation of the odometry history read by of the vehicle.
	/// </summary>
	public string SerializedOdometry
	{
		get
		{
			return string.Join("\n", Explorer.WayOdometry.ConvertAll(
				s => {double time = s.Item1;
			          double[] r  = s.Item2;
				      return time.ToString("g6") + " " +
			                 String.Join(" ", Array.ConvertAll(r, x => x.ToString("g6"))); }
			));
		}
	}
	
	/// <summary>
	/// Get a string representation of the trajectory of the most likely particle
	/// (which may jump when the best particle changes).
	/// </summary>
	public string SerializedEstimate
	{
		get
		{
			return string.Join("\n|\n", Navigator.WayTrajectories.ConvertAll(
			                                t => t.Item1.ToString("g6") + "\n" +
			                                SerializeWayPoints(t.Item2)));
		
		}
	}

	/// <summary>
	/// Get a string representation of the measurements in every time step
	/// </summary>
	public string SerializedMeasurements
	{
		get
		{
			return string.Join("\n", WayMeasurements.ConvertAll(m => m.Item1.ToString("g6") + ":" + string.Join(";",
			                                                         m.Item2.ConvertAll(x => string.Join(" ", x)))));
		}
	}

	/// <summary>
	/// Get a string representation of the map models of the best particle;
	/// may jump along with the best particle.
	/// </summary>
	public string SerializedMaps
	{
		get
		{
			return string.Join("\n|\n", Navigator.WayMaps.ConvertAll(m => m.Item1.ToString("g6") + "\n" + string.Join("\n",
			                                                              m.Item2.ConvertAll(g => g.ToString()))));
		}
	}

	/// <summary>
	/// Get a string representation of the groundtruth visible map history.
	/// </summary>
	public string SerializedVisibleMaps
	{
		get
		{
			return string.Join("\n|\n", Explorer.WayVisibleMaps.ConvertAll(m => m.Item1.ToString("g6") + "\n" + string.Join("\n",
			                                                               m.Item2.ConvertAll(g => g.ToString()))));
		}
	}

	/// <summary>
	/// Serializes a trajectory waypoints.
	/// </summary>
	/// <returns>Serialization string.</returns>
	/// <param name="waypoints">Trajectory.</param>
	public static string SerializeWayPoints(TimedState waypoints)
	{
		return string.Join("\n", waypoints.ConvertAll(s => {double time = s.Item1;
		                                                    double[] p = s.Item2;
		                                                    return time.ToString("g6") + " " +
		                                                            String.Join(" ", Array.ConvertAll(p, x => x.ToString("g6"))); }));
	}

	/// <summary>
	/// Construct a simulation from its components.
	/// </summary>
	/// <param name="title">Window title.</param>
	/// <param name="explorer">Main vehicle in the simulation. This is the only entity the user directly controls.</param>
	/// <param name="navigator">SLAM solver.</param>
	/// <param name="commands">List of stored commands for the vehicle as odometry readings.</param>
	/// <param name="realtime">If true, the system works at the highest speed it can;
	/// otherwise, the framerate is fixed and even if processing takes longer than the timestep, the simulation works
	/// as it had taken exactly the nominal rate.</param>
	/// <param name="noterminate"> If true, the simulation will remain active after cues
	/// like command depletion or vehicle requirements ask to do so.</param>
	public Simulation(string title,
	                  Vehicle<MeasurerT, PoseT, MeasurementT> explorer,
	                  Navigator<MeasurerT, PoseT, MeasurementT> navigator,
	                  List<double[]> commands,
	                  bool realtime, bool noterminate)
		: base(title, explorer, navigator, realtime)
	{
		Commands        = commands;
		commandindex    = 0;
		commanddepleted = false;

		this.noterminate = noterminate;

		timer = new Stopwatch();
		timer.Start();

		InitVehicle     = new SimulatedVehicle<MeasurerT, PoseT, MeasurementT>(explorer);
		SidebarHistory  = new List<Color[]>();
		WayMeasurements = new TimedMeasurements();
		// note that WayMeasurements starts empty as measurements are between frames
		// so it will have a length |frames| - 1
	}

	/// <summary>
	/// Create a simulation object from a pair of formatted description files.
	/// </summary>
	/// <param name="scenefile">Scene descriptor filename.</param>
	/// <param name="commandfile">Vehicle input command instructions filename.
	/// Can be null or empty (no automatic instructions).</param>
	/// <param name="particlecount">Number of particles for the RB-PHD algorithm.
	/// If only mapping is done, it's not used, but the mode can be changed at runtime.</param>
	/// <param name="input">Vehicle input type, either a sensor or a simulation algorithm.</param>
	/// <param name="algorithm">SLAM navigation algorithm.</param>
	/// <param name="onlymapping">If true, no localization is executed and the robot's position
	/// is assumed perfectly known.</param>
	/// <param name="realtime">If true, the system works at the highest speed it can;
	/// otherwise, the framerate is fixed and even if processing takes longer than the timestep, the simulation works
	/// as it had taken exactly the nominal rate.</param>
	/// <param name="sidebar">Allow a sidebar with extra details.</param>
	/// <param name="noterminate"> If true, the simulation will remain active after cues
	/// like command depletion or vehicle requirements ask to do so.</param>
	/// <returns>Prepared simulation object.</returns>
	public static Simulation<MeasurerT, PoseT, MeasurementT>
	    FromFiles(string scenefile, string commandfile = "", int particlecount = 5,
	              VehicleType input = VehicleType.Simulation,
	              NavigationAlgorithm algorithm = NavigationAlgorithm.PHD,
	              bool onlymapping = false, bool realtime = false,
	              bool sidebar = true, bool noterminate = false)
	{
		Vehicle<MeasurerT, PoseT, MeasurementT>   explorer;
		Navigator<MeasurerT, PoseT, MeasurementT> navigator;
		List<double[]> commands;

		// batch methods extra information
		TimedState        estimate     = null;
		TimedArray        odometry     = null;
		TimedMeasurements measurements = null;

		try {
			switch (input) {
			case VehicleType.Kinect:
				if (typeof(MeasurementT) == typeof(PixelRangeMeasurement) &&
					typeof(PoseT)        == typeof(Pose3D) &&
					typeof(MeasurerT)    == typeof(KinectMeasurer)) {
					explorer = KinectVehicle.FromSensor(scenefile, sidebar) as
					               Vehicle<MeasurerT, PoseT, MeasurementT>;
				}
				else {
					throw new ArgumentException("KinectVehicle can only handle 3D Pose + PixelRange");
				}
				break;
			case VehicleType.Record:
				if (algorithm == NavigationAlgorithm.LoopyPHD) {
					Console.WriteLine("reading Loopy PHD initialization data from file");
					explorer = RecordVehicle<MeasurerT, PoseT, MeasurementT>.
					               FromFile(scenefile, true, out estimate, out odometry, out measurements);
				}
				else {
					explorer = RecordVehicle<MeasurerT, PoseT, MeasurementT>.
					               FromFile(scenefile, false, out estimate, out odometry, out measurements);
				}
				break;
			case VehicleType.Simulation:
			default:
				explorer = SimulatedVehicle<MeasurerT, PoseT, MeasurementT>.
				               FromFile(File.ReadAllText(scenefile));
				break;
			}
		}
		catch (IOException) {
			Console.WriteLine("Couldn't open vehicle file/device: " + scenefile);
			explorer = new SimulatedVehicle<MeasurerT, PoseT, MeasurementT>();
		}

		try {
			if (!string.IsNullOrEmpty(commandfile)) {
				commands =  FP.CommandsFromDescriptor(File.ReadAllLines(commandfile));
			}
			else {
				commands = new List<double[]>();
			}
		}
		catch (FileNotFoundException) {
			Console.WriteLine("command file not found: " + commandfile);
			commands = new List<double[]>();
		}

		switch (algorithm) {
		case NavigationAlgorithm.Odometry:
			navigator = new OdometryNavigator<MeasurerT, PoseT, MeasurementT>(explorer);
			break;
		case NavigationAlgorithm.ISAM2:
			navigator = new ISAM2Navigator<MeasurerT, PoseT, MeasurementT>(explorer, onlymapping);
			break;
		case NavigationAlgorithm.LoopyPHD:
			Console.WriteLine("initializing Loopy PHD");
			if (estimate != null) {
				navigator = new LoopyPHDNavigator<MeasurerT, PoseT, MeasurementT>(explorer, estimate,
				                                                                  odometry, measurements);
			}
			else {
				navigator = new LoopyPHDNavigator<MeasurerT, PoseT, MeasurementT>(explorer, commands,
				                new PHDNavigator<MeasurerT, PoseT, MeasurementT>(explorer, particlecount, onlymapping));
			}

			explorer = new FakeVehicle<MeasurerT, PoseT, MeasurementT>(explorer, true);

			Console.WriteLine("Loopy PHD initialized");

			break;
		case NavigationAlgorithm.PHD:
		default:
			navigator = new PHDNavigator<MeasurerT, PoseT, MeasurementT>(explorer, particlecount, onlymapping);
			break;
		}

		string title = "monorfs - simulating " + scenefile + " [" + algorithm + "]";

		return new Simulation<MeasurerT, PoseT, MeasurementT>(title, explorer, navigator, commands, realtime, noterminate);
	}

	/// <summary>
	/// Save all the state history to a file.
	/// </summary>
	/// <param name="filename">Output file name.</param>
	/// <param name="includesidebar">If false, skip the sidebar video.</param>
	public void SaveToFile(string filename, bool includesidebar = true, bool verbose = true)
	{

		string tmp    = Util.TemporaryDir();
		string output = Path.Combine(tmp, "out");
		Directory.CreateDirectory(output);

		if (verbose) {
			Console.WriteLine("writing output");
		}

		// with artificial data, the scene file has useful information
		// with real sensors only write pose and focal characteristics
		if (verbose) {
			Console.WriteLine("  -- writing scene file");
		}

		string scenedata = InitVehicle.ToString();
		File.WriteAllText(Path.Combine(output, "scene.world"), scenedata);

		if (verbose) {
			Console.WriteLine("  -- writing trajectory history");
		}

		File.WriteAllText(Path.Combine(output, "trajectory.out"),   SerializedTrajectory);

		if (verbose) {
			Console.WriteLine("  -- writing odometry history");
		}

		File.WriteAllText(Path.Combine(output, "odometry.out"),     SerializedOdometry);

		if (verbose) {
			Console.WriteLine("  -- writing estimate history");
		}

		File.WriteAllText(Path.Combine(output, "estimate.out"),     SerializedEstimate);

		if (verbose) {
			Console.WriteLine("  -- writing map model history");
		}

		File.WriteAllText(Path.Combine(output, "maps.out"),         SerializedMaps);

		if (verbose) {
			Console.WriteLine("  -- writing visible map history");
		}

		File.WriteAllText(Path.Combine(output, "vismaps.out"),      SerializedVisibleMaps);

		if (verbose) {
			Console.WriteLine("  -- writing measurements history");
		}

		File.WriteAllText(Path.Combine(output, "measurements.out"), SerializedMeasurements);

		if (verbose) {
			Console.WriteLine("  -- writing tags");
		}

		File.WriteAllText(Path.Combine(output, "tags.out"), SerializedTags);

		if (verbose) {
			Console.WriteLine("  -- writing config");
		}

		File.WriteAllText(Path.Combine(output, "config.cfg"), Config.ToString());

		if (Explorer.HasSidebar && includesidebar) {
			if (verbose) {
				Console.WriteLine("  -- writing sidebar video");
			}

			Util.SaveAsAvi(SidebarHistory, SideBuffer.Width, SideBuffer.Height,
			               Path.Combine(output, "sidebar.avi"));
		}

		if (verbose) {
			Console.WriteLine("  -- compressing");
		}

		if (File.Exists(filename)) {
			File.Delete(filename);
		}

		string dirpath = Path.GetDirectoryName(filename);
		if (dirpath != "") {
			Directory.CreateDirectory(dirpath);
		}

		ZipFile.CreateFromDirectory(output, filename);

		if (verbose) {
			Console.WriteLine("  -- cleaning up");
		}

		Directory.Delete(tmp, true);
	}

	/// <summary>
	/// Allow the simulation to run logic such as updating the world
	/// and gathering input.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="keyboard">Current keyboard state information.</param>
	/// <param name="prevkeyboard">Old keyboard state information. Used to get newly pressed keys.</param>
	/// <param name="multiplier">Movement scale multiplier.</param>
	protected override void Update(GameTime time, KeyboardState keyboard, KeyboardState prevkeyboard, double multiplier)
	{
		if (!string.IsNullOrEmpty(CheckpointFile)) {
			int elapsed = (int) timer.Elapsed.TotalSeconds;

			if (elapsed > CheckpointCycleTime) {
				Console.WriteLine("Writing checkpoint to " + CheckpointFile);
				SaveToFile(CheckpointFile, false, false);
				Console.WriteLine("Checkpoint complete");

				timer.Restart();
			}
		}

		if (!noterminate && (commanddepleted || Explorer.WantsToStop)) {
			Exit();
			return;
		}

		double dlocx  = 0;
		double dlocy  = 0;
		double dlocz  = 0;
		double dyaw   = 0;
		double dpitch = 0;
		double droll  = 0;

		bool forceslam         = false;
		bool forcemapping      = false;
		bool forcereset        = keyboard.IsKeyDown(Keys.R);
		bool forcehistoryreset = keyboard.IsKeyDown(Keys.T);
		
		if (keyboard.IsKeyDown(Keys.I)) {
			dlocz += multiplier;
		}

		if (keyboard.IsKeyDown(Keys.K)) {
			dlocz -=  multiplier;
		}

		if (keyboard.IsKeyDown(Keys.J)) {
			dyaw -= multiplier;
		}

		if (keyboard.IsKeyDown(Keys.L)) {
			dyaw += multiplier;
		}

		if (keyboard.IsKeyDown(Keys.W)) {
			dpitch += multiplier;
		}

		if (keyboard.IsKeyDown(Keys.S)) {
			dpitch -= multiplier;
		}

		if (keyboard.IsKeyDown(Keys.A)) {
			droll -= multiplier;
		}

		if (keyboard.IsKeyDown(Keys.D)) {
			droll += multiplier;
		}
		
		if (keyboard.IsKeyDown(Keys.M) && !prevkeyboard.IsKeyDown(Keys.M)) {
			forceslam    = Navigator.OnlyMapping;
			forcemapping = !Navigator.OnlyMapping;
		}

		double[] autocmd = new double[OdoSize];
		double[] keycmd  = new double[6] {dlocx, dlocy, dlocz, dpitch, dyaw, droll};

		double[] odometry = Explorer.Pose.AddKeyboardInput(autocmd, keycmd);
		
		if (Paused) {
			Explorer.Pose = Explorer.Pose.AddOdometry(odometry);
		}
		else {
			if (commandindex < Commands.Count) {
				autocmd  = Commands[commandindex];
				odometry = Explorer.Pose.AddKeyboardInput(autocmd, keycmd);

				// extra parameters
				if (autocmd.Length > OdoSize) {
					// the first can force slam or mapping
					if (autocmd[OdoSize] > 0) {
						forceslam    = true;
						forcemapping = false;
					}
					else if (autocmd[OdoSize] < 0) {
						forceslam    = false;
						forcemapping = true;
					}
					// else, autocmd[OdoSize] == 0, do nothing

					// the second takes a screenshot; if "true"
					// the next three components specify the camera angles and zoom
					// for the screenshot
					// this is unavailable in headless mode because of the lack of a graphics context
					// to draw the image
					if (Graphics != null && autocmd.Length > OdoSize + 1 && autocmd[OdoSize + 1] > 0) {
						double theta = autocmd[OdoSize + 2];
						double phi   = autocmd[OdoSize + 3];
						double zoom  = autocmd[OdoSize + 4];

						SetCamera(theta, phi, zoom);

						// the new camera will not take effect until the scene is redrawn
						Draw(time);
						Screenshot();
					}
				}

				commandindex++;
			}
			else if (Commands.Count != 0) {
				Paused          = true;
				commanddepleted = true;
				// the depletion will trigger an exit in the next update
				// do not do it now, as the last frame still needs to be processed and rendered
			}

			if (Explorer.WantsSLAM) {
				forceslam = true;
			}

			if (Explorer.WantsMapping) {
				forcemapping = true;
			}

			if (forceslam) {
				Tags.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, "SLAM mode on"));
				Navigator.StartSlam();
			}
			else if (forcemapping) {
				Tags.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, "Mapping mode on"));
				Navigator.StartMapping();
			}

			GameTime origtime = time;
			time = new GameTime(time.TotalGameTime.Add(FrameElapsed), time.ElapsedGameTime);
			Explorer.Update(time, odometry);

			if (UseOdometry) {
				double[] corruptodometry = Explorer.ReadOdometry(origtime);
				Navigator.Update(time, corruptodometry);
			}
			else {
				Navigator.Update(time, new double[OdoSize]);
			}

			if (time.TotalGameTime - lastnavigationupdate.TotalGameTime >= MeasureElapsed) {
				List<MeasurementT> measurements = Explorer.Measure(time);
				WayMeasurements.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, measurements.ConvertAll(m => m.ToLinear())));
				
				try {
					Navigator.SlamUpdate(time, measurements);
				}
				catch (InvalidOperationException e) {
					if (e.Data["module"] == null) {
						Console.WriteLine("Unknown invalid operation.");
						foreach (DictionaryEntry entry in e.Data) {
							Console.WriteLine(entry.Key + ": " + entry.Value);
						}
					}
					else if (e.Data["module"].Equals("gtsam")) {
						Tags.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, "!gtsam failure"));
						Console.WriteLine("gtsam failed.");
					}
					else if (e.Data["module"].Equals("association")) {
						Console.WriteLine("Tried to use perfect data association when none exists.");
					}
					commanddepleted = true;  // force exit in next call
				}

				lastnavigationupdate = new GameTime(time.TotalGameTime, time.ElapsedGameTime);
			}
		}

		if (forcehistoryreset) {
			Navigator.ResetHistory();
			Explorer .ResetHistory();
		}

		if (forcereset) {
			Navigator.ResetMapModel();
		}
	}
	
	/// <summary>
	/// This is called when the simulator should draw itself.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected override void Draw(GameTime time)
	{
		base.Draw(time);

		if (!Paused && Explorer.HasSidebar) {
			Color[] data = new Color[SideBuffer.Width * SideBuffer.Height];

			SideBuffer.GetData(data);
			SidebarHistory.Add(data);
		}
	}

	/// <summary>
	/// Run the simulation without generating any GUI;
	/// bypasses MonoGame.
	/// </summary>
	public void RunHeadless()
	{
		TimeSpan      dt         = MeasureElapsed;
		GameTime      time       = new GameTime(new TimeSpan(), dt);
		KeyboardState keyboard   = new KeyboardState();

		if (Commands.Count == 0) {  // no infinite simulations, make it one command = one frame
			Commands = new List<double[]>();
		}

		int i = 1;
		while (!commanddepleted && !Explorer.WantsToStop && !Abort) {
			Update(time, keyboard, keyboard, 1.0);
			time = new GameTime(time.TotalGameTime.Add(dt), dt);
			Console.Write(i + ", ");
			i++;
		}
	}
}
}
