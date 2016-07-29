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
using System.Collections.Generic;
using System.IO;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
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
/// Vehicle input type.
/// </summary>
public enum VehicleType { Simulation, Kinect, Record }

/// <summary>
/// Interactive manipulator for a simulation of vehicle navigation through a landmark map.
/// </summary>
public class Simulation : Manipulator
{
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
				             r[0].ToString("g6") + " " +
				             r[1].ToString("g6") + " " +
				             r[2].ToString("g6") + " " +
				             r[3].ToString("g6") + " " +
				             r[4].ToString("g6") + " " +
				             r[5].ToString("g6");}
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
			return string.Join("\n|\n", Navigator.WayTrajectories.ConvertAll(t => t.Item1.ToString("g6") + "\n" + 
			                                                                      Simulation.SerializeWayPoints(t.Item2)));
		
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
		                                                           p[0].ToString("g6") + " " +
		                                                           p[1].ToString("g6") + " " +
		                                                           p[2].ToString("g6") + " " +
		                                                           p[3].ToString("g6") + " " +
		                                                           p[4].ToString("g6") + " " +
		                                                           p[5].ToString("g6") + " " +
		                                                           p[6].ToString("g6");}));
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
	public Simulation(string title, Vehicle explorer, Navigator navigator, List<double[]> commands,
	                  bool realtime, bool noterminate)
		: base(title, explorer, navigator, realtime)
	{
		Commands        = commands;
		commandindex    = 0;
		commanddepleted = false;

		this.noterminate = noterminate;

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
	public static Simulation FromFiles(string scenefile, string commandfile = "", int particlecount = 5,
	                                   VehicleType input = VehicleType.Simulation,
	                                   NavigationAlgorithm algorithm = NavigationAlgorithm.PHD,
	                                   bool onlymapping = false, bool realtime = false,
	                                   bool sidebar = true, bool noterminate = false)
	{
		Vehicle        explorer;
		Navigator      navigator;
		List<double[]> commands;

		// batch methods extra information
		TimedState        estimate     = null;
		TimedArray        odometry     = null;
		TimedMeasurements measurements = null;

		try {
			switch (input) {
			case VehicleType.Kinect:
				explorer = FP.VehicleFromSensor(scenefile, sidebar);
				break;
			case VehicleType.Record:
				if (algorithm == NavigationAlgorithm.LoopyPHD) {
					Console.WriteLine("reading Loopy PHD initialization data from file");
					explorer = FP.VehicleFromRecord(scenefile, true, out estimate, out odometry, out measurements);
				}
				else {
					explorer = FP.VehicleFromRecord(scenefile, false, out estimate, out odometry, out measurements);
				}
				break;
			case VehicleType.Simulation:
			default:
				explorer = FP.VehicleFromSimFile(File.ReadAllText(scenefile));
				break;
			}
		}
		catch (IOException) {
			Console.WriteLine("Couldn't open vehicle file/device: " + scenefile);
			explorer = new SimulatedVehicle();
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
			navigator = new OdometryNavigator(explorer);
			break;
		case NavigationAlgorithm.ISAM2:
			navigator = new ISAM2Navigator(explorer, onlymapping);
			break;
		case NavigationAlgorithm.LoopyPHD:
			Console.WriteLine("initializing Loopy PHD");
			if (estimate != null) {
				navigator = new LoopyPHDNavigator(explorer, estimate, odometry, measurements);
			}
			else {
				navigator = new LoopyPHDNavigator(explorer, commands,
				                                  new PHDNavigator(explorer, particlecount, onlymapping));
			}
			Console.WriteLine("Loopy PHD initialized");

			break;
		case NavigationAlgorithm.PHD:
		default:
			navigator = new PHDNavigator(explorer, particlecount, onlymapping);
			break;
		}

		string title = "monorfs - simulating " + scenefile + " [" + algorithm + "]";

		return new Simulation(title, explorer, navigator, commands, realtime, noterminate);
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
			dlocz += 0.02 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.K)) {
			dlocz -= 0.02 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.J)) {
			dyaw += 0.1 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.L)) {
			dyaw -= 0.1 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.W)) {
			dpitch -= 0.1 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.S)) {
			dpitch += 0.1 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.A)) {
			droll -= 0.1 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.D)) {
			droll += 0.1 * multiplier;
		}
		
		if (keyboard.IsKeyDown(Keys.M) && !prevkeyboard.IsKeyDown(Keys.M)) {
			forceslam    = Navigator.OnlyMapping;
			forcemapping = !Navigator.OnlyMapping;
		}
		
		if (!Paused) {
			if (commandindex < Commands.Count) {
				double[] autocmd = Commands[commandindex];
		
				dlocx  += autocmd[0];
				dlocy  += autocmd[1];
				dlocz  += autocmd[2];
				dyaw   += autocmd[3];
				dpitch += autocmd[4];
				droll  += autocmd[5];

				// the 7th parameter, if any, can force slam or mapping
				if (autocmd.Length > 6) {
					if (autocmd[6] > 0) {
						forceslam    = true;
						forcemapping = false;
					}
					else if (autocmd[6] < 0) {
						forceslam    = false;
						forcemapping = true;
					}
					// else, autocmd[6] == 0, do nothing
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
			Explorer.Update(time, new double[6] {dlocx, dlocy, dlocz, dpitch, dyaw, droll});

			if (UseOdometry) {
				double[] odm = Explorer.ReadOdometry(origtime);
				Navigator.Update(time, odm);
			}
			else {
				Navigator.Update(time, new double[6] {0, 0, 0, 0, 0, 0});
			}

			if (time.TotalGameTime - lastnavigationupdate.TotalGameTime >= MeasureElapsed) {
				List<double[]> measurements = Explorer.Measure(time);
				WayMeasurements.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, measurements));
				
				try {
					Navigator.SlamUpdate(time, measurements);
				}
				catch (InvalidOperationException e) {
					if (e.Data["module"].Equals("gtsam")) {
						Tags.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, "!gtsam failure"));
					}
					else if (e.Data["module"].Equals("association")) {
						Console.WriteLine("Tried to use perfect data association when none exists.");
					}
					commanddepleted = true;  // force exit in next call
				}

				lastnavigationupdate = new GameTime(time.TotalGameTime, time.ElapsedGameTime);
			}

			if (forcehistoryreset) {
				Navigator.ResetHistory();
				Explorer .ResetHistory();
			}

			if (forcereset) {
				Navigator.ResetMapModel();
			}
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
