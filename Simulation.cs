﻿// Simulation.cs
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

using Accord.Math;
using System.Text;

using TimedState        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMeasurements = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<double[]>>>;

namespace monorfs
{
/// <summary>
/// Navigation algorithm.
/// </summary>
public enum NavigationAlgorithm { PHD, ISAM2 }

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
	/// Measure cycle period. Every this amount of
	/// time the SLAM solver is invoked.
	/// </summary>
	public readonly TimeSpan MeasureElapsed = new TimeSpan(10000000/30);
	
	/// <summary>
	/// Last time the navigator update method was called.
	/// </summary>
	private GameTime lastnavigationupdate = new GameTime();

	/// <summary>
	/// Defines if the particles look at the odometry input as relevant information
	/// for the pose update. This may be set to false if there is no odometry
	/// or for performance comparisons of the system with and without it.
	/// </summary>
	public const bool UseOdometry = true;

	/// <summary>
	/// Every sidebar frame seen so far.
	/// </summary>
	public List<Texture2D> SidebarHistory;

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
	/// Get a string representation of the trajectory of the most likely particle
	/// (which may jump when the best particle changes).
	/// </summary>
	public string SerializedEstimate
	{
		get
		{
			return string.Join("\n|\n", Navigator.WayTrajectories.ConvertAll(t => t.Item1.ToString("F6") + "\n" + 
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
			return string.Join("\n", WayMeasurements.ConvertAll(m => m.Item1.ToString("F6") + ":" + string.Join(";",
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
			return string.Join("\n|\n", Navigator.WayMaps.ConvertAll(m => m.Item1.ToString("F6") + "\n" + string.Join("\n",
			                                                              m.Item2.ConvertAll(g => g.LinearSerialization))));
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
		                                                    return time.ToString("F6") + " " +
		                                                           p[0].ToString("F6") + " " +
		                                                           p[1].ToString("F6") + " " +
		                                                           p[2].ToString("F6") + " " +
		                                                           p[3].ToString("F6") + " " +
		                                                           p[4].ToString("F6") + " " +
		                                                           p[5].ToString("F6") + " " +
		                                                           p[6].ToString("F6");}));
	}

	/// <summary>
	/// Construct a simulation from its components.
	/// </summary>
	/// <param name="explorer">Main vehicle in the simulation. This is the only entity the user directly controls.</param>
	/// <param name="commands">List of stored commands for the vehicle as odometry readings.</param>
	/// <param name="particlecount">Number of particles for the RB-PHD algorithm.
	/// If only mapping is done, it is irrelevant.</param>
	/// <param name="onlymapping">If true, no localization is executed and the robot's position
	/// is assumed perfectly known.</param>
	/// <param name="simulate">True means a full simulation is performed;
	/// false uses real sensor data (realtime or from a file).</param>
	/// <param name="realtime">If true, the system works at the highest speed it can;
	/// otherwise, the framerate is fixed and even if processing takes longer than the timestep, the simulation works
	/// as it had taken exactly the nominal rate.</param>
	/// <param name="mapclip">Initial observable area in the form [left, right, bottom, top]</param>
	/// <param name="algorithm">SLAM navigation algorithm.</param>
	public Simulation(Vehicle explorer, List<double[]> commands, int particlecount,
	                  bool onlymapping, bool realtime, float[] mapclip,
	                  NavigationAlgorithm algorithm)
		: base(explorer,
		       (algorithm == NavigationAlgorithm.PHD) ?
		           (Navigator) new PHDNavigator(explorer, particlecount, onlymapping) :
		           (Navigator) new ISAM2Navigator(explorer, onlymapping),
		        realtime, mapclip)
	{
		Commands        = commands;
		commandindex    = 0;
		commanddepleted = false;

		SidebarHistory  = new List<Texture2D>();
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
	/// <param name="onlymapping">If true, no localization is executed and the robot's position
	/// is assumed perfectly known.</param>
	/// <param name="simulate">True means a full simulation is performed;
	/// false uses real sensor data (real device or a recording from a file).</param>
	/// <param name="realtime">If true, the system works at the highest speed it can;
	/// otherwise, the framerate is fixed and even if processing takes longer than the timestep, the simulation works
	/// as it had taken exactly the nominal rate.</param>
	/// <param name="algorithm">SLAM navigation algorithm.</param>
	/// <returns>Prepared simulation object.</returns>
	public static Simulation FromFiles(string scenefile, string commandfile = "", int particlecount = 5,
	                                   bool onlymapping = false, bool simulate = true, bool realtime = false,
	                                   NavigationAlgorithm algorithm = NavigationAlgorithm.PHD)
	{
		Vehicle        explorer;
		List<double[]> commands;
		float[]        mapclip;

		if (simulate) {
			explorer = VehicleFromSimFile(File.ReadAllText(scenefile), out mapclip);
		}
		else {
			explorer = VehicleFromSensor(scenefile, out mapclip);
		}

		try {
			if (!string.IsNullOrEmpty(commandfile)) {
				commands =  commandsFromDescriptor(File.ReadAllLines(commandfile));
			}
			else {
				commands = new List<double[]>();
			}
		}
		catch (FileNotFoundException) {
			Console.WriteLine("command file not found: " + commandfile);
			commands = new List<double[]>();
		}

		return new Simulation(explorer, commands, particlecount, onlymapping, realtime, mapclip, algorithm);
	}

	/// <summary>
	/// Create a command list from a descriptor string array.
	/// </summary>
	/// <param name="commandstr">Command descriptor array.</param>
	/// <returns>Command list.</returns>
	private static List<double[]> commandsFromDescriptor(string[] commandstr)
	{
		List<double[]> commands = new List<double[]>(commandstr.Length);

		for (int i = 0; i < commandstr.Length; i++) {
			commands.Add(ParseDoubleList(commandstr[i]));
			// the item structure is {dlocx, dlocy, dlocz, dyaw, dpitch, droll}
		}

		return commands;
	}
	

	/// <summary>
	/// Allow the simulation to perform any initialization it needs to do before it starts running.
	/// </summary>
	protected override void Initialize()
	{
		base.Initialize();
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
		if (commanddepleted) {
			Exit();
		}

		double dlocx     = 0;
		double dlocy     = 0;
		double dlocz     = 0;
		double dyaw      = 0;
		double dpitch    = 0;
		double droll     = 0;

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

				commandindex++;
			}
			else if (Commands.Count != 0) {
				Paused          = true;
				commanddepleted = true;
				// the depletion will trigger an exit in the next update
				// do not do it now, as the last frame still needs to be processed and rendered
			}

			Explorer.Update(time, dlocx, dlocy, dlocz, dyaw, dpitch, droll);

			if (UseOdometry) {
				Navigator.Update(time, dlocx, dlocy, dlocz, dyaw, dpitch, droll);
			}
			else {
				Navigator.Update(time, 0, 0, 0, 0, 0, 0);
			}

			if (time.TotalGameTime - lastnavigationupdate.TotalGameTime >= MeasureElapsed) {
				List<double[]> measurements = Explorer.Measure();
				WayMeasurements.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, measurements));
			
				Navigator.SlamUpdate(time, measurements);

				lastnavigationupdate = new GameTime(time.TotalGameTime, time.ElapsedGameTime);
			}

			if (forcehistoryreset) {
				Navigator.ResetHistory();
				Explorer .ResetHistory();
			}

			if (forcereset) {
				Navigator.ResetMapModel();
			}

			if (forceslam) {
				Navigator.StartSlam();
			}
			else if (forcemapping) {
				Navigator.StartMapping();
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
			Texture2D frame = new Texture2D(Graphics, SideBuffer.Width, SideBuffer.Height);
			Color[]   data  = new Color[SideBuffer.Width * SideBuffer.Height];

			SideBuffer.GetData(data);
			frame     .SetData(data);

			SidebarHistory.Add(frame);
		}
	}
}
}
