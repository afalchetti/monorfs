// Simulation.cs
// Vehicle navigation simulation
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

using System;
using System.Collections.Generic;
using System.IO;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

using Accord.Math;
using System.Text;

namespace monorfs
{
/// <summary>
/// Interactive manipulator for a simulation of vehicle navigation through a landmark map.
/// </summary>
public class Simulation : Manipulator
{
	/// <summary>
	/// Automatic simulation command input.
	/// </summary>
	public CircularBuffer<double[]> Commands { get; private set; }

	/// <summary>
	/// Frame number for the command list.
	/// </summary>
	private int commandindex;

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
	/// Get a string representation of the trajectories of the vehicle and the
	/// most likely particle (which may jump when the best particle changes).
	/// </summary>
	public string SerializedTrajectories
	{
		get
		{
			return string.Join("\n", Explorer.Waypoints.ConvertAll(p => p[0].ToString("F6") + " " +
			                                                            p[1].ToString("F6") + " " +
			                                                            p[2].ToString("F6") + " " +
			                                                            p[3].ToString("F6"))) +
			       "\n|\n" +
			       string.Join("\n", Navigator.Waypoints.ConvertAll(p => p[0].ToString("F6") + " " +
			                                                             p[1].ToString("F6") + " " +
			                                                             p[2].ToString("F6") + " " +
			                                                             p[3].ToString("F6")));
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
	/// Create a simulation object from apair of formatted desccription files.
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
	/// <returns>Prepared simulation object.</returns>
	public static Simulation FromFiles(string scenefile, string commandfile = "", int particlecount = 5,
	                                   bool onlymapping = false, bool simulate = true, bool realtime = false)
	{
		Vehicle                  explorer;
		CircularBuffer<double[]> commands;
		float[]                  mapclip;

		if (simulate) {
			explorer = vehicleFromSimFile(File.ReadAllText(scenefile), out mapclip);
		}
		else {
			explorer = vehicleFromSensor(scenefile, out mapclip);
		}

		try {
			if (!string.IsNullOrEmpty(commandfile)) {
				commands =  commandsFromDescriptor(File.ReadAllLines(commandfile));
			}
			else {
				commands = new CircularBuffer<double[]>(1);
				commands.Add(new double[7] {0, 0, 0, 0, 0, 0, 0});
			}
		}
		catch (FileNotFoundException) {
			commands = new CircularBuffer<double[]>(1);
			commands.Add(new double[7] {0, 0, 0, 0, 0, 0, 0});
		}

		return new Simulation(explorer, commands, mapclip, particlecount, onlymapping, realtime);
	}

	/// <summary>
	/// Construct a simulation from its parts.
	/// </summary>
	/// <param name="explorer">Main vehicle in the simulation. This is the only entity the user directly controls.</param>
	/// <param name="commands">List of stored commands for the vehicle as odometry readings.</param>
	/// <param name="mapclip">Initial observable area in the form [left, right, bottom, top]</param>
	/// <param name="particlecount">Number of particles for the RB-PHD algorithm.
	/// If only mapping is done, it is irrelevant.</param>
	/// <param name="onlymapping">If true, no localization is executed and the robot's position
	/// is assumed perfectly known.</param>
	/// <param name="simulate">True means a full simulation is performed;
	/// false uses real sensor data (realtime or from a file).</param>
	/// <param name="realtime">If true, the system works at the highest speed it can;
	/// otherwise, the framerate is fixed and even if processing takes longer than the timestep, the simulation works
	/// as it had taken exactly the nominal rate.</param>
	public Simulation(Vehicle explorer, CircularBuffer<double[]> commands, float[] mapclip, int particlecount = 5,
	                  bool onlymapping = false, bool realtime = false)
		: base(explorer, new Navigator(explorer, particlecount, onlymapping), particlecount, realtime, mapclip)
	{
		Commands = commands;
	}

	/// <summary>
	/// Create a vehicle from a simulation file.
	/// </summary>
	/// <param name="scene">Scene descriptor text.</param>
	/// <param name="mapclip">Secondary output; initial observable area.</param>
	/// <returns>Simulated vehicle parsed from file.</returns>
	private static Vehicle vehicleFromSimFile(string scene, out float[] mapclip)
	{
		Dictionary<string, List<string>> dict = Util.ParseDictionary(scene);

		mapclip = Accord.Math.Matrix.ToSingle(ParseDoubleList(dict["world"][0]));

		if (mapclip.Length != 4) {
			throw new FormatException("The map clipping area must be specified by four elements: left, right, bottom, top");
		}

		double[] vehiclepose = ParseDoubleList(dict["vehicle"][0]);

		if (vehiclepose.Length != 7) {
			throw new FormatException("Vehicle description must have exactly seven arguments: x, y, z (location), theta, ax, ay, az (rotation axis)");
		}
		
		double[] location = new double[3] {vehiclepose[0], vehiclepose[1], vehiclepose[2]};
		double   angle    = vehiclepose[3];
		double[] axis     = new double[3] {vehiclepose[4], vehiclepose[5], vehiclepose[6]};

		List<double[]> maploc      = new List<double[]>();
		List<string>   mapdescript = dict["landmarks"];

		for (int i = 0; i < mapdescript.Count; i++) {
			double[] landmark = ParseDoubleList(mapdescript[i]);

			if (landmark.Length != 3) {
				throw new FormatException("Map landmarks must be 3D");
			}

			maploc.Add(landmark);
		}

		return new SimulatedVehicle(location, angle, axis, maploc);
	}
	
	/// <summary>
	/// Create a vehicle from a real sensor device.
	/// </summary>
	/// <param name="scene">Device (or recorded file) path.</param>
	/// <param name="mapclip">Secondary output; initial observable area.</param>
	/// <returns>Vehicle linked to sensor.</returns>
	private static Vehicle vehicleFromSensor(string sensor, out float[] mapclip)
	{
		mapclip = new float[4] {-6, 6, -3, 3};
		return new KinectVehicle(sensor);
	}

	/// <summary>
	/// Create a command list from a descriptor string array.
	/// </summary>
	/// <param name="commandstr">Command descriptor array.</param>
	/// <returns>Command list.</returns>
	private static CircularBuffer<double[]> commandsFromDescriptor(string[] commandstr)
	{
		if (commandstr.Length < 1) {
			commandstr = new string[1] {"0 0 0 0 0 0 0"};
		}

		CircularBuffer<double[]> commands = new CircularBuffer<double[]>(commandstr.Length);

		for (int i = 0; i < commandstr.Length; i++) {
			commands.Add(ParseDoubleList(commandstr[i]));
			// the item structure is {dlocx, dlocy, dlocz, dyaw, dpitch, droll, dcamera}
		}

		return commands;
	}
	

	/// <summary>
	/// Allow the simulation to perform any initialization it needs to do before it starts running.
	/// </summary>
	protected override void Initialize()
	{
		commandindex = 0;
		base.Initialize();
	}
	
	/// <summary>
	/// LoadContent will be called once per simulation and is the place to load
	/// all of your content.
	/// </summary>
	///protected override void LoadContent()
	///{
	///	base.LoadContent();
	///}

	/// <summary>
	/// Allows the simulation to run logic such as updating the world,
	/// checking for collisions, gathering input, and playing audio.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="keyboard">Current keyboard state information.</param>
	/// <param name="prevkeyboard">Old keyboard state information. Used to get newly pressed keys.</param>
	/// <param name="multiplier">Movement scale multiplier.</param>
	protected override void Update(GameTime time, KeyboardState keyboard, KeyboardState prevkeyboard, double multiplier)
	{
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
			if (commandindex < Commands.Size) {
				double[] autocmd = Commands.Next();
		
				dlocx  += autocmd[0];
				dlocy  += autocmd[1];
				dlocz  += autocmd[2];
				dyaw   += autocmd[3];
				dpitch += autocmd[4];
				droll  += autocmd[5];

				commandindex++;
			}
			else if (Commands.Size != 1) {
				Paused = true;
				commandindex = 0;
			}

			bool DoPredict = !keyboard.IsKeyDown(Keys.P);
			bool DoCorrect = !keyboard.IsKeyDown(Keys.C);
			bool DoPrune   = !keyboard.IsKeyDown(Keys.Q);

			Explorer.Update(time, dlocx, dlocy, dlocz, dyaw, dpitch, droll);

			if (UseOdometry) {
				Navigator.Update(time, dlocx, dlocy, dlocz, dyaw, dpitch, droll);
			}
			else {
				Navigator.Update(time, 0, 0, 0, 0, 0, 0);
			}

			if (time.TotalGameTime - lastnavigationupdate.TotalGameTime >= MeasureElapsed) {
				List<double[]> measurements = Explorer.Measure();
			
				Navigator.SlamUpdate(time, measurements, DoPredict, DoCorrect, DoPrune);

				lastnavigationupdate = new GameTime(time.TotalGameTime, time.ElapsedGameTime);
			}

			if (forcehistoryreset) {
				Navigator.ResetHistory();
				Explorer .ResetHistory();
			}

			if (forcereset) {
				Navigator.ResetModels();
			}

			if (forceslam) {
				Navigator.StartSlam(ParticleCount);
			}
			else if (forcemapping) {
				Navigator.StartMapping();
			}
		}
	}

	/// <summary>
	/// Get an array of doubles from a space-separated string descriptor.
	/// </summary>
	/// <param name="descriptor">String representing the list. Separated by spaces.</param>
	/// <returns>The double array.</returns>
	public static double[] ParseDoubleList(string descriptor) {
		string[] values = descriptor.Split(' ');
		double[] point = new double[values.Length];
		
		try {
			for (int i = 0; i < point.Length; i++) {
				point[i] = double.Parse(values[i]);
			}
		}
		catch (FormatException) {
			throw new FormatException("the double descriptor '" + descriptor + "' is malformed");
		}

		return point;
	}
}
}
