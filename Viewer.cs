// Viewer.cs
// Prerecorded manipulator
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

using Accord.Extensions.Imaging;
using Accord.Math;
using System.Text;

using TimedState        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel     = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<monorfs.Gaussian>>>;
using TimedMeasurements = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<double[]>>>;

namespace monorfs
{
/// <summary>
/// Prerecorded file viewer. This module does not use real input from a slam algorithm
/// and instead strictly follows a predefined motion and estimation. It is useful to
/// consistently re-analyze an experiment, as well as to present such results to a third party.
/// It is also useful as a cache for long simulations, e.g. when using thousands of particles.
/// </summary>
public class Viewer : Manipulator
{
	/// <summary>
	/// Recorded vehicle trajectory indexed by time.
	/// The first entry of each tuple is a time point in seconds; the second is the state.
	/// </summary>
	public TimedState Trajectory { get; private set; }

	/// <summary>
	/// Recorded estimated vehicle trajectory indexed by time.
	/// The first entry of each tuple is a time point in seconds; the second is the state.
	/// </summary>
	public TimedState Estimate { get; private set; }

	/// <summary>
	/// Recorded maximum-a-posteriori estimate for the map indexed by time.
	/// The first entry of each tuple is a time point in seconds; the second is the map model.
	/// </summary>
	public TimedMapModel Map { get; private set; }

	/// <summary>
	/// Recorded vehicle sensed noisy measurements.
	/// The first entry of each tuple is a time point in seconds; the second is the measurement list.
	/// </summary>
	public TimedMeasurements Measurements { get; private set; }

	/// <summary>
	/// Sidebar video filename.
	/// </summary>
	private string sidebarfile;
	
	/// <summary>
	/// Every recorded sidebar frame.
	/// </summary>
	public List<Texture2D> SidebarHistory { get; private set; }

	private Texture2D SideBuffer2;

	/// <summary>
	/// Vehicle waypoints internal cache.
	/// </summary>
	private List<double[]> vehiclewaypoints;

	/// <summary>
	/// Obtain vehicle history that corresponds to the current indexed time.
	/// </summary>
	public List<double[]> VehicleWaypoints
	{
		get {
			List<double[]> waypoints = new List<double[]>();

			for (int k = 0; k < i; k++) {
				waypoints.Add(vehiclewaypoints[k]);
			}

			return waypoints;
		}
	}

	/// <summary>
	/// Estimate waypoints internal cache.
	/// </summary>
	private List<double[]> estimatewaypoints;

	/// <summary>
	/// Obtain estimate history that corresponds to the current indexed time.
	/// </summary>
	public List<double[]> EstimateWaypoints
	{
		get {
			List<double[]> waypoints = new List<double[]>();

			for (int k = 0; k < i; k++) {
				waypoints.Add(estimatewaypoints[k]);
			}

			return waypoints;
		}
	}

	/// <summary>
	/// Mapping between trajectory indices and equivalent map indices (which may have a slower framerate).
	/// </summary>
	private int[] mapindices;

	/// <summary>
	/// Frame index currently shown onscreen.
	/// </summary>
	private int i;

	/// <summary>
	/// Construct a visualization from its components.
	/// </summary>
	/// <param name="explorer">Main vehicle in the visualization.</param>
	/// <param name="trajectory">Recorded vehicle trajectory.</param>
	/// <param name="estimate">Recorded estimated vehicle trajectory.</param>
	/// <param name="map">Recorded maximum-a-posteriori estimate for the map.</param>
	/// <param name="fps">Frame rate.</param>
	/// <param name="mapclip">Initial observable area in the form [left, right, bottom, top]</param>
	/// <param name="sidebarfile">Sidebar video filename.</param>
	public Viewer(SimulatedVehicle explorer, TimedState trajectory, TimedState estimate,
	              TimedMapModel map, TimedMeasurements measurements,
	              double fps, float[] mapclip, string sidebarfile)
		: base(explorer, new Navigator(explorer, 1, false), 1, false, mapclip, fps)
	{
		Trajectory   = trajectory;
		Estimate     = estimate;
		Map          = map;
		Measurements = measurements;
		i            = 0;

		vehiclewaypoints  = new List<double[]>();
		estimatewaypoints = new List<double[]>();
		
		foreach (var point in trajectory) {
			vehiclewaypoints.Add(new double[1] {point.Item1}.Concatenate(point.Item2));
		}

		foreach (var point in estimate) {
			estimatewaypoints.Add(new double[1] {point.Item1}.Concatenate(point.Item2));
		}

		while (Measurements.Count < map.Count) {
			Measurements.Add(Tuple.Create(map[Measurements.Count].Item1, new List<double[]>()));
		}

		mapindices = new int[trajectory.Count];

		int h = 0;
		for (int k = 0; k < trajectory.Count; k++) {
			while (h < map.Count - 1 && map[h].Item1 < trajectory[k].Item1) {
				h++;
			}

			mapindices[k] = h;
		}

		// note that the sidebar video read has to wait until LoadContent to get an appropiate Graphics context
		this.sidebarfile = sidebarfile;
		IsFixedTimeStep  = true;
	}

	/// <summary>
	/// Create a visualization object from a pair of formatted description files.
	/// </summary>
	/// <param name="vehiclefile">Vehicle trajectory descriptor file.</param>
	/// <param name="estimatefile">Vehicle trajectory estimation descriptor file.</param>
	/// <param name="mapfile">Map estimation descriptor file.</param>
	/// <param name="sidebarfile">Sidebar history video file.</param>
	/// <param name="measurefile">Vehicle sensed measurements file, may be null or empty.</param>
	/// <param name="scenefile">Scene descriptor file, may be null or empty.</param>
	/// <returns>Prepared visualization object.</returns>
	/// <remarks>All file must be previously sorted by time value. This property is assumed.</remarks>
	public static Viewer FromFiles(string vehiclefile, string estimatefile, string mapfile, string sidebarfile = "", string measurefile = "", string scenefile = "")
	{
		SimulatedVehicle  explorer;
		TimedState        trajectory;
		TimedState        estimate;
		TimedMapModel     map;
		TimedMeasurements measurements;
		float[]           mapclip;

		trajectory   = trajectoryFromDescriptor(File.ReadAllLines(vehiclefile),  7);
		estimate     = trajectoryFromDescriptor(File.ReadAllLines(estimatefile), 3);
		map          = mapHistoryFromDescriptor(File.ReadAllText(mapfile));

		if (!string.IsNullOrEmpty(measurefile)) {
			measurements = measurementsFromDescriptor(File.ReadAllText(measurefile));
		}
		else {
			measurements = new TimedMeasurements();
		}

		if (!string.IsNullOrEmpty(scenefile)) {
			explorer = VehicleFromSimFile(File.ReadAllText(scenefile), out mapclip);
		}
		else {
			explorer = new SimulatedVehicle(new double[3] {0, 0, 0}, 0, new double[3] {0, 0, 1}, new List<double[]>());
			mapclip = new float[4] {-6, 6, -3, 3};
		}

		return new Viewer(explorer, trajectory, estimate, map, measurements, 30, mapclip, sidebarfile);
	}

	/// <summary>
	/// Load any multimedia content that the object requires to draw correctly on screen.
	/// </summary>
	protected override void LoadContent()
	{
		Texture2D nulltexture = new Texture2D(Graphics, 1, 1);

		if (!string.IsNullOrEmpty(sidebarfile)) {
			SidebarHistory = frameListFromAvi(sidebarfile);

			for (int i = SidebarHistory.Count; i < Trajectory.Count; i++) {
				SidebarHistory.Add(nulltexture);
			}
		}
		else {
			SidebarHistory = new List<Texture2D>();

			foreach (var state in Trajectory) {
				SidebarHistory.Add(nulltexture);
			}
		}

		base.LoadContent();
	}

	/// <summary>
	/// Get a trajectory from a formatted string descriptor.
	/// </summary>
	/// <param name="lines">Array of formatted state vectors moving through time.</param>
	/// <param name="dim">Expected state dimension.</param>
	/// <returns>The trajectory, list of vectors representing state as a function of discrete time
	/// (time is delivered as the first entry of each tuple).</returns>
	private static TimedState trajectoryFromDescriptor(string[] lines, int dim = 7)
	{
		TimedState trajectory = new TimedState();

		foreach (string line in lines) {
			double[] values = ParseDoubleList(line);

			if (values.Length != dim + 1) {  // an extra dimension for time
				throw new FormatException("wrong state dimension");
			}

			double   time  = values[0];
			double[] state = values.Submatrix(1, values.Length - 1);

			trajectory.Add(Tuple.Create(time, state));
		}

		return trajectory;
	}

	/// <summary>
	/// Get a map trajectory (or history) from a formatted string descriptor.
	/// </summary>
	/// <param name="descriptor">Formatted map descriptor moving through time.</param>
	/// <param name="dim">Expected world dimension.</param>
	/// <returns>The map history, list of vectors representing map model components as a function of discrete time
	/// (time is delivered as the first entry of each tuple).</returns>
	private static TimedMapModel mapHistoryFromDescriptor(string descriptor, int dim = 3)
	{
		string[]      frames  = descriptor.Split(new string[] {"\n|\n"}, StringSplitOptions.RemoveEmptyEntries);
		TimedMapModel history = new TimedMapModel();

		foreach (string frame in frames) {
			string[] lines = frame.Split(new char[] {'\n'}, StringSplitOptions.RemoveEmptyEntries);
			double time    = -1;

			try {
				time = double.Parse(lines[0]);
			}
			catch (FormatException) {
				throw new FormatException("bad map format: missing time");
			}

			history.Add(Tuple.Create(time, mapFromDescriptor(lines.Submatrix(1, lines.Length - 1), dim)));
		}

		return history;
	}

	/// <summary>
	/// Get a measurement history from a formatted string descriptor.
	/// </summary>
	/// <param name="descriptor">Formatted measurement history descriptor through time.</param>
	/// <param name="dim">Expected world dimension.</param>
	/// <returns>The measurement history, list of vectors representing point measurements as a function of discrete time
	/// (time is delivered as the first entry of each tuple).</returns>
	private static TimedMeasurements measurementsFromDescriptor(string descriptor, int dim = 3)
	{
		string[]          frames  = descriptor.Split('\n');
		TimedMeasurements history = new TimedMeasurements();

		foreach (string frame in frames) {
			string[] parts = frame.Split(':');
			double time    = -1;

			if (parts.Length != 2) {
				throw new FormatException("bad measurement format: no ':' delimiter found");
			}

			try {
				time = double.Parse(parts[0]);
			}
			catch (FormatException) {
				throw new FormatException("bad measurement format: missing time");
			}

			string[]       points       = parts[1].Split(';');
			List<double[]> measurements = new List<double[]>();

			foreach (string point in points) {
				if (point == "") {
					continue;
				}

				string[] strcomps   = point.Split(' ');
				double[] components = new double[strcomps.Length];

				for (int i = 0; i < strcomps.Length; i++) {
					try {
						components[i] = double.Parse(strcomps[i]);
					}
					catch (FormatException) {
						throw new FormatException("bad measurement format: invalid point");
					}
				}

				measurements.Add(components);
			}

			history.Add(Tuple.Create(time, measurements));
		}

		return history;
	}

	/// <summary>
	/// Get a list of frames from a video file in AVI format.
	/// </summary>
	/// <param name="filename">Input video filename.</param>
	/// <returns>Frame list.</returns>
	private List<Texture2D> frameListFromAvi(string filename)
	{
		List<Texture2D> frames = new List<Texture2D>();

		using (FileCapture reader = new FileCapture(filename)) {
			reader.Open();
			Console.WriteLine("reading file " + filename);

			foreach (Image<Bgr<byte>> image in reader) {
				Texture2D    frame  = new Texture2D(Graphics, image.Width, image.Height);
				Bgr<byte>[,] data   = new Bgr<byte>[image.Height, image.Width];
				Color[]      linear = new Color[image.Width * image.Height];

				image.CopyTo(data);

				int h = 0;
				for (int k = 0; k < image.Height; k++) {
				for (int i = 0; i < image.Width; i++) {
					linear[h++] = new Color(data[k, i].R, data[k, i].G, data[k, i].B);
				}
				}

				Console.WriteLine("frame " + reader.Position);
				frame.SetData(linear);
				frames.Add(frame);
			}

			Console.WriteLine("done.");
			reader.Close();
		}

		return frames;
	}

	/// <summary>
	/// Get a map model from a formatted string descriptor.
	/// </summary>
	/// <param name="descriptor">Formatted map descriptor.</param>
	/// <param name="dim">Expected world dimension.</param>
	/// <returns>The map model as a vector of gaussian components.</returns>
	private static List<Gaussian> mapFromDescriptor(string[] lines, int dim = 3)
	{
		List<Gaussian> map = new List<Gaussian>();

		for (int i = 0; i < lines.Length; i++) {
			Gaussian component = ParseGaussianDescriptor(lines[i]);
			map.Add(component);

			if (component.Mean.Length != dim) {
				throw new FormatException("wrong gaussian dimension");
			}
		}

		return map;
	}

	/// <summary>
	/// Allow the viewer to run logic such as updating the world,
	/// and gathering input.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="keyboard">Current keyboard state information.</param>
	/// <param name="prevkeyboard">Old keyboard state information. Used to get newly pressed keys.</param>
	/// <param name="multiplier">Movement scale multiplier.</param>
	protected override void Update(GameTime time, KeyboardState keyboard, KeyboardState prevkeyboard, double multiplier)
	{
		if (i >= Trajectory.Count) {
			i  = Trajectory.Count - 1;
			Paused = true;
		}
		else if (i < 0) {
			i = 0;
			Paused = true;
		}

		SideBuffer2            = SidebarHistory[i];
		Explorer .State        = Trajectory[i].Item2;
		Explorer .Waypoints    = VehicleWaypoints;
		Navigator.Waypoints    = EstimateWaypoints;
		Navigator.MapModels[0] = Map[mapindices[i]].Item2;
		Navigator.VehicleParticles[0].Waypoints = EstimateWaypoints;

		Explorer.MappedMeasurements.Clear();
		foreach (double[] z in Measurements[mapindices[i]].Item2) {
			Explorer.MappedMeasurements.Add(Explorer.MeasureToMap(z));
		}

		Message = string.Format("time = {0,12:N4}        frame = {1,5:N0}", Trajectory[i].Item1, i);
		
		// frame-by-frame fine time lookup, reverse
		if (keyboard.IsKeyDown(Keys.Q) && !prevkeyboard.IsKeyDown(Keys.Q)) {
			i--;
			Paused = true;
		}
		
		// frame-by-frame, forward
		if (keyboard.IsKeyDown(Keys.W) && !prevkeyboard.IsKeyDown(Keys.W)) {
			i++;
			Paused = true;
		}

		// normal speed, reverse
		if (keyboard.IsKeyDown(Keys.A)) {
			i--;
			Paused = true;
		}

		// normal speed, forward
		if (keyboard.IsKeyDown(Keys.S)) {
			Paused = false;
		}

		if (!Paused) {
			i++;
		}
	}
	
	/// <summary>
	/// This is called when the viewer should draw itself.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected override void Draw(GameTime time)
	{
		base.Draw(time);

		// overwrite the sidebar
		// TODO clean this up, shouldn't need to overwrite but instead use an unified interface.
		//      the problem probably is that explorer shouldn't be in charge of rendering, only
		//      providing the Texture2D and this should be overridable in the update method
		Graphics.SetRenderTarget(null);
		Flip.Begin(SpriteSortMode.Immediate, BlendState.NonPremultiplied, SamplerState.LinearClamp,
		           DepthStencilState.Default, RasterizerState.CullNone);

		Flip.Draw(SideBuffer2, sidedest, SideBuffer2.Bounds, Color.White);
		
		Flip.End();
	}
}
}
