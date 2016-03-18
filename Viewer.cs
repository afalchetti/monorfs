// Viewer.cs
// Prerecorded manipulator
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

using Accord.Extensions.Imaging;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

using FP = monorfs.FileParser;

using TimedState        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedTrajectory   = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<System.Tuple<double, double[]>>>>;
using TimedMapModel     = System.Collections.Generic.List<System.Tuple<double, monorfs.Map>>;
using TimedMeasurements = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<double[]>>>;
using TimedMessage      = System.Collections.Generic.List<System.Tuple<double, string>>;

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
	public TimedTrajectory Estimate { get; private set; }

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
	/// Obtain vehicle history that corresponds to the current indexed time.
	/// </summary>
	public TimedState VehicleWaypoints
	{
		get {
			TimedState waypoints = new TimedState();

			for (int k = 0; k <= FrameIndex; k++) {
				waypoints.Add(Trajectory[k]);
			}

			return waypoints;
		}
	}

	/// <summary>
	/// Sidebar video filename.
	/// </summary>
	private string sidebarfile;
	
	/// <summary>
	/// Every recorded sidebar frame.
	/// </summary>
	public List<Texture2D> SidebarHistory { get; private set; }

	/// <summary>
	/// "overriden" SideBuffer image.
	/// </summary>
	private Texture2D SideBuffer2;

	/// <summary>
	/// Mapping between trajectory indices and equivalent map indices (which may have a slower framerate).
	/// </summary>
	private int[] mapindices;

	/// <summary>
	/// Frame index currently shown onscreen.
	/// </summary>
	private int FrameIndex;

	/// <summary>
	/// Smooth tag color updater generator.
	/// </summary>
	private IEnumerator<Color> tagupdater;

	/// <summary>
	/// True if the tag is no longer being displayed onscreen.
	/// </summary>
	private bool taggone;

	/// <summary>
	/// True if the user has added new tags, so the file should be rewritten.
	/// </summary>
	public bool TagChanged { get; private set; }

	/// <summary>
	/// Construct a visualization from its components.
	/// </summary>
	/// <param name="title">Window title.</param>
	/// <param name="explorer">Main vehicle in the visualization.</param>
	/// <param name="trajectory">Recorded vehicle trajectory.</param>
	/// <param name="estimate">Recorded estimated vehicle trajectory.</param>
	/// <param name="map">Recorded maximum-a-posteriori estimate for the map.</param>
	/// <param name="measurements">Recorded vehicle measurements.</param>
	/// <param name="tags">Tags in the timeline.</param>
	/// <param name="fps">Frame rate.</param>
	/// <param name="sidebarfile">Sidebar video filename.</param>
	public Viewer(string title, SimulatedVehicle explorer, TimedState trajectory, TimedTrajectory estimate,
	              TimedMapModel map, TimedMeasurements measurements, TimedMessage tags,
	              double fps, string sidebarfile)
		: base(title, explorer, new PHDNavigator(explorer, 1, false), false, fps)
	{
		Trajectory   = trajectory;
		Estimate     = estimate;
		Map          = map;
		Measurements = measurements;
		Tags         = tags;
		FrameIndex   = 0;
		TagChanged   = false;

		while (Measurements.Count < map.Count) {
			Measurements.Add(Tuple.Create(map[Measurements.Count].Item1, new List<double[]>()));
		}

		mapindices = new int[trajectory.Count];

		int h = 0;
		for (int k = 0; k < estimate.Count; k++) {
			while (h < map.Count - 1 && map[h].Item1 < estimate[k].Item1) {
				h++;
			}

			mapindices[k] = h;
		}

		// note that the sidebar video read has to wait until LoadContent to get an appropiate Graphics context
		this.sidebarfile = sidebarfile;
		IsFixedTimeStep  = true;

		// override base class behavior (that is based on explorer HasSidebar)
		if (!string.IsNullOrEmpty(sidebarfile)) {
			ScreenCut = 0.7;
		}
	}

	/// <summary>
	/// Create a visualization object from a pair of formatted description files.
	/// </summary>
	/// <param name="datafile">Compressed prerecorded data file.</param>
	/// <param name="filterhistory">If true, show the filtered trajectory history, otherwise, the smooth one,
	/// i.e. the smooth history may change retroactively from future knowledge. Note however that the
	/// recorded vehicle may not support smoothing and may perform the same in both modes.
	/// Note also that this is not the algorithm mode, only the visualization; the algorithm
	/// could still take past information into account, but it won't be visualized.</param> 
	/// <param name="tmpdir">Temporary data directory, to be removed after use.</param>
	/// <returns>Prepared visualization object.</returns>
	/// <remarks>All file must be previously sorted by time value. This property is assumed.</remarks>
	public static Viewer FromFiles(string datafile, bool filterhistory, out string tmpdir)
	{
		tmpdir         = Util.TemporaryDir();
		string datadir = Path.Combine(tmpdir, "data");

		ZipFile.ExtractToDirectory(datafile, datadir);

		string scenefile    = Path.Combine(datadir, "scene.world");
		string vehiclefile  = Path.Combine(datadir, "trajectory.out");
		string estimatefile = Path.Combine(datadir, "estimate.out");
		string mapfile      = Path.Combine(datadir, "maps.out");
		string measurefile  = Path.Combine(datadir, "measurements.out");
		string tagfile      = Path.Combine(datadir, "tags.out");
		string sidebarfile  = Path.Combine(datadir, "sidebar.avi");

		if (!File.Exists(scenefile)) {
			scenefile = "";
		}

		if (!File.Exists(measurefile)) {
			measurefile = "";
		}

		if (!File.Exists(tagfile)) {
			tagfile = "";
		}

		if (!File.Exists(sidebarfile)) {
			sidebarfile = "";
		}

		SimulatedVehicle  explorer;
		TimedState        trajectory;
		TimedTrajectory   estimate;
		TimedMapModel     map;
		TimedMeasurements measurements;
		TimedMessage      tags;

		trajectory   = FP.TimedArrayFromDescriptor       (File.ReadAllLines(vehiclefile),  7);
		estimate     = FP.TrajectoryHistoryFromDescriptor(File.ReadAllText(estimatefile), 7, filterhistory);
		map          = FP.MapHistoryFromDescriptor       (File.ReadAllText(mapfile));

		if (!string.IsNullOrEmpty(measurefile)) {
			measurements = FP.MeasurementsFromDescriptor(File.ReadAllText(measurefile));
		}
		else {
			measurements = new TimedMeasurements();
		}

		if (!string.IsNullOrEmpty(tagfile)) {
			tags = FP.TimedMessageFromDescriptor(File.ReadAllLines(tagfile));
		}
		else {
			tags = new TimedMessage();
		}

		if (!string.IsNullOrEmpty(scenefile)) {
			explorer = FP.VehicleFromSimFile(File.ReadAllText(scenefile));
		}
		else {
			explorer = new SimulatedVehicle(Pose3D.Identity, new List<double[]>());
		}

		return new Viewer("monorfs - viewing " + datafile, explorer, trajectory, estimate, map, measurements, tags, 30, sidebarfile);
	}

	/// <summary>
	/// Allow the manipulator to perform any initialization it needs to do before it starts running.
	/// </summary>
	protected override void Initialize()
	{
		taggone     = true;
		tagupdater  = System.Linq.Enumerable.Empty<Color>().GetEnumerator();
		tagupdater.MoveNext();

		base.Initialize();
	}

	/// <summary>
	/// Load any multimedia content that the object requires to draw correctly on screen.
	/// </summary>
	protected override void LoadContent()
	{
		Texture2D nulltexture = new Texture2D(Graphics, 1, 1);

		if (!string.IsNullOrEmpty(sidebarfile)) {
			SidebarHistory = frameListFromAvi(sidebarfile);
			SidebarHistory.Insert(0, nulltexture);

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
	/// Allow the viewer to run logic such as updating the world,
	/// and gathering input.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="keyboard">Current keyboard state information.</param>
	/// <param name="prevkeyboard">Old keyboard state information. Used to get newly pressed keys.</param>
	/// <param name="multiplier">Movement scale multiplier.</param>
	protected override void Update(GameTime time, KeyboardState keyboard, KeyboardState prevkeyboard, double multiplier)
	{
		if (FrameIndex >= Trajectory.Count) {
			FrameIndex = Trajectory.Count - 1;
			Paused     = true;
		}
		else if (FrameIndex < 0) {
			FrameIndex = 0;
			Paused     = true;
		}

		bool speedup = keyboard.IsKeyDown(Keys.LeftShift);

		SideBuffer2            = SidebarHistory[FrameIndex];
		Explorer .WayPoints    = VehicleWaypoints;
		Explorer .Pose         = new Pose3D(Explorer.WayPoints[Explorer.WayPoints.Count - 1].Item2);
		Navigator.BestMapModel = Map[mapindices[FrameIndex]].Item2;
		Navigator.BestEstimate.WayPoints = Estimate[FrameIndex].Item2;
		Navigator.BestEstimate.Pose      = new Pose3D(Navigator.BestEstimate.WayPoints[Navigator.BestEstimate.WayPoints.Count - 1].Item2);

		Explorer.MappedMeasurements.Clear();
		foreach (double[] z in Measurements[mapindices[FrameIndex]].Item2) {
			Explorer.MappedMeasurements.Add(Explorer.MeasureToMap(z));
		}

		if (!taggone) {
			TagColor = tagupdater.Current;
			taggone  = !tagupdater.MoveNext();
		}

		double viewtime = Estimate[FrameIndex].Item1;
		int    tagindex = Tags.BinarySearch(Tuple.Create(viewtime, ""));

		if (tagindex != ~Tags.Count) {
			if (tagindex < 0) {
				tagindex = ~tagindex;
			}

			if (Math.Abs(Tags[tagindex].Item1 - viewtime) < 1e-5) {
				TagMessage = Tags[tagindex].Item2;
				tagupdater = smoothFader(Color.White, Color.DimGray);
				taggone    = !tagupdater.MoveNext();
			}
		}

		Message = string.Format("time = {0,12:N4}        frame = {1,5:N0}", viewtime, FrameIndex);
		
		// frame-by-frame fine time lookup, reverse
		if (keyboard.IsKeyDown(Keys.Q) && !prevkeyboard.IsKeyDown(Keys.Q)) {
			FrameIndex -= (speedup) ? 8 : 1;
			Paused      = true;
		}
		
		// frame-by-frame, forward
		if (keyboard.IsKeyDown(Keys.W) && !prevkeyboard.IsKeyDown(Keys.W)) {
			FrameIndex += (speedup) ? 8 : 1;
			Paused      = true;
		}

		// normal speed, reverse
		if (keyboard.IsKeyDown(Keys.A)) {
			FrameIndex -= (speedup) ? 8 : 1;
			Paused      = true;
		}

		// normal speed, forward
		if (keyboard.IsKeyDown(Keys.S)) {
			Paused = false;
		}

		// add a new tag
		if (keyboard.IsKeyDown(Keys.G) && !prevkeyboard.IsKeyDown(Keys.G)) {
			Tags.Add(Tuple.Create(viewtime, "User tag"));
			Tags.Sort();
			TagChanged = true;
		}

		if (!Paused) {
			FrameIndex += (speedup) ? 8 : 1;
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
		Graphics.SetRenderTarget(null);
		Flip.Begin(SpriteSortMode.Immediate, BlendState.NonPremultiplied, SamplerState.LinearClamp,
		           DepthStencilState.Default, RasterizerState.CullNone);

		Flip.Draw(SideBuffer2, SideDest, SideBuffer2.Bounds, Color.White);
		
		Flip.End();
	}

	/// <summary>
	/// Create a generator of smooth transition between colors.
	/// </summary>
	/// <param name="initial">Initial color.</param>
	/// <param name="final">Final color.</param>
	/// <returns>Color transition generator.</returns>
	private IEnumerator<Color> smoothFader(Color initial, Color final)
	{
		for (int x = 0; x <= TransitionFrames; x++) {
			double alpha = Util.SmoothTransition((double) x / TransitionFrames);
			yield return Color.Lerp(initial, final, (float) alpha);
		}
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
}
}
