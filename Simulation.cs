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
/// Vehicle navigation simulation through a landmark map.
/// </summary>
public class Simulation : Game
{
	/// <summary>
	/// Measure cycle period in seconds. Every this
	/// amount of time the SLAM solver is invoked.
	/// </summary>
	public const double MeasurePeriod = 1.0/10;

	/// <summary>
	/// Simulation frame period (frames per second inverse).
	/// </summary>
	public readonly TimeSpan FrameElapsed = new TimeSpan(10000000/60);

	/// <summary>
	/// If true, the calculations are bound by realtime constraints,
	/// i.e. if it takes too long, the vehicle will move proportionally.
	/// Otherwise, the simulated timestep is always the same, regardless of
	/// how long the operations take.
	/// </summary>
	public const bool Realtime = false;

	/// <summary>
	/// Simulated time.
	/// </summary>
	private GameTime simtime = new GameTime();

	/// <summary>
	/// Main vehicle.
	/// </summary>
	public Vehicle Explorer { get; private set; }

	/// <summary>
	/// Map description through points of interest
	/// </summary>
	public List<double[]> Landmarks { get; private set; }

	/// <summary>
	/// SLAM solver.
	/// </summary>
	public Navigator Navigator { get; private set; }

	/// <summary>
	/// Cached measurements from the update process for rendering purposes.
	/// </summary>
	public List<double[]> MeasurementReadings;

	// MonoGame-related Fields

	/// <summary>
	/// Manager responsible for rendering.
	/// </summary>
	private GraphicsDeviceManager graphicsManager;

	/// <summary>
	/// Render output.
	/// </summary>
	private GraphicsDevice graphics;

	/// <summary>
	/// Rendering shader definition.
	/// </summary>
	private BasicEffect effect;

	/// <summary>
	/// Double buffer.
	/// </summary>
	public RenderTarget2D Buffer { get; private set; }

	/// <summary>
	/// Double batch flipper.
	/// </summary>
	private SpriteBatch flip;

	/// <summary>
	/// Double buffer flipping destination rectangle.
	/// </summary>
	private Rectangle bufferdest;

	/// <summary>
	/// Map clipping area.
	/// It is formatted as [left, right, bottom, top].
	/// </summary>
	private float[] mapclip;

	/// <summary>
	/// Camera angle.
	/// </summary>
	private double camangle;

	/// <summary>
	/// Cmaera matrix.
	/// </summary>
	private double[,] camera;
	
	/// <summary>
	/// Last time the navigator update method was called.
	/// </summary>
	private GameTime lastnavigationupdate = new GameTime();

	/// <summary>
	/// Automatic simulation command input.
	/// </summary>
	private CircularBuffer<double[]> Commands;

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
	/// Construct a simulation from a formatted desccription file.
	/// </summary>
	/// <param name="scene">Scene descriptor filename.</param>
	/// <param name="commands">Vehicle input command instructions filename. Can be empty (no automatic instructions).</param>
	public Simulation(string scene, string commands)
	{
		initScene(File.ReadAllText(scene));

		this.Navigator = new Navigator(Explorer, 50, false);

		try {
			initCommands(File.ReadAllLines(commands));
		}
		catch (FileNotFoundException) {
			Commands = new CircularBuffer<double[]>(1);
			Commands.Add(new double[5] {0, 0, 0, 0, 0});
		}

		// MonoGame-related construction
		this.graphicsManager = new GraphicsDeviceManager(this);

		this.Content.RootDirectory = "Content";
		this.IsMouseVisible        = true;

		this.graphicsManager.PreferredBackBufferWidth  = (int)(800*1.2);
		this.graphicsManager.PreferredBackBufferHeight = (int)(450*1.2);
		this.graphicsManager.PreferMultiSampling       = true;
		this.graphicsManager.IsFullScreen              = false;


		this.bufferdest = clipCenter(graphicsManager.PreferredBackBufferWidth,
		                             graphicsManager.PreferredBackBufferHeight,
		                             (mapclip[1] - mapclip[0]) / (mapclip[3] - mapclip[2]));

		camangle = 0;
		camera   = Accord.Math.Matrix.Identity(3);
	}

	/// <summary>
	/// Initialize the scene.
	/// </summary>
	/// <param name="scene">Scene descriptor text.</param>
	private void initScene(string scene)
	{
		Dictionary<string, List<string>> dict = Util.ParseDictionary(scene);

		this.mapclip = Accord.Math.Matrix.ToSingle(ParseDoubleList(dict["world"][0]));

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

		this.Landmarks           = new List<double[]>();
		this.MeasurementReadings = new List<double[]>();
		this.Explorer            = new Vehicle(location, angle, axis, this.Landmarks);

		for (int i = 0; i < maploc.Count; i++) {
			this.Landmarks.Add(maploc[i]);
		}
	}

	/// <summary>
	/// Read and initialize the command list.
	/// </summary>
	/// <param name="commands">Command descriptor array.</param>
	private void initCommands(string[] commandstr)
	{
		if (commandstr.Length < 1) {
			commandstr = new string[1] {"0 0 0 0 0"};
		}

		Commands = new CircularBuffer<double[]>(commandstr.Length);

		for (int i = 0; i < commandstr.Length; i++) {
			Commands.Add(ParseDoubleList(commandstr[i]));
			// the item structure is {ds, dyaw, dpitch, droll, dcamera}
		}
	}

	/// <summary>
	/// Get the biggest centered rectangle of the specified
	/// aspect ratio tha fits inside a screen of specified size.
	/// </summary>
	/// <param name="width">Screen width.</param>
	/// <param name="height">Screen height.</param>
	/// <param name="aspectratio">Target aspect ratio.</param>
	/// <returns> Biggest centered rectangle.</returns>
	private Rectangle clipCenter(int width, int height, float aspect)
	{
		float rectheight = height;
		float rectwidth  = rectheight * aspect;

		if (rectwidth > width) {
			rectwidth  = width;
			rectheight = rectwidth / aspect;
		}
		
		int offx = (int)((width  - rectwidth)  / 2);
		int offy = (int)((height - rectheight) / 2);

		return new Rectangle(offx, offy, (int) rectwidth, (int) rectheight);
	}
	

	/// <summary>
	/// Allows the game to perform any initialization it needs to before starting to run.
	/// </summary>
	protected override void Initialize()
	{
		this.graphics           = this.graphicsManager.GraphicsDevice;
		this.Explorer .Graphics = this.graphics;
		this.Navigator.Graphics = this.graphics;

		this.graphics.BlendState        = BlendState.NonPremultiplied;
		this.graphics.DepthStencilState = DepthStencilState.Default;
		this.graphics.RasterizerState   = RasterizerState.CullNone;
		this.graphics.SamplerStates[0]  = SamplerState.LinearClamp;

		this.effect            = new BasicEffect(this.graphics);
		this.effect.Alpha      = 1.0f;
		this.effect.View       = Microsoft.Xna.Framework.Matrix.Identity;
		this.effect.World      = Microsoft.Xna.Framework.Matrix.Identity;
		this.effect.Projection = Microsoft.Xna.Framework.Matrix.CreateOrthographicOffCenter(mapclip[0], mapclip[1], mapclip[2], mapclip[3], -100, 100);

		this.effect.LightingEnabled    = false;
		this.effect.VertexColorEnabled = true;

		this.Buffer = new RenderTarget2D(graphics, 2 * bufferdest.Width, 2 * bufferdest.Height,
		                                 false, SurfaceFormat.Color, DepthFormat.Depth16,
		                                 0, RenderTargetUsage.DiscardContents);

		//this.IsFixedTimeStep = false;

		base.Initialize();
	}
	
	/// <summary>
	/// LoadContent will be called once per game and is the place to load
	/// all of your content.
	/// </summary>
	protected override void LoadContent()
	{
		this.flip = new SpriteBatch(this.graphics);
	}

	/// <summary>
	/// Allows the game to run logic such as updating the world,
	/// checking for collisions, gathering input, and playing audio.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected override void Update(GameTime time)
	{
		simtime = Realtime ? time : new GameTime(simtime.TotalGameTime.Add(FrameElapsed), FrameElapsed);

		// TODO : force framerate constant
		KeyboardState keyboard = Keyboard.GetState();

		double ds     = 0;
		double dyaw   = 0;
		double dpitch = 0;
		double droll  = 0;
		double dcam   = 0;
		
		bool fast = keyboard.IsKeyDown(Keys.LeftShift);
		bool slow = keyboard.IsKeyDown(Keys.LeftControl);

		double multiplier = 1.0;
		multiplier *= (fast) ? 2.0 : 1.0;
		multiplier /= (slow) ? 4.0 : 1.0;
		
		if (keyboard.IsKeyDown(Keys.I)) {
			ds += 0.02 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.K)) {
			ds -= 0.02 * multiplier;
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

		if (keyboard.IsKeyDown(Keys.B)) {
			dcam += 0.06 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.V)) {
			dcam -= 0.06 * multiplier;
		}

		double[] autocmd = Commands.Next();
		
		ds     += autocmd[0];
		dyaw   += autocmd[1];
		dpitch += autocmd[2];
		droll  += autocmd[3];
		dcam   += autocmd[4];
		
		bool DoPredict = !keyboard.IsKeyDown(Keys.P);
		bool DoCorrect = !keyboard.IsKeyDown(Keys.C);
		bool DoPrune   = !keyboard.IsKeyDown(Keys.Q);

		Explorer.Update (simtime, 0, 0, ds, dyaw, dpitch, droll);
		Navigator.Update(simtime, 0, 0, ds, dyaw, dpitch, droll);

		camangle += dcam;
		camera    = MatrixExtensions.CreateRotationX(camangle);

		if (simtime.TotalGameTime.TotalSeconds - lastnavigationupdate.TotalGameTime.TotalSeconds > MeasurePeriod) {
			List<double[]> measurements = Explorer.Measure();

			Navigator.SlamUpdate(simtime, measurements, DoPredict, DoCorrect, DoPrune);

			lastnavigationupdate = new GameTime(simtime.TotalGameTime, simtime.ElapsedGameTime);

			MeasurementReadings = new List<double[]>();
			foreach (double[] z in measurements) {
				MeasurementReadings.Add(Explorer.MeasureToMap(z));
			}
		}

		base.Update(time);
	}

	/// <summary>
	/// This is called when the game should draw itself.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected override void Draw(GameTime time)
	{
		this.graphics.SetRenderTarget(Buffer);
		this.graphics.Clear(Color.DarkSeaGreen);

		foreach (EffectPass pass in effect.CurrentTechnique.Passes) {
			pass.Apply();
			
			Explorer .RenderFOV(camera);
			Navigator.RenderTrajectory(camera);
			Explorer .RenderTrajectory(camera);
			

			foreach (double[] landmark in Landmarks) {
				RenderLandmark(landmark, camera);
			}

			foreach (double[] measure in MeasurementReadings) {
				RenderMeasure(measure, camera);
			}

			Explorer .RenderBody(camera);
			Navigator.Render(camera);
		}

		graphics.SetRenderTarget(null);
		flip.Begin(SpriteSortMode.Immediate, BlendState.NonPremultiplied, SamplerState.LinearClamp,
		           DepthStencilState.Default, RasterizerState.CullNone);

		flip.Draw(Buffer, bufferdest, Color.White);

		//Console.WriteLine(1.0/time.ElapsedGameTime.TotalSeconds);

		flip.End();

		base.Draw(time);
	}

	/// <summary>
	/// Simple point landmark rendering.
	/// </summary>
	/// <param name="landmark">Point landmark position.</param>
	/// <param name="camera">Camera rotation matrix.</param>
	private void RenderLandmark(double[] landmark, double[,] camera)
	{
		const float halflen = 0.024f;
		
		Color innercolor =  Color.LightGray;
		Color outercolor =  Color.Black;

		landmark = camera.Multiply(landmark);
		
		VertexPositionColor[] invertices  = new VertexPositionColor[4];
		double[][]            outvertices = new double[4][];

		outvertices[0] = new double[] {landmark[0] - halflen, landmark[1] - halflen, landmark[2]};
		outvertices[1] = new double[] {landmark[0] - halflen, landmark[1] + halflen, landmark[2]};
		outvertices[2] = new double[] {landmark[0] + halflen, landmark[1] + halflen, landmark[2]};
		outvertices[3] = new double[] {landmark[0] + halflen, landmark[1] - halflen, landmark[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		graphics.DrawUser2DPolygon(outvertices, 0.02f, outercolor, true);
	}

	/// <summary>
	/// Simple point measurement rendering.
	/// </summary>
	/// <param name="measurement">Point measurement position.</param>
	/// <param name="camera">Camera rotation matrix.</param>
	private void RenderMeasure(double[] measurement, double[,] camera)
	{
		const float halflen = 0.03f;
		
		Color color =  Color.Crimson;

		measurement = camera.Multiply(measurement);
		
		double[][] vertices = new double[2][];

		vertices[0] = new double[] {measurement[0] - halflen, measurement[1] - halflen, measurement[2]};
		vertices[1] = new double[] {measurement[0] + halflen, measurement[1] + halflen, measurement[2]};
		
		graphics.DrawUser2DPolygon(vertices, 0.01f, color, true);

		vertices[0] = new double[] {measurement[0] - halflen, measurement[1] + halflen, measurement[2]};
		vertices[1] = new double[] {measurement[0] + halflen, measurement[1] - halflen, measurement[2]};

		graphics.DrawUser2DPolygon(vertices, 0.01f, color, true);
	}

	/// <summary>
	/// Get an array of doubles from a space-separated string descriptor.
	/// </summary>
	/// <param name="descriptor">String representing the list. Separated by spaces.</param>
	/// <returns>The double array.</returns>
	public double[] ParseDoubleList(string descriptor) {
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
