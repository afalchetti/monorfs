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
	public const double MeasurePeriod = 1.0/30;

	/// <summary>
	/// Simulation frame period (frames per second inverse).
	/// </summary>
	public readonly TimeSpan FrameElapsed = new TimeSpan(10000000/30);

	/// <summary>
	/// If true, the calculations are bound by realtime constraints,
	/// i.e. if it takes too long, the vehicle will move proportionally.
	/// Otherwise, the simulated timestep is always the same, regardless of
	/// how long the operations take.
	/// </summary>
	public bool Realtime { get; private set; }

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
	/// Number of localization Montecarlo filter particles.
	/// </summary>
	public int ParticleCount { get; private set; }

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
	/// Scene double buffer.
	/// </summary>
	public RenderTarget2D SceneBuffer { get; private set; }

	/// <summary>
	/// Sidebar double buffer.
	/// </summary>
	public RenderTarget2D SideBuffer { get; private set; }

	/// <summary>
	/// Double batch flipper.
	/// </summary>
	private SpriteBatch flip;

	/// <summary>
	/// Previous update frame keyboard state.
	/// </summary>
	private KeyboardState prevkeyboard;
	
	/// <summary>
	/// Scene double buffer flipping destination rectangle.
	/// </summary>
	private Rectangle scenedest;

	/// <summary>
	/// Sidebar double buffer flipping destination rectangle.
	/// </summary>
	private Rectangle sidedest;

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
	/// Camera matrix.
	/// </summary>
	private double[][] camera;
	
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
	/// <param name="commands">Vehicle input command instructions filename.
	/// Can be null or empty (no automatic instructions).</param>
	/// <param name="particlecount">Number of particles for the RB-PHD algorithm.
	/// If only mapping is done, it is irrelevant.</param>
	/// <param name="onlymapping">If true, no localization is executed and the robot's position
	/// is assumed perfectly known.</param>
	/// <param name="simulate">True means a full simulation is performed;
	/// false uses real sensor data (realtime or from a file).</param>
	public Simulation(string scene, string commands = "", int particlecount = 5, bool onlymapping = false, bool simulate = true, bool realtime = false)
	{
		if (simulate) {
			initSceneFromSimFile(File.ReadAllText(scene));
		}
		else {
			initSceneFromSensor(scene);
		}

		ParticleCount = particlecount;
		Navigator     = new Navigator(Explorer, particlecount, onlymapping);
		Realtime      = realtime;

		try {
			if (!string.IsNullOrEmpty(commands)) {
				initCommands(File.ReadAllLines(commands));
			}
		}
		catch (FileNotFoundException) {
			commands = null;
		}

		if (string.IsNullOrEmpty(commands)) {
			Commands = new CircularBuffer<double[]>(1);
			Commands.Add(new double[5] {0, 0, 0, 0, 0});
		}

		// MonoGame-related construction
		graphicsManager = new GraphicsDeviceManager(this);

		Content.RootDirectory = "Content";
		IsMouseVisible        = true;

		graphicsManager.PreferredBackBufferWidth  = (int)(1000*1.0);
		graphicsManager.PreferredBackBufferHeight = (int)(450*1.0);
		graphicsManager.PreferMultiSampling       = true;
		graphicsManager.IsFullScreen              = false;

		const double screencut = 0.7;

		scenedest = clipCenter((int)(graphicsManager.PreferredBackBufferWidth * screencut),
		                       graphicsManager.PreferredBackBufferHeight,
		                       (mapclip[1] - mapclip[0]) / (mapclip[3] - mapclip[2]));

		sidedest = clipCenter((int)(graphicsManager.PreferredBackBufferWidth * (1 - screencut)),
		                      graphicsManager.PreferredBackBufferHeight,
		                      (float) Explorer.SidebarWidth / Explorer.SidebarHeight);

		sidedest.X += (int)(graphicsManager.PreferredBackBufferWidth * screencut);

		camangle = 0;
		camera   = Accord.Math.Matrix.Identity(3).ToArray();
	}

	/// <summary>
	/// Initialize the scene from a simulation file.
	/// </summary>
	/// <param name="scene">Scene descriptor text.</param>
	private void initSceneFromSimFile(string scene)
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

		Landmarks = new List<double[]>();
		Explorer  = new SimulatedVehicle(location, angle, axis, this.Landmarks);

		for (int i = 0; i < maploc.Count; i++) {
			Landmarks.Add(maploc[i]);
		}
	}

	/// <summary>
	/// Initialize the scene from a real sensor device.
	/// </summary>
	/// <param name="sensor">Device (or recorded file) path.</param>
	private void initSceneFromSensor(string sensor)
	{
		mapclip  = new float[4] {-6, 6, -3, 3};
		Explorer = new KinectVehicle(sensor);
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
	/// Allows the game to perform any initialization it needs to do before it starts running.
	/// </summary>
	protected override void Initialize()
	{
		graphics           = graphicsManager.GraphicsDevice;
		Explorer .Graphics = graphics;
		Navigator.Graphics = graphics;

		graphics.BlendState        = BlendState.NonPremultiplied;
		graphics.DepthStencilState = DepthStencilState.Default;
		graphics.RasterizerState   = RasterizerState.CullNone;
		graphics.SamplerStates[0]  = SamplerState.LinearClamp;

		effect            = new BasicEffect(graphics);
		effect.Alpha      = 1.0f;
		effect.View       = Microsoft.Xna.Framework.Matrix.Identity;
		effect.World      = Microsoft.Xna.Framework.Matrix.Identity;
		effect.Projection = Microsoft.Xna.Framework.Matrix.CreateOrthographicOffCenter(mapclip[0], mapclip[1], mapclip[2], mapclip[3], -100, 100);

		effect.LightingEnabled    = false;
		effect.VertexColorEnabled = true;
		
		SceneBuffer = new RenderTarget2D(graphics, 2 * scenedest.Width, 2 * scenedest.Height,
		                                 false, SurfaceFormat.Color, DepthFormat.Depth16,
		                                 0, RenderTargetUsage.DiscardContents);

		SideBuffer = new RenderTarget2D(graphics, sidedest.Width, sidedest.Height,
		                                false, SurfaceFormat.Color, DepthFormat.Depth16,
		                                0, RenderTargetUsage.DiscardContents);

		if (!Realtime) {
			TargetElapsedTime = FrameElapsed;
		}
		else {
			IsFixedTimeStep = false;
		}

		
		prevkeyboard = Keyboard.GetState();

		base.Initialize();
	}
	
	/// <summary>
	/// LoadContent will be called once per game and is the place to load
	/// all of your content.
	/// </summary>
	protected override void LoadContent()
	{
		flip          = new SpriteBatch(graphics);
		Explorer.Flip = this.flip;
	}

	/// <summary>
	/// Allows the game to run logic such as updating the world,
	/// checking for collisions, gathering input, and playing audio.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected override void Update(GameTime time)
	{
		simtime = Realtime ? time : new GameTime(simtime.TotalGameTime.Add(FrameElapsed), FrameElapsed);

		KeyboardState keyboard = Keyboard.GetState();

		double ds     = 0;
		double dyaw   = 0;
		double dpitch = 0;
		double droll  = 0;
		double dcam   = 0;
		
		bool fast = keyboard.IsKeyDown(Keys.LeftShift);
		bool slow = keyboard.IsKeyDown(Keys.LeftControl);

		bool forceslam         = false;
		bool forcemapping      = false;
		bool forcereset        = keyboard.IsKeyDown(Keys.R);
		bool forcehistoryreset = keyboard.IsKeyDown(Keys.T);

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
		
		if (keyboard.IsKeyDown(Keys.M) && !prevkeyboard.IsKeyDown(Keys.M)) {
			forceslam    = Navigator.OnlyMapping;
			forcemapping = !Navigator.OnlyMapping;
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
		}

		if (forcehistoryreset) {
			Navigator.ResetHistory();
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

		prevkeyboard = keyboard;

		base.Update(time);
	}

	/// <summary>
	/// This is called when the game should draw itself.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected override void Draw(GameTime time)
	{
		// scene panel
		graphics.SetRenderTarget(SceneBuffer);
		graphics.Clear(Color.DarkSeaGreen);

		foreach (EffectPass pass in effect.CurrentTechnique.Passes) {
			pass.Apply();
			
			Explorer .Render(camera);
			Navigator.Render(camera);
		}

		// sidebar
		graphics.SetRenderTarget(SideBuffer);
		graphics.Clear(Color.Black);

		flip.Begin(SpriteSortMode.Immediate, BlendState.NonPremultiplied, SamplerState.LinearClamp,
		           DepthStencilState.Default, RasterizerState.CullNone);

		Explorer.RenderSide();

		flip.End();

		// to window
		graphics.SetRenderTarget(null);
		flip.Begin(SpriteSortMode.Immediate, BlendState.NonPremultiplied, SamplerState.LinearClamp,
		           DepthStencilState.Default, RasterizerState.CullNone);

		flip.Draw(SceneBuffer, scenedest, SceneBuffer.Bounds, Color.White);
		flip.Draw(SideBuffer,  sidedest,  SideBuffer .Bounds, Color.White);
		
		flip.End();

		Console.WriteLine((1.0/time.ElapsedGameTime.TotalSeconds).ToString("F2"));

		base.Draw(time);
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

	/// <summary>
	/// Dispose any resources.
	/// </summary>
	/// <param name="disposing">Mask. If false, nothing is done.</param>
	protected override void Dispose(bool disposing)
	{
		if (disposing) {
			if (Explorer != null) {
				Explorer.Dispose();
			}

			base.Dispose(disposing);
		}
	}
}
}
