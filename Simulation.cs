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

namespace monorfs
{
/// <summary>
/// Vehicle navigation simulation through a landmark map.
/// </summary>
public class Simulation : Game
{
	/// <summary>
	/// Measure cycle period in miliseconds. Every this
	/// amount of time the SLAM solver is invoked.
	/// </summary>
	public const double MeasurePeriod = 100;

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
	private RenderTarget2D buffer;

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
	/// Last time the navigator update method was called in
	/// milliseconds since an arbitrary starting time.
	/// </summary>
	private double lastnavigationupdate = 0;

	/// <summary>
	/// Construct a simulation from a formatted desccription file.
	/// </summary>
	/// <param name="filename">Scene descriptor file.</param>
	public Simulation(string filename)
	{
		string descriptor = File.ReadAllText(filename);

		Dictionary<string, List<string>> dict = Util.ParseDictionary(descriptor);

		this.mapclip = Accord.Math.Matrix.ToSingle(ParseDoubleList(dict["world"][0]));

		if (mapclip.Length != 4) {
			throw new FormatException("The map clipping area must be specified by four elements: left, right, bottom, top");
		}

		double[] vehiclepose = ParseDoubleList(dict["vehicle"][0]);

		if (vehiclepose.Length != 3) {
			throw new FormatException("Vehicle description must have exactly three arguments: x, y and theta");
		}

		List<double[]> maploc      = new List<double[]>();
		List<string>   mapdescript = dict["landmarks"];

		for (int i = 0; i < mapdescript.Count; i++) {
			double[] landmark = ParseDoubleList(mapdescript[i]);

			if (landmark.Length != 2) {
				throw new FormatException("Map landmarks must be 2D");
			}

			maploc.Add(landmark);
		}

		this.Explorer            = new Vehicle(vehiclepose[0], vehiclepose[1], vehiclepose[2]);
		this.Landmarks           = new List<double[]>();
		this.MeasurementReadings = new List<double[]>();

		for (int i = 0; i < maploc.Count; i++) {
			this.Landmarks.Add(maploc[i]);
		}

		this.Navigator = new Navigator(Explorer);

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
		this.graphics.DepthStencilState = DepthStencilState.None;
		this.graphics.RasterizerState   = RasterizerState.CullNone;
		this.graphics.SamplerStates[0]  = SamplerState.LinearClamp;

		this.effect            = new BasicEffect(this.graphics);
		this.effect.Alpha      = 1.0f;
		this.effect.View       = Matrix.Identity;
		this.effect.World      = Matrix.Identity;
		this.effect.Projection = Matrix.CreateOrthographicOffCenter(mapclip[0], mapclip[1], mapclip[2], mapclip[3], -1, 2);

		this.effect.LightingEnabled    = false;
		this.effect.VertexColorEnabled = true;

		this.buffer = new RenderTarget2D(graphics, 2 * bufferdest.Width, 2 * bufferdest.Height,
		                                 false, SurfaceFormat.Color, DepthFormat.None,
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
		KeyboardState keyboard = Keyboard.GetState();

		double ds     = 0;
		double dtheta = 0;
		
		if (keyboard.IsKeyDown(Keys.Down)) {
			ds -= 0.01;
		}

		if (keyboard.IsKeyDown(Keys.Up)) {
			ds += 0.01;
		}

		if (keyboard.IsKeyDown(Keys.Left)) {
			dtheta += 0.02;
		}

		if (keyboard.IsKeyDown(Keys.Right)) {
			dtheta -= 0.02;
		}
		
		bool DoPredict = !keyboard.IsKeyDown(Keys.P);
		bool DoCorrect = !keyboard.IsKeyDown(Keys.C);
		bool DoPrune   = !keyboard.IsKeyDown(Keys.Q);

		Explorer.Update(time, ds, 0, dtheta);

		if (time.TotalGameTime.TotalMilliseconds - lastnavigationupdate > MeasurePeriod) {
			List<double[]> measurements = Explorer.Measure(Landmarks);
			Navigator.Update(measurements, DoPredict, DoCorrect, DoPrune);
			lastnavigationupdate = time.TotalGameTime.TotalMilliseconds;

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
		this.graphics.SetRenderTarget(buffer);
		this.graphics.Clear(Color.DarkSeaGreen);

		foreach (EffectPass pass in effect.CurrentTechnique.Passes) {
			pass.Apply();
			
			Explorer.RenderTrajectory();
			Explorer.RenderFOV();
			
			foreach (double[] landmark in Landmarks) {
				RenderLandmark(landmark);
			}

			foreach (double[] measure in MeasurementReadings) {
				RenderMeasure(measure);
			}

			Explorer .RenderBody();
			Navigator.Render();
		}

		graphics.SetRenderTarget(null);
		flip.Begin(SpriteSortMode.Immediate, BlendState.NonPremultiplied, SamplerState.LinearClamp,
			DepthStencilState.None, RasterizerState.CullNone);

		flip.Draw(buffer, bufferdest, Color.White);

		//Console.WriteLine(1.0/time.ElapsedGameTime.TotalSeconds);

		flip.End();

		base.Draw(time);
	}

	/// <summary>
	/// Simple point landmark rendering.
	/// </summary>
	/// <param name="landmark">Point landmark position.</param>
	private void RenderLandmark(double[] landmark)
	{
		const float halflen = 0.024f;
		
		Color innercolor =  Color.LightGray;
		Color outercolor =  Color.Black;
		
		VertexPositionColor[] invertices  = new VertexPositionColor[4];
		VertexPositionColor[] outvertices = new VertexPositionColor[4];

		invertices[0] = new VertexPositionColor(new Vector3((float) landmark[0] - halflen, (float) landmark[1] - halflen, 0), innercolor);
		invertices[1] = new VertexPositionColor(new Vector3((float) landmark[0] - halflen, (float) landmark[1] + halflen, 0), innercolor);
		invertices[2] = new VertexPositionColor(new Vector3((float) landmark[0] + halflen, (float) landmark[1] - halflen, 0), innercolor);
		invertices[3] = new VertexPositionColor(new Vector3((float) landmark[0] + halflen, (float) landmark[1] + halflen, 0), innercolor);

		outvertices[0] = invertices[0];
		outvertices[1] = invertices[1];
		outvertices[2] = invertices[3];
		outvertices[3] = invertices[2];

		graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		graphics.DrawUser2DPolygon(outvertices, 0.02f, outercolor, true);
	}

	/// <summary>
	/// Simple point measurement rendering.
	/// </summary>
	/// <param name="measurement">Point measurement position.</param>
	private void RenderMeasure(double[] measurement)
	{
		const float halflen = 0.024f;
		
		Color color =  Color.Crimson;
		
		VertexPositionColor[] vertices = new VertexPositionColor[2];

		vertices[0] = new VertexPositionColor(new Vector3((float) measurement[0] - halflen, (float) measurement[1] - halflen, 0), color);
		vertices[1] = new VertexPositionColor(new Vector3((float) measurement[0] + halflen, (float) measurement[1] + halflen, 0), color);
		
		graphics.DrawUser2DPolygon(vertices, 0.02f, color, true);

		vertices[0] = new VertexPositionColor(new Vector3((float) measurement[0] - halflen, (float) measurement[1] + halflen, 0), color);
		vertices[1] = new VertexPositionColor(new Vector3((float) measurement[0] + halflen, (float) measurement[1] - halflen, 0), color);

		graphics.DrawUser2DPolygon(vertices, 0.02f, color, true);
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
			throw new FormatException("the point descriptor '" + descriptor + "' is malformed");
		}

		return point;
	}
}
}
