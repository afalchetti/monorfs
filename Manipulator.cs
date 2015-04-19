// Manipulator.cs
// General interactive controller for slam algorithms
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
/// Interactive controller for slam algorithms.
/// </summary>
public abstract class Manipulator : Game
{
	/// <summary>
	/// Main vehicle.
	/// </summary>
	public Vehicle Explorer { get; private set; }

	/// <summary>
	/// SLAM solver.
	/// </summary>
	public Navigator Navigator { get; private set; }

	/// <summary>
	/// Number of localization Montecarlo filter particles.
	/// </summary>
	public int ParticleCount { get; private set; }

	/// <summary>
	/// Frame period (frames per second inverse).
	/// </summary>
	public readonly TimeSpan FrameElapsed = new TimeSpan(10000000/30);

	/// <summary>
	/// Simulated time.
	/// </summary>
	private GameTime simtime = new GameTime();

	/// <summary>
	/// Manager responsible for rendering.
	/// </summary>
	private GraphicsDeviceManager graphicsManager;

	/// <summary>
	/// Render output.
	/// </summary>
	public GraphicsDevice Graphics { get; private set; }

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
	public SpriteBatch Flip { get; private set; }

	/// <summary>
	/// If true, the motion and measurement dynamics are not followed.
	/// </summary>
	public bool Paused { get; set; }

	/// <summary>
	/// If true, the calculations are bound by realtime constraints,
	/// i.e. if it takes too long, the vehicle will move proportionally.
	/// Otherwise, the simulated timestep is always the same, regardless of
	/// how long the operations take.
	/// </summary>
	public bool Realtime { get; private set; }

	/// <summary>
	/// Map clipping area.
	/// It is formatted as [left, right, bottom, top].
	/// </summary>
	public float[] MapClip { get; private set; }

	/// <summary>
	/// Previous frame keyboard state.
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
	/// Camera angle.
	/// </summary>
	private double camangle;

	/// <summary>
	/// Camera zoom factor.
	/// </summary>
	private double camzoom;

	/// <summary>
	/// Camera matrix.
	/// </summary>
	private double[][] camera;

	/// <summary>
	/// Construct a Manipulator from its components.
	/// </summary>
	public Manipulator(Vehicle explorer, Navigator navigator, int particlecount, bool realtime, float[] mapclip)
	{
		Explorer      = explorer;
		Navigator     = navigator;
		ParticleCount = particlecount;
		Realtime      = realtime;
		MapClip       = mapclip;

		graphicsManager = new GraphicsDeviceManager(this);

		Content.RootDirectory = "Content";
		IsMouseVisible        = true;

		graphicsManager.PreferredBackBufferWidth  = (int)(1000*1.0);
		graphicsManager.PreferredBackBufferHeight = (int)(450*1.0);
		graphicsManager.PreferMultiSampling       = true;
		graphicsManager.IsFullScreen              = false;
	}

	/// <summary>
	/// Get the biggest centered rectangle of the specified
	/// aspect ratio that fits inside a screen of specified size.
	/// </summary>
	/// <param name="width">Screen width.</param>
	/// <param name="height">Screen height.</param>
	/// <param name="aspect">Target aspect ratio.</param>
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
	/// Allow the manipulator to perform any initialization it needs to do before it starts running.
	/// </summary>
	protected override void Initialize()
	{
		Graphics                    = graphicsManager.GraphicsDevice;
		Explorer .Graphics          = Graphics;
		Navigator.Graphics          = Graphics;
		Graphics .BlendState        = BlendState.NonPremultiplied;
		Graphics .DepthStencilState = DepthStencilState.Default;
		Graphics .RasterizerState   = RasterizerState.CullNone;
		Graphics .SamplerStates[0]  = SamplerState.LinearClamp;

		effect            = new BasicEffect(Graphics);
		effect.Alpha      = 1.0f;
		effect.View       = Microsoft.Xna.Framework.Matrix.Identity;
		effect.World      = Microsoft.Xna.Framework.Matrix.Identity;
		effect.Projection = Microsoft.Xna.Framework.Matrix.CreateOrthographicOffCenter(MapClip[0], MapClip[1], MapClip[2], MapClip[3], -100, 100);

		effect.LightingEnabled    = false;
		effect.VertexColorEnabled = true;

		const double screencut = 0.7;

		scenedest = clipCenter((int)(graphicsManager.PreferredBackBufferWidth * screencut),
		                       graphicsManager.PreferredBackBufferHeight,
		                       (MapClip[1] - MapClip[0]) / (MapClip[3] - MapClip[2]));

		sidedest = clipCenter((int)(graphicsManager.PreferredBackBufferWidth * (1 - screencut)),
		                      graphicsManager.PreferredBackBufferHeight,
		                      (float) Explorer.SidebarWidth / Explorer.SidebarHeight);

		sidedest.X += (int)(graphicsManager.PreferredBackBufferWidth * screencut);
		
		SceneBuffer = new RenderTarget2D(Graphics, 2 * scenedest.Width, 2 * scenedest.Height,
		                                 false, SurfaceFormat.Color, DepthFormat.Depth16,
		                                 0, RenderTargetUsage.DiscardContents);

		SideBuffer = new RenderTarget2D(Graphics, sidedest.Width, sidedest.Height,
		                                false, SurfaceFormat.Color, DepthFormat.Depth16,
		                                0, RenderTargetUsage.DiscardContents);

		if (!Realtime) {
			TargetElapsedTime = FrameElapsed;
		}
		else {
			IsFixedTimeStep = false;
		}

		prevkeyboard = Keyboard.GetState();
		camangle     = 0;
		camzoom      = 1;
		camera       = Accord.Math.Matrix.Identity(3).ToArray();
		Paused       = false;
		
		base.Initialize();
	}
	
	/// <summary>
	/// LoadContent will be called once per run and is the place to load
	/// all content.
	/// </summary>
	protected override void LoadContent()
	{
		Flip          = new SpriteBatch(Graphics);
		Explorer.Flip = this.Flip;
	}

	/// <summary>
	/// Allows the manipulator to run logic such as updating the world,
	/// checking for collisions, gathering input, and playing audio.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected override void Update(GameTime time)
	{
		simtime = Realtime ? time : new GameTime(simtime.TotalGameTime.Add(FrameElapsed), FrameElapsed);

		KeyboardState keyboard = Keyboard.GetState();

		double dcamangle = 0;
		double dcamzoom  = 0;
		
		bool fast = keyboard.IsKeyDown(Keys.LeftShift);
		bool slow = keyboard.IsKeyDown(Keys.LeftControl);

		double multiplier = 1.0;
		multiplier *= (fast) ? 2.0 : 1.0;
		multiplier /= (slow) ? 4.0 : 1.0;
		
		if (keyboard.IsKeyDown(Keys.Up)) {
			dcamangle += 0.06 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.Down)) {
			dcamangle -= 0.06 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.Right)) {
			dcamzoom += 0.05 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.Left)) {
			dcamzoom -= 0.05 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.Escape) && !prevkeyboard.IsKeyDown(Keys.Escape)) {
			Paused = !Paused;
		}

		camangle += dcamangle;
		camzoom   = Math.Max(0.1, Math.Min(10.0, camzoom * (1 + dcamzoom)));
		camera    = camzoom.Multiply(MatrixExtensions.CreateRotationX(camangle));

		Update(simtime, keyboard, prevkeyboard, multiplier);

		prevkeyboard = keyboard;
		base.Update(time);
	}

	/// <summary>
	/// Additional update steps, hooked at the end of the general update method.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="keyboard">Current keyboard state information.</param>
	/// <param name="prevkeyboard">Old keyboard state information. Used to get newly pressed keys.</param>
	/// <param name="multiplier">Movement scale multiplier.</param>
	protected virtual void Update(GameTime time, KeyboardState keyboard, KeyboardState prevkeyboard, double multiplier) {}

	/// <summary>
	/// This is called when the manipulator should draw itself.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected override void Draw(GameTime time)
	{
		// scene panel
		Graphics.SetRenderTarget(SceneBuffer);
		Graphics.Clear(Color.DarkSeaGreen);

		foreach (EffectPass pass in effect.CurrentTechnique.Passes) {
			pass.Apply();
			
			Explorer .Render(camera);
			Navigator.Render(camera);
		}

		// sidebar
		Graphics.SetRenderTarget(SideBuffer);
		Graphics.Clear(Color.Black);

		Flip.Begin(SpriteSortMode.Immediate, BlendState.NonPremultiplied, SamplerState.LinearClamp,
		           DepthStencilState.Default, RasterizerState.CullNone);
		
		Explorer.RenderSide();

		Flip.End();

		// to window
		Graphics.SetRenderTarget(null);
		Flip.Begin(SpriteSortMode.Immediate, BlendState.NonPremultiplied, SamplerState.LinearClamp,
		           DepthStencilState.Default, RasterizerState.CullNone);

		Flip.Draw(SceneBuffer, scenedest, SceneBuffer.Bounds, Color.White);
		Flip.Draw(SideBuffer,  sidedest,  SideBuffer .Bounds, Color.White);
		
		Flip.End();

		Console.WriteLine((1.0/time.ElapsedGameTime.TotalSeconds).ToString("F2"));

		base.Draw(time);
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
