// Manipulator.cs
// General interactive controller for slam algorithms
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

using Accord.Math;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

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
	/// Frame period (frames per second inverse).
	/// </summary>
	public readonly TimeSpan FrameElapsed;

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
	public readonly bool Realtime;

	/// <summary>
	/// Map clipping area.
	/// It is formatted as [left, right, bottom, top].
	/// </summary>
	public readonly float[] MapClip = Config.MapClip;

	/// <summary>
	/// Previous frame keyboard state.
	/// </summary>
	private KeyboardState prevkeyboard;

	/// <summary>
	/// Previous frame mouse state.
	/// </summary>
	private MouseState prevmouse;

	/// <summary>
	/// Focused landmark.
	/// </summary>
	private double[] mousefocus;

	/// <summary>
	/// Focused landmark rpojection on the screen.
	/// </summary>
	private double[] mousefocusproj;

	/// <summary>
	/// Smooth target position updater generator.
	/// </summary>
	private IEnumerator<double[]> targetupdater;

	/// <summary>
	/// True if the target is not moving and in a stable position.
	/// </summary>
	private bool targetreached;
	
	/// <summary>
	/// Scene double buffer flipping destination rectangle.
	/// </summary>
	protected Rectangle SceneDest;

	/// <summary>
	/// Sidebar double buffer flipping destination rectangle.
	/// </summary>
	protected Rectangle SideDest;

	/// <summary>
	/// Screen ratio between scene and side targets, in [0, 1].
	/// </summary>
	protected double ScreenCut { get; set; }

	/// <summary>
	/// Camera ground angle.
	/// </summary>
	private double camtheta;

	/// <summary>
	/// Camera elevation angle.
	/// </summary>
	private double camphi;

	/// <summary>
	/// Camera zoom factor (distance to target).
	/// </summary>
	private double camzoom;

	/// <summary>
	/// Camera target.
	/// </summary>
	private double[] camtarget;

	/// <summary>
	/// Camera matrix.
	/// </summary>
	private double[][] camera;

	/// <summary>
	/// Number of frames in smooth transitions.
	/// </summary>
	public const int TransitionFrames = 30;

	/// <summary>
	/// Text message displayed next to the rendered simulation. 
	/// </summary>
	public string Message { get; protected set; }

	/// <summary>
	/// Render font.
	/// </summary>
	private SpriteFont font;

	/// <summary>
	/// Text message position.
	/// </summary>
	private Vector2 messagepos;

	/// <summary>
	/// Construct a Manipulator from its components.
	/// </summary>
	/// <param name="title">Window title.</param>
	/// <param name="explorer">Explorer vehicle.</param>
	/// <param name="navigator">SLAM solver.</param>
	/// <param name="realtime">Realtime data processing.</param>
	/// <param name="fps">Frame per seconds.</param>
	protected Manipulator(string title, Vehicle explorer, Navigator navigator, bool realtime, double fps = 30)
	{
		Window.Title = title;

		Explorer      = explorer;
		Navigator     = navigator;
		Realtime      = realtime;

		graphicsManager = new GraphicsDeviceManager(this);

		Content.RootDirectory = "Content";
		IsMouseVisible        = true;

		graphicsManager.PreferredBackBufferWidth  = (int)(1000*1.0);
		graphicsManager.PreferredBackBufferHeight = (int)(450*1.0);
		graphicsManager.PreferMultiSampling       = true;
		graphicsManager.IsFullScreen              = false;

		ScreenCut = (Explorer.HasSidebar) ? 0.7 : 1.0;

		FrameElapsed = new TimeSpan((long) (10000000/fps));
		Message      = "";
		messagepos   = new Vector2(350, graphicsManager.PreferredBackBufferHeight - 30);

		IsFixedTimeStep   = false;
		TargetElapsedTime = FrameElapsed;
	}

	/// <summary>
	/// Get the biggest centered rectangle of the specified
	/// aspect ratio that fits inside a screen of specified size.
	/// </summary>
	/// <param name="width">Screen width.</param>
	/// <param name="height">Screen height.</param>
	/// <param name="aspect">Target aspect ratio.</param>
	/// <returns> Biggest centered rectangle.</returns>
	private static Rectangle clipCenter(int width, int height, float aspect)
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

		SceneDest = clipCenter((int) (graphicsManager.PreferredBackBufferWidth * ScreenCut),
		                       graphicsManager.PreferredBackBufferHeight - 40,
		                       (MapClip[1] - MapClip[0]) / (MapClip[3] - MapClip[2]));

		SideDest = clipCenter((int) (graphicsManager.PreferredBackBufferWidth * (1 - ScreenCut)),
		                      graphicsManager.PreferredBackBufferHeight - 40,
		                      (float) Explorer.SidebarWidth / Explorer.SidebarHeight);

		SideDest.X += (int) (graphicsManager.PreferredBackBufferWidth * ScreenCut);
		
		SceneBuffer = new RenderTarget2D(Graphics, 2 * SceneDest.Width, 2 * SceneDest.Height,
		                                 false, SurfaceFormat.Color, DepthFormat.Depth16,
		                                 0, RenderTargetUsage.DiscardContents);

		SideBuffer = new RenderTarget2D(Graphics, SideDest.Width, SideDest.Height,
		                                false, SurfaceFormat.Color, DepthFormat.Depth16,
		                                0, RenderTargetUsage.DiscardContents);

		prevkeyboard = Keyboard.GetState();
		prevmouse    = Mouse.GetState();

		mousefocus     = null;
		mousefocusproj = null;
		targetreached  = true;
		targetupdater  = System.Linq.Enumerable.Empty<double[]>().GetEnumerator();
		targetupdater.MoveNext();

		camtheta     = 0;
		camphi       = 0;
		camzoom      = 1;
		camtarget    = new double[3] {0, 0, 0};
		camera       = Accord.Math.Matrix.Identity(4).ToArray();
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
		font          = Content.Load<SpriteFont>("pescadero");
	}

	/// <summary>
	/// Allow the manipulator to run logic such as updating the world
	/// and gathering input.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected override void Update(GameTime time)
	{
		simtime = Realtime ? time : new GameTime(simtime.TotalGameTime.Add(FrameElapsed), FrameElapsed);

		KeyboardState keyboard = Keyboard.GetState();
		MouseState    mouse    = Mouse.GetState();

		double dcamtheta = 0;
		double dcamphi   = 0;
		double dcamzoom  = 0;
		
		bool fast = keyboard.IsKeyDown(Keys.LeftShift);
		bool slow = keyboard.IsKeyDown(Keys.LeftControl);
		bool alt  = keyboard.IsKeyDown(Keys.LeftAlt);

		double multiplier = 1.0;
		multiplier *= (fast) ? 2.0 : 1.0;
		multiplier /= (slow) ? 4.0 : 1.0;
		
		if (keyboard.IsKeyDown(Keys.Delete)) {
			Exit();
			return;
		}

		if (keyboard.IsKeyDown(Keys.Up) && !alt) {
			dcamphi -= 0.06 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.Down) && !alt) {
			dcamphi += 0.06 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.Right) && !alt) {
			dcamtheta += 0.06 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.Left) && !alt) {
			dcamtheta -= 0.06 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.Up) && alt) {
			dcamzoom -= 0.05 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.Down) && alt) {
			dcamzoom += 0.05 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.Escape) && !prevkeyboard.IsKeyDown(Keys.Escape)) {
			Paused = !Paused;
		}

		if (!targetreached) {
			camtarget     = targetupdater.Current;
			targetreached = !targetupdater.MoveNext();
		}

		camtheta += dcamtheta;
		camphi    = Math.Max(-Math.PI/2 + 0.01, Math.Min(Math.PI/2 - 0.01, camphi + dcamphi));
		camzoom   = Math.Max(0.01, Math.Min(100.0, camzoom * (1 + dcamzoom)));
		camera    = MatrixExtensions.AngleDistanceCamera(camtarget, camtheta, camphi, camzoom);

		Update(simtime, keyboard, prevkeyboard, multiplier);

		mousefocus = getMouseFocus(mouse, out mousefocusproj);

		if (mouse.LeftButton == ButtonState.Pressed && prevmouse.LeftButton == ButtonState.Released) {
			targetupdater = smoothTranslater(new double[3] {camtarget[0], camtarget[1], camtarget[2]}, mousefocus);
			targetreached = !targetupdater.MoveNext();
		}

		prevkeyboard = keyboard;
		prevmouse    = mouse;
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
			RenderHUD();
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

		Flip.Draw(SceneBuffer, SceneDest, SceneBuffer.Bounds, Color.White);
		Flip.Draw(SideBuffer,  SideDest,  SideBuffer .Bounds, Color.White);
		Flip.DrawString(font, Message, messagepos, Color.White);
		
		Flip.End();

		base.Draw(time);
	}

	/// <summary>
	/// Render any heads-up display.
	/// </summary>
	public void RenderHUD()
	{
		if (mousefocusproj != null) {
			const float halflen = 0.085f;

			Color      color    = Color.OrangeRed;
			double[][] vertices = new double[8][];

			vertices[0] = new double[] {mousefocusproj[0] -     halflen, mousefocusproj[1] -     halflen, mousefocusproj[2]};
			vertices[1] = new double[] {mousefocusproj[0] - 1.3*halflen, mousefocusproj[1] +           0, mousefocusproj[2]};
			vertices[2] = new double[] {mousefocusproj[0] -     halflen, mousefocusproj[1] +     halflen, mousefocusproj[2]};
			vertices[3] = new double[] {mousefocusproj[0] +           0, mousefocusproj[1] + 1.3*halflen, mousefocusproj[2]};
			vertices[4] = new double[] {mousefocusproj[0] +     halflen, mousefocusproj[1] +     halflen, mousefocusproj[2]};
			vertices[5] = new double[] {mousefocusproj[0] + 1.3*halflen, mousefocusproj[1] +           0, mousefocusproj[2]};
			vertices[6] = new double[] {mousefocusproj[0] +     halflen, mousefocusproj[1] -     halflen, mousefocusproj[2]};
			vertices[7] = new double[] {mousefocusproj[0] +           0, mousefocusproj[1] - 1.3*halflen, mousefocusproj[2]};

			Graphics.DrawUser2DPolygon(vertices, 0.03f, color, true);
		}
	}

	/// <summary>
	/// Get the closest landmark to the mouse.
	/// If no landmark is near enough, returns null.
	/// </summary>
	/// <param name="mouse">Mouse state.</param>
	/// <param name="projected">Viewport projection of the closest landmark.
	/// Null if no landmark is close enough</param>
	/// <returns>Focused landmark.</returns>
	private double[] getMouseFocus(MouseState mouse, out double[] projected)
	{
		Point mousepos = mouse.Position;
		projected = null;

		double[] closest    = null;
		double   closedist2 = double.MaxValue;

		foreach (double[] landmark in Explorer.Landmarks) {
			double[] proj = camera.TransformH(landmark);

			Vector3  screen    = Vector3.Transform(new Vector3((float) proj[0], (float) proj[1], (float) proj[2]), effect.Projection);
			Point    screenxy  = SceneDest.Center + new Point((int)(screen.X * SceneDest.Width / 2.0), -(int)(screen.Y * SceneDest.Height / 2.0));
			double[] diff      = new double[2] {mousepos.X - screenxy.X, mousepos.Y - screenxy.Y};
			double   distance2 = diff.SquareEuclidean();

			if (distance2 < closedist2) {
				closest    = landmark;
				projected  = proj;
				closedist2 = distance2;
			}
		}



		if (closedist2 > 10 * 10) {
			projected = null;
			return null;
		}
		
		return closest;
	}

	/// <summary>
	/// Create a generator of smooth transition between target position.
	/// </summary>
	/// <param name="initial">Initial position.</param>
	/// <param name="final">Final position.</param>
	/// <returns>Transition generator.</returns>
	private IEnumerator<double[]> smoothTranslater(double[] initial, double[] final)
	{
		if (final == null) {
			yield break;
		}

		for (int x = 0; x <= TransitionFrames; x++) {
			double alpha = Util.SmoothTransition((double) x / TransitionFrames);
			yield return (1-alpha).Multiply(initial).Add(alpha.Multiply(final));
		}
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

			if (Navigator != null) {
				Navigator.Dispose();
			}

			base.Dispose(disposing);
		}
	}
}
}
