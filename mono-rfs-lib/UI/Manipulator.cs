﻿// Manipulator.cs
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
using System.IO;

using Accord.Math;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

using TimedMessage = System.Collections.Generic.List<System.Tuple<double, string>>;

namespace monorfs
{
/// <summary>
/// Interactive controller for slam algorithms.
/// </summary>
public abstract class Manipulator<MeasurerT, PoseT, MeasurementT> : Game
	where PoseT        : IPose<PoseT>, new()
	where MeasurementT : IMeasurement<MeasurementT>, new()
	where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
{
	/// <summary>
	/// Saved odometry vector length.
	/// </summary>
	protected static int OdoSize;

	/// <summary>
	/// Saved state vector length.
	/// </summary>
	protected static int StateSize;

	/// <summary>
	/// Saved measurement vector length.
	/// </summary>
	protected static int MeasureSize;

	/// <summary>
	/// Main vehicle.
	/// </summary>
	public Vehicle<MeasurerT, PoseT, MeasurementT> Explorer { get; private set; }

	/// <summary>
	/// SLAM solver.
	/// </summary>
	public Navigator<MeasurerT, PoseT, MeasurementT> Navigator { get; private set; }

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
	/// Rendering shader for Kinect HUD.
	/// </summary>
	private BasicEffect hudeffect;
	
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
	/// If true, stop anything in the next frame and try to exit gracefully.
	/// </summary>
	public bool Abort {
		get { return abort; }
		set { abort = value; }
	}

	/// <summary>
	/// Internal abort flag.
	/// </summary>
	public volatile bool abort;

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
	public readonly float[] MapClip = Config.MapClip.ToSingle();

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
	/// Output directory/filename for saving screenshots;
	/// a number and an extension will be appended for each screenshot taken,
	/// so "dir/file" becomes "dir/file1.png", "dir/file2.png" and so on.
	/// It defaults to "MY_PICTURES_FOLDER/monorfs"
	/// </summary>
	public string ScreenshotPrefix { get; set; }

	/// <summary>
	/// Number of screenshots taken so far.
	/// </summary>
	private int screenshotcount = 0;

	/// <summary>
	/// Text message displayed next to the rendered simulation. 
	/// </summary>
	public string Message { get; protected set; }

	/// <summary>
	/// Render font.
	/// </summary>
	private SpriteFont font;

	/// <summary>
	/// Render font for axes.
	/// </summary>
	private SpriteFont fontaxis;

	/// <summary>
	/// Text message position.
	/// </summary>
	private Vector2 messagepos;

	/// <summary>
	/// Reference sidebar drawing width.
	/// </summary>
	public int SidebarWidth { get; protected set; }

	/// <summary>
	/// Reference sidebar drawing height.
	/// </summary>
	public int SidebarHeight { get; protected set; }

	/// <summary>
	/// Tags in the timeline.
	/// </summary>
	public TimedMessage Tags { get; protected set; }

	/// <summary>
	/// Tag message displayed next to the rendered simulation. 
	/// </summary>
	public string TagMessage { get; protected set; }

	/// <summary>
	/// Tag color (should fade with time).
	/// </summary>
	public Color TagColor { get; protected set; }

	/// <summary>
	/// Tag message position.
	/// </summary>
	public Vector2 TagMessagePos { get; protected set; }

	/// <summary>
	/// Get a string representation of the tags in the timeline.
	/// </summary>
	public string SerializedTags
	{
		get
		{
			return string.Join("\n", Tags.ConvertAll(
				s => {double time    = s.Item1;
			          string message = s.Item2;
				      return time.ToString("g6") + " " + message; }
			));
		}
	}

	/// <summary>
	/// Calculate global constants.
	/// </summary>
	static Manipulator()
	{
		OdoSize     = new PoseT().OdometrySize;
		StateSize   = new PoseT().StateSize;
		MeasureSize = new MeasurementT().Size;
	}

	/// <summary>
	/// Construct a Manipulator from its components.
	/// </summary>
	/// <param name="title">Window title.</param>
	/// <param name="explorer">Explorer vehicle.</param>
	/// <param name="navigator">SLAM solver.</param>
	/// <param name="realtime">Realtime data processing.</param>
	/// <param name="fps">Frame per seconds.</param>
	protected Manipulator(string title,
	                      Vehicle<MeasurerT, PoseT, MeasurementT> explorer,
	                      Navigator<MeasurerT, PoseT, MeasurementT> navigator,
	                      bool realtime, double fps = 30)
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

		ScreenCut     = (Explorer.HasSidebar) ? 0.7 : 1.0;
		SidebarWidth  = (Explorer.HasSidebar) ? 640 : 1;
		SidebarHeight = (Explorer.HasSidebar) ? 2 * 480 : 1;

		ScreenshotPrefix = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyPictures), "monorfs");

		FrameElapsed = new TimeSpan((long) (10000000/fps));
		Message      = "";
		messagepos   = new Vector2(350, graphicsManager.PreferredBackBufferHeight - 30);

		Tags          = new TimedMessage();
		TagMessage    = "";
		TagMessagePos = new Vector2(620, graphicsManager.PreferredBackBufferHeight - 30);
		TagColor      = Color.Black;

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
		effect.Alpha      = 1;
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
		                      (float) SidebarWidth / SidebarHeight);

		SideDest.X += (int) (graphicsManager.PreferredBackBufferWidth * ScreenCut);
		
		SceneBuffer = new RenderTarget2D(Graphics, 6 * SceneDest.Width, 6 * SceneDest.Height,
		                                 false, SurfaceFormat.Color, DepthFormat.Depth16,
		                                 0, RenderTargetUsage.DiscardContents);

		SideBuffer = new RenderTarget2D(Graphics, SideDest.Width, SideDest.Height,
		                                false, SurfaceFormat.Color, DepthFormat.Depth16,
		                                0, RenderTargetUsage.DiscardContents);


		hudeffect            = new BasicEffect(Graphics);
		hudeffect.Alpha      = 1.0f;
		hudeffect.View       = Microsoft.Xna.Framework.Matrix.Identity;
		hudeffect.World      = Microsoft.Xna.Framework.Matrix.Identity;
		hudeffect.Projection = Microsoft.Xna.Framework.Matrix.CreateOrthographicOffCenter(new Rectangle(0, 0, SideDest.Width, SideDest.Height), -100, 100);

		hudeffect.LightingEnabled    = false;
		hudeffect.VertexColorEnabled = true;

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
		font          = Content.Load<SpriteFont>("OpenSans");
		fontaxis      = Content.Load<SpriteFont>("OpenSansLarge");
	}

	/// <summary>
	/// Allow the manipulator to run logic such as updating the world
	/// and gathering input.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected override void Update(GameTime time)
	{
		if (Abort) {
			Exit();
			return;
		}

		if (Realtime) {
			simtime = time;
		}

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
		if (keyboard.IsKeyDown(Keys.Z)) {
			Screenshot();
		}

		if (keyboard.IsKeyDown(Keys.Up) && !alt) {
			dcamphi += 0.06 * multiplier;
		}

		if (keyboard.IsKeyDown(Keys.Down) && !alt) {
			dcamphi -= 0.06 * multiplier;
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

		SetCamera(camtheta + dcamtheta,
		          Math.Max(-Math.PI/2 + 0.01, Math.Min(Math.PI/2 - 0.01, camphi + dcamphi)),
		          Math.Max(0.01, Math.Min(100.0, camzoom * (1 + dcamzoom))));

		Update(new GameTime(simtime.TotalGameTime, simtime.ElapsedGameTime), keyboard, prevkeyboard, multiplier);

		mousefocus = getMouseFocus(mouse, out mousefocusproj);

		if (mouse.LeftButton == ButtonState.Pressed && prevmouse.LeftButton == ButtonState.Released) {
			targetupdater = smoothTranslater(new double[3] {camtarget[0], camtarget[1], camtarget[2]}, mousefocus);
			targetreached = !targetupdater.MoveNext();
		}

		prevkeyboard = keyboard;
		prevmouse    = mouse;
		base.Update(time);

		if (!Realtime) {
			simtime = new GameTime(simtime.TotalGameTime.Add(FrameElapsed), FrameElapsed);
		}
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
		// visualization
		Graphics.SetRenderTarget(SceneBuffer);
		Graphics.Clear(Color.White);

		Graphics .DepthStencilState = DepthStencilState.Default;

		foreach (EffectPass pass in effect.CurrentTechnique.Passes) {
			pass.Apply();
			
			Navigator.Render(camera);
		}
		
		Graphics .DepthStencilState = DepthStencilState.None;

		foreach (EffectPass pass in effect.CurrentTechnique.Passes) {
			pass.Apply();

			RenderHUD(camera);
		}

		Flip.Begin(SpriteSortMode.Immediate, BlendState.NonPremultiplied, SamplerState.LinearClamp,
		           DepthStencilState.Default, RasterizerState.CullNone);

		double limit = Config.AxisLimit;
		renderAxisAnnotations(new double[3] {-limit, 0, 0}, new double[3] {+limit, 0, 0}, 2, "x [m]", false);
		renderAxisAnnotations(new double[3] {0, -limit, 0}, new double[3] {0, +limit, 0}, 2, "y [m]", true);
		renderAxisAnnotations(new double[3] {0, 0, -limit}, new double[3] {0, 0, +limit}, 2, "z [m]", false);

		Flip.End();

		// sidebar
		Graphics.SetRenderTarget(SideBuffer);
		Graphics.Clear(Color.Black);

		Flip.Begin(SpriteSortMode.FrontToBack, BlendState.NonPremultiplied, SamplerState.LinearClamp,
		           DepthStencilState.Default, RasterizerState.CullNone);
		
		Explorer.RenderSide();

		Flip.End();

		// HUD
		foreach (EffectPass pass in hudeffect.CurrentTechnique.Passes) {
			pass.Apply();
			Explorer.RenderSideHUD();
		}

		// text and axes
		Graphics.SetRenderTarget(null);
		Flip.Begin(SpriteSortMode.Immediate, BlendState.NonPremultiplied, SamplerState.LinearClamp,
		           DepthStencilState.Default, RasterizerState.CullNone);
		
		Flip.Draw(SceneBuffer, SceneDest, SceneBuffer.Bounds, Color.White);
		Flip.Draw(SideBuffer,  SideDest,  SideBuffer .Bounds, Color.White);
		Flip.DrawString(font, Message,    messagepos,    Color.White);
		Flip.DrawString(font, TagMessage, TagMessagePos, TagColor);
		
		//double limit = Config.AxisLimit;
		//renderAxisAnnotations(new double[3] {-limit, 0, 0}, new double[3] {+limit, 0, 0}, 5);
		//renderAxisAnnotations(new double[3] {0, -limit, 0}, new double[3] {0, +limit, 0}, 5);
		//renderAxisAnnotations(new double[3] {0, 0, -limit}, new double[3] {0, 0, +limit}, 5);
		
		Flip.End();

		base.Draw(time);
	}

	/// <summary>
	/// Take a screenshot of the currently shown scene.
	/// </summary>
	public void Screenshot()
	{
		using (var file = new FileStream(ScreenshotPrefix + (screenshotcount + 1) + ".png", FileMode.Create)) {
			Util.SaveAsPng(SceneBuffer, file);
			screenshotcount++;
		}
	}

	/// <summary>
	/// Change the camera settings.
	/// </summary>
	/// <param name="theta">Ground angle.</param>
	/// <param name="phi">Elevation angle.</param>
	/// <param name="zoom">Zoom factor.</param>
	public void SetCamera(double theta, double phi, double zoom)
	{
		camtheta = theta;
		camphi   = phi;
		camzoom  = zoom;
		camera   = MatrixExtensions.AngleDistanceCamera(camtarget, theta, phi, zoom);
	}

	/// <summary>
	/// Render any heads-up display.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderHUD(double[][] camera)
	{
		double[][] vertices;
		Color      color;

		if (mousefocusproj != null) {
			const float halflen = 0.085f;

			color    = Color.OrangeRed;
			vertices = new double[8][];

			vertices[0] = new double[3] {mousefocusproj[0] -     halflen, mousefocusproj[1] -     halflen, mousefocusproj[2]};
			vertices[1] = new double[3] {mousefocusproj[0] - 1.3*halflen, mousefocusproj[1] +           0, mousefocusproj[2]};
			vertices[2] = new double[3] {mousefocusproj[0] -     halflen, mousefocusproj[1] +     halflen, mousefocusproj[2]};
			vertices[3] = new double[3] {mousefocusproj[0] +           0, mousefocusproj[1] + 1.3*halflen, mousefocusproj[2]};
			vertices[4] = new double[3] {mousefocusproj[0] +     halflen, mousefocusproj[1] +     halflen, mousefocusproj[2]};
			vertices[5] = new double[3] {mousefocusproj[0] + 1.3*halflen, mousefocusproj[1] +           0, mousefocusproj[2]};
			vertices[6] = new double[3] {mousefocusproj[0] +     halflen, mousefocusproj[1] -     halflen, mousefocusproj[2]};
			vertices[7] = new double[3] {mousefocusproj[0] +           0, mousefocusproj[1] - 1.3*halflen, mousefocusproj[2]};

			Graphics.DrawUser2DPolygon(vertices, 0.03f, color, true);
		}

		// axes
		double limit = Config.AxisLimit;
		renderAxis(new double[3] {-limit, 0, 0}, new double[3] {+limit, 0, 0}, new double[3] {0, 0.1, 0}, 1);
		renderAxis(new double[3] {0, -limit, 0}, new double[3] {0, +limit, 0}, new double[3] {0.1, 0, 0}, 1);
		renderAxis(new double[3] {0, 0, -limit}, new double[3] {0, 0, +limit}, new double[3] {0.1, 0, 0}, 1);
	}

	/// <summary>
	/// Render an ticked and annotated axis.
	/// </summary>
	/// <param name="from">From here.</param>
	/// <param name="to">To here.</param>
	/// <param name="tick">Relative vector for the tick lines.</param>
	/// <param name="tickres">Tick resolution.</param>
	private void renderAxis(double[] from, double[] to, double[] tick, double tickres)
	{
		double[][] vertices = new double[2][];

		vertices[0] = camera.TransformH(from);
		vertices[1] = camera.TransformH(to);
		Graphics.DrawUser2DPolygon(vertices, 0.06f, Color.Black);

		double[] delta    = to.Subtract(from);
		double[] tickunit = tickres.Multiply(delta.Normalize());
		double   length   = delta.Euclidean();
		double   limit    = length / 2;  // [-limit, limit]
		int      nticks   = (int) (limit / tickres);

		double[] zero    = to.Add(from).Divide(2);
		double[] postick = zero.Add(tickunit);
		double[] negtick = zero.Subtract(tickunit);

		for (int i = 0; i < nticks; i++) {
			vertices[0] = camera.TransformH(postick);
			vertices[1] = camera.TransformH(postick.Add(tick));
			Graphics.DrawUser2DPolygon(vertices, 0.06f, Color.Black);

			vertices[0] = camera.TransformH(negtick);
			vertices[1] = camera.TransformH(negtick.Add(tick));
			Graphics.DrawUser2DPolygon(vertices, 0.06f, Color.Black);

			postick = postick.Add(tickunit);
			negtick = negtick.Subtract(tickunit);
		}
	}

	/// <summary>
	/// Render an ticked and annotated axis.
	/// </summary>
	/// <param name="from">From here.</param>
	/// <param name="to">To here.</param>
	/// <param name="tickres">Tick resolution; how often to annotate the axis.</param>
	/// <param name="name">Axis name, written on top the ticks.</param>
	/// <param name="vertical">This is a vertical axis, so move the anootations to the left.</param>
	private void renderAxisAnnotations(double[] from, double[] to, double tickres, string name, bool vertical)
	{
		double[] delta = to.Subtract(from);

		// if too small, don't annotate anything (otherwise it will just be a mess)
		double[] proj = camera.TransformH(delta).Submatrix(0, 1);
		if (proj.SquareEuclidean() < 1 * 1) {
			return;
		}

		double[] tickunit = tickres.Multiply(delta.Normalize());
		double   length   = delta.Euclidean();
		double   limit    = length / 2;  // [-limit, limit]
		int      nticks   = (int) (limit / tickres);

		double[] zero    = to.Add(from).Divide(2);
		double[] postick = zero.Add(tickunit);
		double[] negtick = zero.Subtract(tickunit);

		float a = SceneBuffer.Width / (MapClip[1] - MapClip[0]);
		float x = SceneBuffer.Width / 2;
		float y = SceneBuffer.Height / 2;

		float scale = 1.5f;
		float rotangle = (float)(Util.NormalizeAngle(-Math.Atan2(proj[1], proj[0])));

		bool invertoffset = false;

		if (Math.Abs(rotangle) > Math.PI/2 && !vertical) {
			rotangle    += (float) Math.PI;
			invertoffset = true;
		}

		double[] nproj = proj.Normalize();

		if (nproj[0] < 0) {
			nproj = (-1).Multiply(nproj);
			invertoffset ^= true;
		}

		Vector2 t;

		if (vertical) {
			t = new Vector2((float)(x + 450 * scale * nproj[0] - 350 * nproj[1]),
			                (float)(y - 450 * scale * nproj[1] - 350 * nproj[0]));
		}
		else if (!invertoffset) {
			t = new Vector2((float)(x + ((name == "x [m]") ? 420 : 600) * scale * nproj[0] + 120 * nproj[1]),
			                (float)(y - ((name == "x [m]") ? 420 : 600) * scale * nproj[1] + 120 * nproj[0]));
		}
		else {
			t = new Vector2((float)(x + 580 * scale * nproj[0] - 120 * nproj[1]),
			                (float)(y - 580 * scale * nproj[1] - 120 * nproj[0]));
		}

		Flip.DrawString(fontaxis, name, t, Color.Black, rotangle,
		                new Vector2(0, 0), scale, SpriteEffects.None, 0);

		for (int i = 1; i <= nticks; i++) {
			double[] ppos = camera.TransformH(postick);
			double[] npos = camera.TransformH(negtick);

			Vector2 p = new Vector2(a * (float) ppos[0] + x, a * (float) -ppos[1] + y);
			Vector2 n = new Vector2(a * (float) npos[0] + x, a * (float) -npos[1] + y);

			string ptext  = (tickres * i).ToString("F0");
			Vector2 psize = fontaxis.MeasureString(ptext) * scale;

			string ntext  = (-tickres * i).ToString("F0");
			Vector2 nsize = fontaxis.MeasureString(ntext) * scale;

			p.X = p.X - psize.X / 2;
			n.X = n.X - nsize.X / 2;

			if (vertical) {
				p.X = p.X - psize.X / 2 - 30 * scale;
				n.X = n.X - nsize.X / 2 - 30 * scale;
			}

			if (SceneBuffer.Bounds.Contains(new Rectangle((int) p.X, (int) p.Y, (int) (psize.X), (int) (psize.Y)))) {
				Flip.DrawString(fontaxis, ptext, p, Color.Black, 0, new Vector2(0, 0), scale, SpriteEffects.None, 0);
			}

			if (SceneBuffer.Bounds.Contains(new Rectangle((int) n.X, (int) n.Y, (int) (nsize.X), (int) (nsize.Y)))) {
				Flip.DrawString(fontaxis, ntext, n, Color.Black, 0, new Vector2(0, 0), scale, SpriteEffects.None, 0);
			}

			postick = postick.Add(tickunit);
			negtick = negtick.Subtract(tickunit);
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

		foreach (double[] landmark in mousetargets()) {
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
	/// Enumerate all candidate targets for mouse selection.
	/// </summary>
	private IEnumerable<double[]> mousetargets()
	{
		yield return Explorer.Pose.Location;

		foreach (double[] landmark in Explorer.Landmarks) {
			yield return landmark;
		}

		foreach (double[] measurement in Explorer.MappedMeasurements) {
			yield return measurement;
		}

		foreach (Gaussian component in Navigator.BestMapModel) {
			if (component.Weight > 0.8) {
				yield return component.Mean;
			}
		}
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
