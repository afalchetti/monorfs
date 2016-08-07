// KinectVehicle.cs
// Vehicle motion and measurement model for a Kinect sensor
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
using System.Runtime.InteropServices;

using AForge;
using AForge.Imaging;
using Accord.Math;
using Accord.Imaging;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

using OpenNIWrapper;

using SparseMatrix = monorfs.SparseMatrix<double>;
using SparseItem   = monorfs.SparseItem<double>;

namespace monorfs
{
/// <summary>
/// Kinect vehicle model.
/// Extracts measurements from an online Kinect sensor.
/// </summary>
public class KinectVehicle : Vehicle<KinectMeasurer, Pose3D, PixelRangeMeasurement>
{
	/// <summary>
	/// Sensor depth data stream.
	/// </summary>
	private VideoStream depth;

	/// <summary>
	/// Sensor color data stream.
	/// </summary>
	private VideoStream color;

	/// <summary>
	/// Sensor device.
	/// </summary>
	private Device device;

	/// <summary>
	/// Cached history depth maps.
	/// </summary>
	private Queue<float[][]> depthcache;

	/// <summary>
	/// Cached history color images.
	/// </summary>
	private Queue<Color[]> colorcache;

	/// <summary>
	/// Cached history extracted features.
	/// </summary>
	private Queue<List<SparseItem>> featurecache;

	// Cached constants

	/// <summary>
	/// For efficiency purposes, not the whole image is used.
	/// It is subsampled first using this delta factor.
	/// </summary>
	public static int Delta { get { return Config.KinectDelta; } }

	/// <summary>
	/// Device resolution on the X-axis, properly scaled.
	/// </summary>
	public float ResX { get; private set; }

	/// <summary>
	/// Device resolution on the Y-axis, properly scaled.
	/// </summary>
	public float ResY { get; private set; }

	/// <summary>
	/// Image border where no keypoint can be found (because of incomplete patches).
	/// </summary>
	public int Border { get; private set; }

	/// <summary>
	/// Last measurement's depth frame.
	/// </summary>
	public float[][] DepthFrame;

	/// <summary>
	/// Last measurement's color frame.
	/// </summary>
	public Color[] ColorFrame;

	/// <summary>
	/// X coordinate depth->world coefficient.
	/// </summary>
	private float xzalpha;

	/// <summary>
	/// Y coordinate depth->world coefficient.
	/// </summary>
	private float yzalpha;
	
	// MonoGame stuff

	/// <summary>
	/// Sensor depth image reading.
	/// </summary>
	private Texture2D depthsensed;

	/// <summary>
	/// Sensor depth image reading.
	/// </summary>
	private Texture2D colorsensed;

	/// <summary>
	/// Sensor depth image reading.
	/// </summary>
	private List<SparseItem> interest;

	/// <summary>
	/// Interest points from the previous frame.
	/// </summary>
	private List<IFeaturePoint> prevkeypoints;

	/// <summary>
	/// Interest points from the previous frame.
	/// </summary>
	private List<IFeaturePoint> prevprevkeypoints;

	/// <summary>
	/// Keypoint matches against last frame.
	/// </summary>
	private IntPoint[][] matches = new IntPoint[0][];

	/// <summary>
	/// True makes the system perform a heavy keypoint
	/// filtering step before delivering the measurements.
	/// </summary>
	private bool KeypointFilter { get { return Config.KeypointFilter; } }

	/// <summary>
	/// True if the sidebar should be rendered, even if its slows the process a bit.
	/// </summary>
	private bool ShowSidebar;

	/// <summary>
	/// Renderer.
	/// </summary>
	public override GraphicsDevice Graphics
	{
		get
		{
			return base.Graphics;
		}
		set
		{
			base.Graphics = value;
			depthsensed = new Texture2D(Graphics,
			                       (int) (depth.VideoMode.Resolution.Width  / Delta),
			                       (int) (depth.VideoMode.Resolution.Height / Delta),
			                       false,
			                       SurfaceFormat.Color);

			colorsensed = new Texture2D(Graphics,
			                       (int) (depth.VideoMode.Resolution.Width  / Delta),
			                       (int) (depth.VideoMode.Resolution.Height / Delta),
			                       false,
			                       SurfaceFormat.Color);
		}
	}

	/// <summary>
	/// Open the OpenNI (Kinect) environment.
	/// </summary>
	/// <returns>True if the process was successful; false otherwise.</returns>
	public static bool Initialize()
	{
		return OpenNI.Initialize() == OpenNI.Status.Ok;
	}

	/// <summary>
	/// Close the OpenNI (Kinect) environment.
	/// </summary>
	public static void Shutdown()
	{
		OpenNI.Shutdown();
	}

	/// <summary>
	/// Construct a Kinect-based vehicle from a recorded or arctive source.
	/// </summary>
	/// <param name="inputfile">Recorded data file.
	/// If null or empty, an unspecified active hardware sensor is used.</param>
	/// <param name="sidebar">True to show a sidebar with the image processing results.</param>
	public KinectVehicle(string inputfile = null, bool sidebar = true)
		: base(Pose3D.Identity, new KinectMeasurer(575.8156 / Delta,
		                                          new Rectangle(-640 / Delta / 2, -480 / Delta / 2,
		                                                         640 / Delta,      480 / Delta),
		                                          new Range(0.1f, 4f),
		                                          640, 480, 24, () => null))
	{
		try {
			device = Device.Open((!string.IsNullOrEmpty(inputfile)) ? inputfile : Device.AnyDevice);

			if (device == null || !device.HasSensor(Device.SensorType.Depth)) {
				throw new DeviceErrorException("No valid device found.");
			}
			
			depth = device.CreateVideoStream(Device.SensorType.Depth);
			color = device.CreateVideoStream(Device.SensorType.Color);

			DepthFrame = new float[0][];
			ColorFrame = new Color[0];
			
			depthcache    = new Queue<float[][]>();
			colorcache    = new Queue<Color[]>();
			featurecache  = new Queue<List<SparseItem>>();
			interest      = new List<SparseItem>();
			prevkeypoints = new List<IFeaturePoint>();
			prevprevkeypoints = new List<IFeaturePoint>();
			
			if (device.IsFile) {
				device.PlaybackControl.Speed = -1;
			}
			else {
				depth.VideoMode = new VideoMode { DataPixelFormat = VideoMode.PixelFormat.Depth1Mm,
				                                  Fps             = 30,
				                                  Resolution      = new System.Drawing.Size(640, 480) };
				color.VideoMode = new VideoMode { DataPixelFormat = VideoMode.PixelFormat.Rgb888,
				                                  Fps             = 30,
				                                  Resolution      = new System.Drawing.Size(640, 480) };
			}

			ShowSidebar = sidebar;
			HasSidebar  = sidebar;

			ResX    = depth.VideoMode.Resolution.Width  / Delta;
			ResY    = depth.VideoMode.Resolution.Height / Delta;
			xzalpha = 2 * (float) Math.Tan(depth.HorizontalFieldOfView / 2);
			yzalpha = 2 * (float) Math.Tan(depth.VerticalFieldOfView   / 2);

			Border  = 48 / 2 + 3;  // LATCH patch halfsize + ssd halfsize
			
			depth.Start();
			color.Start();

			if (device.IsFile) {
				Cache(30 * 0);
			}
		}
		catch (Exception) {
			if (device != null) {
				device.Dispose();
				device = null;
			}

			throw;
		}
	}

	/// <summary>
	/// Create a vehicle from a real sensor device.
	/// </summary>
	/// <param name="sensor">Device (or recorded file) path.</param>
	/// <param name="sidebar">True to show a sidebar with image processing details.</param>
	/// <returns>Vehicle linked to sensor.</returns>
	public static KinectVehicle FromSensor(string sensor, bool sidebar)
	{
		return new KinectVehicle(sensor, sidebar);
	}
	
	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <returns>Pixel-range measurements.</returns>
	public override List<PixelRangeMeasurement> Measure(GameTime time)
	{
		var measurements = new List<PixelRangeMeasurement>();
		
		NextFrame(out DepthFrame, out ColorFrame, out interest);

		for (int i = 0; i < interest.Count; i++) {
			float range = GetRange(interest[i].I, interest[i].K, (float) interest[i].Value);

			measurements.Add(new PixelRangeMeasurement(interest[i].I - ResX / 2, interest[i].K - ResY / 2, range));
		}
		
		if (ShowSidebar && depthsensed != null) {
			DepthMatrixToTexture(depthsensed, DepthFrame);
			ColorMatrixToTexture(colorsensed, ColorFrame);
		}

		MappedMeasurements.Clear();
		foreach (PixelRangeMeasurement z in measurements) {
			MappedMeasurements.Add(Measurer.MeasureToMap(Pose, z));
		}

		return measurements;
	}

	/// <summary>
	/// Read a frame from the cache if possible, otherwise use the original device.
	/// </summary>
	/// <param name="depthframe">Out. Sensor depth output.</param>
	/// <param name="colorframe">Out. Sensor color output.</param>
	/// <param name="interest">Out. List of interest points extracted from the map.</param>
	private void NextFrame(out float[][] depthframe, out Color[] colorframe, out List<SparseItem> interest)
	{
		if (depthcache.Count > 0) {
			depthframe = depthcache  .Dequeue();
			colorframe = colorcache  .Dequeue();
			interest   = featurecache.Dequeue();

		}
		else {
			ReadProcessFrame(out depthframe, out colorframe, out interest);
		}
	}

	/// <summary>
	/// Cache the stream if possible (file or wait for the kinect to collect some data).
	/// </summary>
	/// <remarks>For now, only the file caching system is implemented.</remarks>
	/// <param name="maxframes">Maximum number of frames to be cached. It can be less if the
	/// stream dies before reaching to that point.</param>
	private void Cache(int maxframes)
	{
		Console.WriteLine("Starting cache process...");
		for (int i = 0; i < maxframes; i++) {
			if (!depth.IsValid) {
				break;
			}
			
			float[][]        depthbitmap;
			Color[]          colorbitmap;
			List<SparseItem> interest;

			ReadProcessFrame(out depthbitmap, out colorbitmap, out interest);

			depthcache  .Enqueue(depthbitmap);
			colorcache  .Enqueue(colorbitmap);
			featurecache.Enqueue(interest);
			Console.WriteLine("frame " + (i + 1) + " / " + maxframes);
		}
	}

	/// <summary>
	/// Get a frame from the device and process it.
	/// </summary>
	/// <param name="depthframe">Out. Sensor depth output.</param>
	/// <param name="colorframe">Out. Sensor color output.</param>
	/// <param name="interest">Out. List of interest points extracted from the map.</param>
	private void ReadProcessFrame(out float[][] depthframe, out Color[] colorframe, out List<SparseItem> interest)
	{
		using (VideoFrameRef frameref = depth.ReadFrame()) {
			depthframe = DepthFrameToArray(frameref);
		}

		using (VideoFrameRef frameref = color.ReadFrame()) {
			colorframe = ColorFrameToArray(frameref);
		}

		interest = ExtractKeypoints(colorframe, depthframe, (int) ResX, (int) ResY, 20);
	}

	/// <summary>
	/// Get a grayscale image from the sensor internal data in float format.
	/// </summary>
	/// <param name="image">Original frame reference.</param>
	/// <returns>Image data array.</returns>
	private float[][] DepthFrameToArray(VideoFrameRef image)
	{
		IntPtr    data      = image.Data;
		int       width     = image.VideoMode.Resolution.Width;
		int       height    = image.VideoMode.Resolution.Height;
		int       subwidth  = width / Delta;
		int       subheight = height / Delta;

		short[] copy = new short[width * height];
		Marshal.Copy(data, copy, 0, copy.Length);


		float[][] array = new float[subwidth][];
		
		for (int i = 0; i < array.Length; i++) {
			array[i] = new float[subheight];
		}

		// calculate averages in Delta x Delta neighborhoods
		int h = 0;
		for (int k = 0; k < array[0].Length; k++) {
		for (int m = 0; m < Delta; m++) {
			for (int i = 0; i < array.Length; i++) {
			for (int n = 0; n < Delta; n++) {
				array[i][k] += (copy[h] == 0) ? float.NaN : copy[h];
				h++;
			}
			}
		}
		}

		float alpha = 1.0f / (Delta * Delta);

		for (int i = 0; i < array.Length; i++) {
			for (int k = 0; k < array[0].Length; k++) {
				array[i][k] *= alpha;
			}
		}

		return array;
	}

	/// <summary>
	/// Get a color image from the sensor internal data in Color format.
	/// </summary>
	/// <param name="image">Original frame reference.</param>
	/// <returns>Image data array.</returns>
	private Color[] ColorFrameToArray(VideoFrameRef image)
	{
		IntPtr    data      = image.Data;
		int       width     = image.VideoMode.Resolution.Width;
		int       height    = image.VideoMode.Resolution.Height;
		int       subwidth  = width / Delta;
		int       subheight = height / Delta;

		byte[] copy = new byte[3 * width * height];
		Marshal.Copy(data, copy, 0, copy.Length);

		Vector3[] lpvector = new Vector3[subwidth * subheight];
		Color[]   lowpass  = new Color[subwidth * subheight];

		int h = 0;
		int m = 0;

		for (int k = 0; k < subheight; k++) {
		for (int p = 0; p < Delta; p++) {
			m = subwidth * k;
			
			for (int i = 0; i < subwidth; i++, m++) {
			for (int n = 0; n < Delta; n++) {
				lpvector[m].X += copy[h++];
				lpvector[m].Y += copy[h++];
				lpvector[m].Z += copy[h++];
			}
			}
		}
		}

		float alpha = 1.0f / (Delta * Delta);

		for (m = 0; m < lowpass.Length; m++) {
			lowpass[m].R = (byte) (alpha * lpvector[m].X);
			lowpass[m].G = (byte) (alpha * lpvector[m].Y);
			lowpass[m].B = (byte) (alpha * lpvector[m].Z);
			lowpass[m].A = (byte) 255;
		}

		return lowpass;
	}

	/// <summary>
	/// Straightforward reverse comparer (big goes before small).
	/// </summary>
	private class ReverseComparer : IComparer<int>
	{
		public int Compare(int a, int b) { return b - a; }
	}

	/// <summary>
	/// Extract keypoints from an image using the FREAK methodology.
	/// </summary>
	/// <param name="image">Input color image.</param>
	/// <param name="depth">Input depth map.</param>
	/// <param name="width">Input image width.</param>
	/// <param name="height">Input image height.</param>
	/// <param name="threshold">Selection threshold value. Higher gives less keypoints.</param>
	/// <returns>List of keypoints in measurement space.</returns>
	private List<SparseItem> ExtractKeypoints(Color[] image, float[][] depth, int width, int height, int threshold)
	{
		List<IFeaturePoint> keypointsF = ExtractRawKeyPoints(image, width, height, KeypointFilter);
		List<SparseItem>    keypoints  = new List<SparseItem>();

		List<IFeaturePoint> filtered = new List<IFeaturePoint>();

		if (KeypointFilter && keypointsF.Count > 4 && prevkeypoints.Count > 4) {
			var descriptors = new Dictionary<IntPoint, IFeaturePoint>();

			foreach (var point in keypointsF) {
				descriptors[new IntPoint((int) point.X, (int) point.Y)] = point;
			}

			var matcher = new KNearestNeighborMatching(3, Distance.Hamming);
			matcher.Threshold = 0.37;

			var matches = matcher.Match(prevkeypoints, keypointsF);
			var ransac  = new RansacHomographyEstimator(0.1, 0.999);

			this.matches    = new IntPoint[2][];
			this.matches[0] = new IntPoint[0];

			try {
				if (matches[0].Length > 4) {
					ransac.Estimate(matches);
					int[] inliers = ransac.Inliers;
					
					filtered = new List<IFeaturePoint>();
					
					this.matches    = new IntPoint[2][];
					this.matches[0] = new IntPoint[inliers.Length];
					this.matches[1] = new IntPoint[inliers.Length];
					
					for (int i = 0; i < inliers.Length; i++) {
						int x = matches[1][inliers[i]].X;
						int y = matches[1][inliers[i]].Y;
						
						this.matches[0][i] = matches[0][inliers[i]];
						this.matches[1][i] = matches[1][inliers[i]];
						
						if (depth[x][y] > 0) {
							filtered.Add(descriptors[matches[1][inliers[i]]]);
						}
					}
				}
			}
			catch (Accord.ConvergenceException) {
				// just continue, like if not enough points were found
			}
		}
		else {
			for (int i = 0; i < keypointsF.Count; i++) {
				int x = (int) keypointsF[i].X;
				int y = (int) keypointsF[i].Y;

				if (depth[x][y] > 0) {
					filtered.Add(keypointsF[i]);
				}
			}
		}

		this.prevprevkeypoints = this.prevkeypoints;
		this.prevkeypoints     = keypointsF;

		foreach (var point in filtered) {
			int x = (int) point.X;
			int y = (int) point.Y;

			keypoints.Add(new SparseItem(x, y, depth[x][y]));
		}

		return keypoints;
	}

	/// <summary>
	/// Set the depth data stream of a Texture2D object.
	/// </summary>
	/// <param name="target">Render target.</param>
	/// <param name="matrix">Original matrix.</param>
	private static void DepthMatrixToTexture(Texture2D target, float[][] matrix)
	{
		var data = new Color[matrix.Length * matrix[0].Length];

		float maxval = matrix.Max();

		int h = 0;
		for (int k = 0; k < matrix[0].Length; k++) {
		for (int i = 0; i < matrix   .Length; i++) {
			int gray = (int)(matrix[i][k] / maxval * 255);
			data[h++] = new Color(gray, gray, gray);
		}
		}

		target.SetData(data);
	}

	/// <summary>
	/// Set the color data stream of a Texture2D object.
	/// </summary>
	/// <param name="target">Render target.</param>
	/// <param name="matrix">Original matrix.</param>
	private static void ColorMatrixToTexture(Texture2D target, Color[] matrix)
	{
		target.SetData(matrix);
	}

	/// <summary>
	/// Obtain an UnmanagedImage object from
	/// a squeezed color matrix.
	/// </summary>
	/// <param name="matrix">Original matrix.</param>
	/// <param name="width">Image width.</param>
	/// <param name="height">Image height.</param>
	/// <returns>Color unmanaged image.</returns>
	private static UnmanagedImage ColorMatrixToImage(Color[] matrix, int width, int height)
	{
		var data = new System.Drawing.Color[matrix.Length];
		UnmanagedImage image = null;

		for (int i = 0; i < matrix.Length; i++) {
			Color color = matrix[i];
			data[i] = System.Drawing.Color.FromArgb((int) color.PackedValue);
		}

		var converter = new Accord.Imaging.Converters.ArrayToImage(width, height);
		converter.Convert(data, out image);

		return image;
	}

	/// <summary>
	/// Obtain a grayscale UnmanagedImage object from
	/// a squeezed color matrix.
	/// </summary>
	/// <param name="matrix">Original matrix.</param>
	/// <param name="width">Image width.</param>
	/// <param name="height">Image height.</param>
	/// <returns>Color unmanaged image.</returns>
	private static UnmanagedImage ColorMatrixToGrayImage(Color[] matrix, int width, int height)
	{
		var data = new byte[matrix.Length];
		UnmanagedImage image = null;

		for (int i = 0; i < matrix.Length; i++) {
			data[i] = (byte)(1/3.0 * ((float) matrix[i].R + (float) matrix[i].G + (float) matrix[i].B));
		}

		var converter = new Accord.Imaging.Converters.ArrayToImage(width, height);
		converter.Convert(data, out image);

		return image;
	}

	/// <summary>
	/// Obtain a grayscale UnmanagedImage object from
	/// a squeezed color matrix.
	/// </summary>
	/// <param name="matrix">Original matrix.</param>
	/// <param name="width">Image width.</param>
	/// <param name="height">Image height.</param>
	/// <param name="computeDescriptors">If true, calculate a matching descriptor for each feature.</param>
	/// <returns>Color unmanaged image.</returns>
	private unsafe static List<IFeaturePoint> ExtractRawKeyPoints(Color[] matrix, int width, int height, bool computeDescriptors)
	{
		const int dsize = 32;
		byte[]    data  = new byte[matrix.Length];
		double*   kp;
		byte*     desc;
		int       length;

		for (int i = 0; i < matrix.Length; i++) {
			data[i] = (byte)(1/3.0 * ((float) matrix[i].R + (float) matrix[i].G + (float) matrix[i].B));
		}

		List<IFeaturePoint> keypoints = new List<IFeaturePoint>();

		fixed(byte* pdata = data) {
			IntPtr pdesc;
			kp   = (double*) getkeypoints((IntPtr) pdata, height, width,
			                              computeDescriptors, out length, out pdesc);
			desc = (byte*) pdesc;
		}

		if (computeDescriptors) {
			for (int i = 0, h = 0, m = 0; i < length; i++) {
				double x = kp[h++];
				double y = kp[h++];
				byte[] descriptor = new byte[dsize];

				for (int k = 0; k < dsize; k++) {
					descriptor[k] = desc[m++];
				}

				keypoints.Add(new SimpleKeyPoint(x, y, descriptor));
			}
		}
		else {
			for (int i = 0, h = 0; i < length; i++) {
				double x = kp[h++];
				double y = kp[h++];
				keypoints.Add(new SimpleKeyPoint(x, y));
			}
		}

		deletearrayd((IntPtr) kp);
		deletearrayc((IntPtr) desc);

		return keypoints;
	}

	[DllImport("libkpextractor.so")]
	private extern static IntPtr getkeypoints(IntPtr data, int rows, int cols, [MarshalAs(UnmanagedType.U1)] bool calcdescriptors, out int length, out IntPtr descriptors);

	[DllImport("libkpextractor.so")]
	private extern static void deletearrayd(IntPtr data);

	[DllImport("libkpextractor.so")]
	private extern static void deletearrayc(IntPtr data);

	/// <summary>
	/// Transform a point in local 3D space (x-y-depth) into a range measurement.
	/// </summary>
	/// <param name="px">Pixel x-coordinate.</param>
	/// <param name="py">Pixel y-coordinate.</param>
	/// <param name="z">Depth coordinate.</param>
	/// <returns>Point range.</returns>
	private float GetRange(int px, int py, float z)
	{
		// depth in OpenNI is the processed z-axis, not the range and its measured in meters
		float nx = px / ResX - 0.5f;
		float ny = 0.5f - py / ResY;

		z /= 5000;

		float x = xzalpha * nx * z;
		float y = yzalpha * ny * z;
		
		return (float) Math.Sqrt(x * x + y * y + z * z);
	}

	/// <summary>
	/// Render the vehicle on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public override void Render(double[][] camera)
	{
		base.Render(camera);
		RenderDepth(camera);
	}

	/// <summary>
	/// Render the depth as a surface onto the scene.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderDepth(double[][] camera)
	{
		Vector3[][] vertices = new Vector3[(int) ResX][];
		Color color  = Color.White;

		for (int i = 0; i < DepthFrame.Length; i++) {
			vertices[i] = new Vector3[(int) ResY];
		}

		float m = 0;

		for (int k = 0; k < DepthFrame[0].Length; k++) {
		for (int i = 0; i < DepthFrame   .Length; i++) {
			float range = GetRange(i, k, DepthFrame[i][k]);
			m = Math.Max(m, DepthFrame[i][k]);
			
			double[] local = Measurer.MeasureToMap(Pose,
			                     new PixelRangeMeasurement(i - ResX / 2, k - ResY / 2, range));
			vertices[i][k] = camera.TransformH(local).ToVector3();
		}
		}

		Graphics.DrawGrid(vertices, color, false);
	}

	/// <summary>
	/// Render the sidebar info screen.
	/// It shows the camera depth data and processed feature points.
	/// </summary>
	public override void RenderSide()
	{
		int vidwidth  = depthsensed.Width;
		int vidheight = depthsensed.Height;
		int gwidth    = Graphics.Viewport.Width;
		int gheight   = Graphics.Viewport.Height;

		Flip.Draw(depthsensed, new Rectangle(0, 0,           gwidth,   gheight / 2),
		                       new Rectangle(0, 0,           vidwidth, vidheight),
		                       Color.Gray, 0, Vector2.Zero, SpriteEffects.None, 1);

		Flip.Draw(colorsensed, new Rectangle(0, gheight / 2, gwidth,   gheight / 2),
		                       new Rectangle(0, 0,           vidwidth, vidheight),
		                       Color.Gray, 0, Vector2.Zero, SpriteEffects.None, 1);
	}

	/// <summary>
	/// Render a Heads Up Display information, on top of any other graphics in the sidebar.
	/// Additional info regarding keypoint matching is presented.
	/// </summary>
	public override void RenderSideHUD()
	{
		int vidwidth  = depthsensed.Width;
		int vidheight = depthsensed.Height;
		int gwidth    = Graphics.Viewport.Width;
		int gheight   = Graphics.Viewport.Height;
		int tgheight  = gheight / 2;

		float ratio = (float) gwidth / vidwidth;

		if (matches.Length > 0) {
			for (int i = 0; i < matches[0].Length; i++) {
				VertexPositionColor[] vertices = new VertexPositionColor[2];

				vertices[0] = new VertexPositionColor(new Vector3(matches[0][i].X * ratio, matches[0][i].Y * ratio, 0), Color.Red);
				vertices[1] = new VertexPositionColor(new Vector3(matches[1][i].X * ratio, matches[1][i].Y * ratio, 0), Color.Red);

				Graphics.DrawUserPrimitives(PrimitiveType.LineList, vertices, 0, 1);
			}
		}

		foreach (var point in prevprevkeypoints) {
			double[][] vertices = new double[4][];

			vertices[0] = new double[3] {point.X * ratio + 2, point.Y * ratio - 2, 0};
			vertices[1] = new double[3] {point.X * ratio + 2, point.Y * ratio + 2, 0};
			vertices[2] = new double[3] {point.X * ratio - 2, point.Y * ratio + 2, 0};
			vertices[3] = new double[3] {point.X * ratio - 2, point.Y * ratio - 2, 0};

			Graphics.DrawUser2DPolygon(vertices, 1.0f, Color.Blue, true);
		}

		foreach (var point in interest) {
			double[][] vertices = new double[4][];

			vertices[0] = new double[3] {point.I * ratio + 2, point.K * ratio - 2, 0};
			vertices[1] = new double[3] {point.I * ratio + 2, point.K * ratio + 2, 0};
			vertices[2] = new double[3] {point.I * ratio - 2, point.K * ratio + 2, 0};
			vertices[3] = new double[3] {point.I * ratio - 2, point.K * ratio - 2, 0};

			Graphics.DrawUser2DPolygon(vertices, 1.0f, Color.White, true);

			vertices[0][1] += tgheight;
			vertices[1][1] += tgheight;
			vertices[2][1] += tgheight;
			vertices[3][1] += tgheight;

			Graphics.DrawUser2DPolygon(vertices, 1.2f, Color.Red, true);
		}
	}

	/// <summary>
	/// Clone an associated vehicle with a new associated simulation particle.
	/// Polymorphism on the return value is allowed and encouraged
	/// to provide specific traits for particular reference vehicle.
	/// Assume default cloning parameters.
	/// </summary>
	/// <param name="vehicle">Vehicle to clone.</param>
	/// <param name="copytrajectory">If true, copy the whole trajectory history.</param>
	/// <returns>The clone.</returns>
	public override TrackVehicle<KinectMeasurer, Pose3D, PixelRangeMeasurement>
	                    TrackClone(TrackVehicle<KinectMeasurer, Pose3D, PixelRangeMeasurement> vehicle,
	                               bool         copytrajectory = false)
	{
		if (vehicle is KinectTrackVehicle) {
			return new KinectTrackVehicle((KinectTrackVehicle) vehicle, copytrajectory);
		}
		else {
			throw new ArgumentException("A Kinect vehicle should be tracked with KinectTrackVehicles");
		}
	}

	/// <summary>
	/// Clone this vehicle with an associated simulation particle.
	/// Polymorphism on the return value is allowed and encouraged
	/// to provide specific traits for particular reference vehicle.
	/// Specify all cloning parameters.
	/// </summary>
	/// <param name="motioncovmultiplier">Motion covariance multiplier.</param>
	/// <param name="measurecovmultiplier">Measurement covariance multiplier.</param>
	/// <param name="pdetection">Detection probability.</param>
	/// <param name="clutter">Clutter density.</param>
	/// <param name="copytrajectory">If true, copy the whole trajectory history.</param>
	/// <returns>The clone.</returns>
	public override TrackVehicle<KinectMeasurer, Pose3D, PixelRangeMeasurement>
	                    TrackClone(double  motioncovmultiplier,
	                               double  measurecovmultiplier,
	                               double  pdetection,
	                               double  clutter,
	                               bool    copytrajectory = false)
	{
		return new KinectTrackVehicle(this, motioncovmultiplier, measurecovmultiplier,
		                              pdetection, clutter, copytrajectory);
	}

	/// <summary>
	/// Dispose all assets.
	/// </summary>
	public override void Dispose()
	{
		if (device != null) {
			device.Dispose();
		}
	}
}

public class SimpleKeyPoint : IFeaturePoint
{
	public double X { get; set; }
	public double Y { get; set; }
	public double[] Descriptor { get; set; }

	public SimpleKeyPoint(double x, double y)
	{
		X          = x;
		Y          = y;
		Descriptor = new double[0];
	}

	public SimpleKeyPoint(double x, double y, byte[] descriptor)
	{
		X          = x;
		Y          = y;
		Descriptor = new double[8 * descriptor.Length];

		for (int i = 0, h = 0; i < descriptor.Length; i++) {
			for (int k = 0, p = (1 << 7); k < 8; k++, p >>= 1) {
			Descriptor[h++] = ((descriptor[i] & p) != 0) ? 1 : 0;
		}
		}
	}
}
}
