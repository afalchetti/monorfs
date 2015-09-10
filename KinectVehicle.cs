﻿// KinectVehicle.cs
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

namespace monorfs
{
/// <summary>
/// Kinect vehicle model.
/// Extracts measurements from an online Kinect sensor.
/// </summary>
public class KinectVehicle : Vehicle
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
	/// Point texture to illustrate landmark positions.
	/// </summary>
	private Texture2D landmark;

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

			landmark = CreatePoint();
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
	public KinectVehicle(string inputfile = null)
		: base(new double[3] {0, 0, 0}, Math.PI, new double[3] {1, 0, 0},
		       575.8156 / Delta,
		       new Rectangle(-640 / Delta / 2, -480 / Delta / 2,
		                      640 / Delta,      480 / Delta),
		       new Range(0.1f, 2f))
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
			
			depthcache   = new Queue<float[][]>();
			colorcache   = new Queue<Color[]>();
			featurecache = new Queue<List<SparseItem>>();
			interest     = new List<SparseItem>();
			
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

			HasSidebar    = true;
			SidebarWidth  = (int) (depth.VideoMode.Resolution.Width  / Delta);
			SidebarHeight = (int) (depth.VideoMode.Resolution.Height / Delta) * 2;

			ResX    = depth.VideoMode.Resolution.Width  / Delta;
			ResY    = depth.VideoMode.Resolution.Height / Delta;
			xzalpha = 2 * (float) Math.Tan(depth.HorizontalFieldOfView / 2);
			yzalpha = 2 * (float) Math.Tan(depth.VerticalFieldOfView   / 2);
			
			depth.Start();
			color.Start();

			if (device.IsFile) {
				Cache(30 * 12);
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
	/// Obtain several measurements from the hidden state.
	/// </summary>
	/// <returns>Pixel-range measurements.</returns>
	public override List<double[]> Measure()
	{
		List<double[]> measurements = new List<double[]>();
		
		NextFrame(out DepthFrame, out ColorFrame, out interest);

		for (int i = 0; i < interest.Count; i++) {
			float range = GetRange(interest[i].I, interest[i].K, (float) interest[i].Value);

			measurements.Add(new double[3] {interest[i].I - ResX / 2, interest[i].K - ResY / 2, range});
		}
		
		if (depthsensed != null) {
			DepthMatrixToTexture(depthsensed, DepthFrame);
			ColorMatrixToTexture(colorsensed, ColorFrame);
		}

		MappedMeasurements.Clear();
		foreach (double[] z in measurements) {
			MappedMeasurements.Add(MeasureToMap(z));
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
		// remove one frame from each stream to compensate for duplication
		// (this happens because each stream creates a frame index and there are two streams)
		// note that now double-stream is mandatory as the features are extracted from the color space
		// and used with depth information so the system will always double the framerate (unless
		// the timestamps are perfectly aligned)
		// FIXME generalize this to handle perfectly aligned streams
		using (VideoFrameRef frameref = depth.ReadFrame()) {}
		using (VideoFrameRef frameref = color.ReadFrame()) {}

		using (VideoFrameRef frameref = depth.ReadFrame()) {
			depthframe = DepthFrameToArray(frameref);
		}

		using (VideoFrameRef frameref = color.ReadFrame()) {
			colorframe = ColorFrameToArray(frameref);
		}

		interest = ExtractKeypoints(colorframe, depthframe, (int) ResX, (int) ResY, 100);
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

		int h = 0;
		for (int k = 0; k < array[0].Length; k++) {
			h = width * k * Delta;

			for (int i = 0; i < array.Length; i++) {
				array[i][k] = copy[h];
				h += Delta;
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

		Color[] subsampled = new Color[subwidth * subheight];

		int h = 0;
		int m = 0;
		for (int k = 0; k < subheight; k++) {
			h = width * k * Delta * 3;

			for (int i = 0; i < subwidth; i++) {
				subsampled[m] = new Color(copy[h], copy[h + 1], copy[h + 2]);

				h += Delta * 3;
				m++;
			}
		}

		return subsampled;
	}

	/// <summary>
	/// Get a list of local maxima using the 8-neighborhood.
	/// </summary>
	/// <param name="image">Original image.</param>
	/// <param name="threshold">Minimum fraction of the maximum value.</param>
	/// <param name="maxcount">Maximum number of interest points. The highest values are chosen.</param>
	/// <returns>Local maxima list.</returns>
	private List<SparseItem> LocalMax(float[][] image, float threshold, int maxcount)
	{
		List<SparseItem> list = new List<SparseItem>();

		float maxval = image.Max();

		for (int i = 1; i < image   .Length - 1; i++) {
		for (int k = 1; k < image[0].Length - 1; k++) {
			bool localmax = image[i][k] > image[i - 1][k    ] &&
			                image[i][k] > image[i + 1][k    ] &&
			                image[i][k] > image[i    ][k - 1] &&
			                image[i][k] > image[i    ][k + 1] &&
			                image[i][k] > image[i + 1][k - 1] &&
			                image[i][k] > image[i + 1][k + 1] &&
			                image[i][k] > image[i - 1][k - 1] &&
			                image[i][k] > image[i - 1][k + 1];

			if ((image[i][k] > threshold * maxval) && localmax) {
				list.Add(new SparseItem(i, k, image[i][k]));
			}
		}
		}

		list.Sort((a, b) => Math.Sign(a.Value - b.Value));

		if (maxcount < list.Count) {
			list.RemoveRange(maxcount, list.Count - maxcount);
		}

		return list;
	}

	/// <summary>
	/// Calculate the intrinsic gaussian curvature of an image.
	/// </summary>
	/// <param name="image">Original image.</param>
	/// <returns>Intrinsic gaussian curvature (k1 x k2).</returns>
	private float[][] CalculateCurvature(float[][] image)
	{
		float[][] curvature = new float[image.Length][];
		
		for (int i = 0; i < curvature.Length; i++) {
			curvature[i] = new float[image[0].Length];
		}
		
		for (int i = 1; i < image   .Length - 1; i++) {
		for (int k = 1; k < image[0].Length - 1; k++) {
			float fx  = 0.5f * (image[i + 1][k] - image[i - 1][k]);
			float fy  = 0.5f * (image[i][k + 1] - image[i][k - 1]);
			float fxx = image[i + 1][k] - 2 * image[i][k] + image[i - 1][k];
			float fyy = image[i][k + 1] - 2 * image[i][k] + image[i][k - 1];
			float fxy = 0.25f * (image[i + 1][k + 1] + image[i - 1][k - 1] - image[i - 1][k + 1] - image[i + 1][k - 1]);

			float denom     = (1 + fx * fx + fy * fy);
			curvature[i][k] = (fxx * fyy - fxy * fxy) / (denom * denom);
		}
		}

		return curvature;
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
	private static List<SparseItem> ExtractKeypoints(Color[] image, float[][] depth, int width, int height, int threshold = 20)
	{
		var detector = new FastRetinaKeypointDetector(threshold);
		int border   = Math.Min(50, (int) (width * 0.05));
		List<SparseItem> keypoints = new List<SparseItem>();

		detector.ComputeDescriptors = FastRetinaKeypointDescriptorType.None;

		using (UnmanagedImage bitmap = ColorMatrixToImage(image, width, height)) {
			FastRetinaKeypoint[] features  = detector.ProcessImage(bitmap).ToArray();

			for (int i = 0; i < features.Length; i++) {
				FastRetinaKeypoint point = features[i];

				int x = (int) point.X;
				int y = (int) point.Y;

				if (x >= border && x < bitmap.Width - border && y >= border && y < bitmap.Height - border && depth[x][y] != 0) {
					keypoints.Add(new SparseItem(x, y, depth[x][y]));
				}
			}
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
	/// Obtain a data stream for a UnmanagedImage object from
	/// a bidimensional matrix.
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
			if (range == 0) {
				range = float.NaN;
			}
			
			double[] local = MeasureToMap(new double[3] {i - ResX / 2, k - ResY / 2, range});
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

		float ratio = (float) gwidth / vidwidth;

		Flip.Draw(depthsensed, new Rectangle(0, 0,           gwidth,   gheight / 2),
		                       new Rectangle(0, 0,           vidwidth, vidheight), Color.Gray);

		Flip.Draw(colorsensed, new Rectangle(0, gheight / 2, gwidth,   gheight / 2),
		                       new Rectangle(0, 0,           vidwidth, vidheight), Color.White);

		foreach (var point in interest) {
			Flip.Draw(landmark, new Vector2(point.I * ratio - landmark.Width / 2, point.K * ratio - landmark.Height / 2));
			Flip.Draw(landmark, new Vector2(point.I * ratio - landmark.Width / 2, point.K * ratio - landmark.Height / 2 + gheight / 2));
		}
	}

	/// <summary>
	/// Create a 3-pixel wide point texture.
	/// </summary>
	/// <returns>Point texture.</returns>
	public Texture2D CreatePoint()
    {
        const int side = 5;
        Texture2D point = new Texture2D(Graphics, side, side);
        Color[]   data  = new Color[side * side];

		// white interior
        for (int i = 0; i < data.Length; i++) {
			data[i] = Color.White;
        }
		
		// black border
        for (int i = 0; i < side; i++) {
			data[i + side * 0]          = Color.Black;
			data[i + side * (side - 1)] = Color.Black;
        }

        for (int k = 0; k < side; k++) {
			data[0          + side * k] = Color.Black;
			data[(side - 1) + side * k] = Color.Black;
        }

        point.SetData(data);

        return point;
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
	public override TrackVehicle TrackClone(TrackVehicle vehicle,
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
	public override TrackVehicle TrackClone(double  motioncovmultiplier,
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
}
