// KinectVehicle.cs
// Vehicle motion and measurement model for a Kinect sensor
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

using System;
using System.Collections.Generic;
using System.Linq;
using AForge;
using Accord;
using Accord.Math;

using Microsoft.Xna.Framework;
using OpenNIWrapper;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using Microsoft.Xna.Framework.Graphics;

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
	/// Sensor device.
	/// </summary>
	private Device device;

	/// <summary>
	/// Cached history depth maps.
	/// </summary>
	private Queue<float[,]> cachedmaps;

	/// <summary>
	/// Cached history processed depth map for feature extraction visualization.
	/// </summary>
	private Queue<float[,]> cachedprocessed;

	/// <summary>
	/// Cached history extracted features.
	/// </summary>
	private Queue<List<SparseItem>> cachedfeatures;

	// Cached constants

	/// <summary>
	/// Device resolution on the X-axis.
	/// </summary>
	private float resx;

	/// <summary>
	/// Device resolution on the Y-axis.
	/// </summary>
	private float resy;

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
	private Texture2D sensed;

	/// <summary>
	/// Sensor depth image reading.
	/// </summary>
	private Texture2D features;

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
			
			sensed   = new Texture2D(Graphics, depth.VideoMode.Resolution.Width, depth.VideoMode.Resolution.Height, false, SurfaceFormat.Color);
			features = new Texture2D(Graphics, depth.VideoMode.Resolution.Width, depth.VideoMode.Resolution.Height, false, SurfaceFormat.Color);
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
		: base()
	{
		try {
			device = Device.Open((!string.IsNullOrEmpty(inputfile)) ? inputfile :Device.AnyDevice);

			if (device == null || !device.HasSensor(Device.SensorType.Depth)) {
				throw new DeviceErrorException("No valid device found.");
			}

			depth = device.CreateVideoStream(Device.SensorType.Depth);
			depth.Start();

			cachedmaps      = new Queue<float[,]>();
			cachedprocessed = new Queue<float[,]>();
			cachedfeatures  = new Queue<List<SparseItem>>();
			
			if (device.IsFile) {
				device.PlaybackControl.Speed = -1;
				Cache(30 * 10);
			}
			else {
				depth.VideoMode = new VideoMode { DataPixelFormat = VideoMode.PixelFormat.Depth1Mm,
				                                  Fps             = 30,
				                                  Resolution      = new System.Drawing.Size(320, 240) };
			}

			SidebarWidth  = depth.VideoMode.Resolution.Width;
			SidebarHeight = depth.VideoMode.Resolution.Height * 2;  // two images stacked vertically

			resx    = depth.VideoMode.Resolution.Width;
			resy    = depth.VideoMode.Resolution.Height;
			xzalpha = 2 * (float) Math.Tan(depth.HorizontalFieldOfView / 2);
			yzalpha = 2 * (float) Math.Tan(depth.VerticalFieldOfView   / 2);
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
	/// Apply the motion model to the vehicle.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="dx">Moved distance from odometry in the local vertical movement-perpendicular direction since last timestep.</param>
	/// <param name="dy">Moved distance from odometry in the local horizontal movement-perpendicular direction since last timestep.</param>
	/// <param name="dz">Moved distance from odometry in the local depth movement-parallel direction since last timestep.</param>
	/// <param name="dyaw">Angle variation from odometry in the yaw coordinate since last timestep.</param>
	/// <param name="dpitch">Angle variation from odometry in the pitch coordinate since last timestep.</param>
	/// <param name="droll">Angle variation from odometry in the roll coordinate since last timestep.</param>
	public override void Update(GameTime time, double dx, double dy, double dz, double dyaw, double dpitch, double droll)
	{
		// no update logic since a real vehicle doesn't have meaningful localization
	}
	
	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// </summary>
	/// <returns>Pixel-range measurements.</returns>
	public override List<double[]> Measure()
	{
		List<double[]> measurements = new List<double[]>();

		float[,]         frame;
		float[,]         processed;
		List<SparseItem> interest;

		NextFrame(out frame, out processed, out interest);

		for (int i = 0; i < interest.Count; i++) {
			float range = GetDepth(interest[i].I, interest[i].K, frame[interest[i].I, interest[i].K]);

			measurements.Add(new double[3] {interest[i].I, interest[i].K, range});
		}
		
		if (sensed != null && features != null) {
			sensed  .SetData(MatrixToTextureData(frame));
			features.SetData(MatrixToTextureData(processed));
		}

		return measurements;
	}

	/// <summary>
	/// Read a frame from the cache if possible, otherwise use the original device.
	/// </summary>
	/// <param name="frame">Out. Sensor output.</param>
	/// <param name="processed">Out. Processed representation (for displaying purposes).</param>
	/// <param name="interest">Out. List of interest points extracted from the map.</param>
	private void NextFrame(out float[,] frame, out float[,] processed, out List<SparseItem> interest)
	{
		if (cachedmaps.Count > 0) {
			frame     = cachedmaps     .Dequeue();
			processed = cachedprocessed.Dequeue();
			interest  = cachedfeatures .Dequeue();

		}
		else {
			ReadProcessFrame(out frame, out processed, out interest);
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
		for (int i = 0; i < maxframes; i++) {
			if (!depth.IsValid) {
				break;
			}
			
			float[,]         bitmap;
			float[,]         processed;
			List<SparseItem> interest;

			ReadProcessFrame(out bitmap, out processed, out interest);

			cachedmaps     .Enqueue(bitmap);
			cachedprocessed.Enqueue(processed);
			cachedfeatures .Enqueue(interest);
		}
	}

	/// <summary>
	/// Get a frame from the device and process it.
	/// </summary>
	/// <param name="frame">Out. Sensor output.</param>
	/// <param name="processed">Out. Processed representation (for displaying purposes).</param>
	/// <param name="interest">Out. List of interest points extracted from the map.</param>
	private void ReadProcessFrame(out float[,] frame, out float[,] processed, out List<SparseItem> interest)
	{
		using (VideoFrameRef frameref = depth.ReadFrame()) {
			frame              = FrameToArray(frameref);
			float[,] curvature = CalculateCurvature(frame);
			interest           = LocalMax(curvature, 0.8f, 25);
			
			// interest = Subsample(bitmap, 10, 10);
			// interest = new List<SparseItem>();

			processed           = curvature;
		}
	}

	/// <summary>
	/// Get a grayscale image internal data in double format.
	/// </summary>
	/// <param name="image">Original image.</param>
	/// <returns>Image data array.</returns>
	private float[,] FrameToArray(VideoFrameRef image)
	{
		IntPtr    data  = image.Data;
		float[,] array = new float[image.VideoMode.Resolution.Width, image.VideoMode.Resolution.Height];

		short[] copy = new short[array.GetLength(0) * array.GetLength(1)];
		Marshal.Copy(data, copy, 0, copy.Length);
		
		int h = 0;
		for (int k = 0; k < array.GetLength(1); k++) {
		for (int i = 0; i < array.GetLength(0); i++) {
			array[i, k] = copy[h++];
		}
		}

		return array;
	}

	/// <summary>
	/// Get a list of local maxima using the 8-neighborhood.
	/// </summary>
	/// <param name="image">Original image.</param>
	/// <param name="threshold">Minimum fraction of the maximum value.</param>
	/// <param name="maxcount">Maximum number of interest points. The highest values are chosen.</param>
	/// <returns>Local maxima list.</returns>
	private List<SparseItem> LocalMax(float[,] image, float threshold, int maxcount)
	{
		List<SparseItem> list = new List<SparseItem>();

		float maxval = image.Max();

		for (int i = 1; i < image.GetLength(0) - 1; i++) {
		for (int k = 1; k < image.GetLength(1) - 1; k++) {
			bool localmax = image[i, k] > image[i - 1, k] &&
			                image[i, k] > image[i + 1, k] &&
			                image[i, k] > image[i,     k - 1] &&
			                image[i, k] > image[i,     k + 1] &&
			                image[i, k] > image[i + 1, k - 1] &&
			                image[i, k] > image[i + 1, k + 1] &&
			                image[i, k] > image[i - 1, k - 1] &&
			                image[i, k] > image[i - 1, k + 1];

			if ((image[i, k] > threshold * maxval) && localmax) {
				list.Add(new SparseItem(i, k, image[i, k]));
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
	/// <returns>Curvature.</returns>
	private float[,] CalculateCurvature(float[,] image)
	{
		float[,] curvature = new float[image.GetLength(0), image.GetLength(1)];
		
		for (int i = 1; i < image.GetLength(0) - 1; i++) {
		for (int k = 1; k < image.GetLength(1) - 1; k++) {
			float fx  = 0.5f * (image[i + 1, k] - image[i - 1, k]);
			float fy  = 0.5f * (image[i, k + 1] - image[i, k - 1]);
			float fxx = image[i + 1, k] - 2 * image[i, k] + image[i - 1, k];
			float fyy = image[i, k + 1] - 2 * image[i, k] + image[i, k - 1];
			float fxy = 0.25f * (image[i + 1, k + 1] + image[i - 1, k - 1] - image[i - 1, k + 1] - image[i + 1, k - 1]);

			float denom = (1 + fx * fx + fy * fy);
			curvature[i, k] = (fxx * fyy - fxy * fxy) / (denom * denom);
		}
		}

		return curvature;
	}

	/// <summary>
	/// Obtain a subsampled image (no filtering performed).
	/// </summary>
	/// <param name="image">Original image.</param>
	/// <param name="xcount">New matrix width.</param>
	/// <param name="ycount">Mew matrix height.</param>
	/// <returns>Subsampled matrix as a sparse entry list.</returns>
	private List<SparseItem> Subsample(float[,] image, int xcount, int ycount)
	{
		List<SparseItem> subsampled = new List<SparseItem>();

		for (int i = 0; i < xcount; i++) {
		for (int k = 0; k < ycount; k++) {
			int imgX = (int)((i + 0.5) * image.GetLength(0) / xcount);
			int imgY = (int)((k + 0.5) * image.GetLength(1) / ycount);
			subsampled.Add(new SparseItem(imgX, imgY, image[imgX, imgY]));
		}
		}

		return subsampled;
	}

	/// <summary>
	/// Obtain a data stream for a Texture2D object from
	/// a bidimensional matrix.
	/// </summary>
	/// <param name="matrix">Original matrix.</param>
	/// <returns>Color data stream.</returns>
	private Microsoft.Xna.Framework.Color[] MatrixToTextureData(float[,] matrix)
	{
		var data = new Microsoft.Xna.Framework.Color[matrix.GetLength(0) * matrix.GetLength(1)];

		float maxval = matrix.Max();
		float minval = matrix.Min();

		int h = 0;
		for (int k = 0; k < matrix.GetLength(1); k++) {
		for (int i = 0; i < matrix.GetLength(0); i++) {
			int gray = (int)((matrix[i, k] - minval) / (maxval - minval) * 255);
			data[h++] = new Microsoft.Xna.Framework.Color(gray, gray, gray);
		}
		}

		return data;
	}

	/// <summary>
	/// Obtain a data stream for a Texture2D object from
	/// a bidimensional matrix expressed as a sparse matrix.
	/// </summary>
	/// <param name="list">Original matrix represented as a list of matrix entries.</param>
	/// <param name="width">Output image width.</param>
	/// <param name="height">Output image height.</param>
	/// <returns>Color data stream.</returns>
	private Microsoft.Xna.Framework.Color[] ListToTextureData(IEnumerable<SparseItem> list, int width, int height)
	{
		var data = new Microsoft.Xna.Framework.Color[width * height];

		double maxval = list.Aggregate(double.NegativeInfinity, (cum, item) => Math.Max(cum, item.Value));
		double minval = list.Aggregate(double.PositiveInfinity, (cum, item) => Math.Min(cum, item.Value));
		
		foreach (var item in list) {
			int gray = (int)((item.Value - minval) / (maxval - minval) * 255);
			data[width * item.K + item.I] = new Microsoft.Xna.Framework.Color(gray, gray, gray);
		}

		return data;
	}

	private float GetDepth(int px, int py, float z)
	{
		// depth in OpenNI is the processed z-axis, not the range and its measured in millimeters
		float nx = px / resx - 0.5f;
		float ny = 0.5f - py / resy;

		float x = xzalpha * nx * z;
		float y = yzalpha * ny * z;
		
		return (float) Math.Sqrt(x * x + y * y + z * z) / 1000;
	}

	/// <summary>
	/// Render the sidebar info screen.
	/// It shows the camera depth data and processed feature points.
	/// </summary>
	public override void RenderSide()
	{
		int vidwidth  = depth.VideoMode.Resolution.Width;
		int vidheight = depth.VideoMode.Resolution.Height;
		int gwidth    = Graphics.Viewport.Width;
		int gheight   = Graphics.Viewport.Height;

		Flip.Draw(sensed,   new Rectangle(0, 0,          gwidth,   gheight / 2),
		                    new Rectangle(0, 0,          vidwidth, vidheight), Color.White);
		Flip.Draw(features, new Rectangle(0, gheight/ 2, gwidth,   gheight / 2),
		                    new Rectangle(0, 0,          vidwidth, vidheight), Color.White);
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
