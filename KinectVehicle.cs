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
	/// Sensor depth image reading.
	/// </summary>
	private Texture2D sensed;

	/// <summary>
	/// Sensor depth image reading.
	/// </summary>
	private Texture2D features;

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
			
			if (device.IsFile) {
				device.PlaybackControl.Speed = -1;
			}
			else {
				depth.VideoMode = new VideoMode { DataPixelFormat = VideoMode.PixelFormat.Depth1Mm,
				                                  Fps             = 30,
				                                  Resolution      = new System.Drawing.Size(320, 240) };
			}

			SidebarWidth  = depth.VideoMode.Resolution.Width;
			SidebarHeight = depth.VideoMode.Resolution.Height * 2;  // two images stacked vertically
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

		using (VideoFrameRef frame = depth.ReadFrame()) {
			double[,] bitmap = FrameToArray(frame);

			//List<SparseItem> interest = LocalMax(CalculateCurvature(bitmap), 0.8, 50);
			List<SparseItem> interest = Subsample(bitmap, 10, 10);

			for (int i = 0; i < interest.Count; i++) {
				float x, y, z;

				CoordinateConverter.ConvertDepthToWorld(depth, interest[i].I, interest[i].K, (int) bitmap[interest[i].I, interest[i].K], out x, out y, out z);

				// depth in OpenNI is the processed z-axis, not the range and its measured in millimeters
				double range = Math.Sqrt(x * x + y * y + z * z) / 1000;

				measurements.Add(new double[3] {interest[i].I, interest[i].K, range});
			}
		
			if (sensed != null && features != null) {
				sensed  .SetData(MatrixToTextureData(bitmap));
				features.SetData(ListToTextureData(interest, depth.VideoMode.Resolution.Width, depth.VideoMode.Resolution.Height));
			}
		}

		return measurements;
	}

	/// <summary>
	/// Get a grayscale image internal data in double format.
	/// </summary>
	/// <param name="image">Original image.</param>
	/// <returns>Image data array.</returns>
	private double[,] FrameToArray(VideoFrameRef image)
	{
		IntPtr    data  = image.Data;
		double[,] array = new double[image.VideoMode.Resolution.Width, image.VideoMode.Resolution.Height];

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
	private List<SparseItem> LocalMax(double[,] image, double threshold, int maxcount)
	{
		List<SparseItem> list = new List<SparseItem>();

		double maxval = image.Max();

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
	private double[,] CalculateCurvature(double[,] image)
	{
		double[,] curvature = new double[image.GetLength(0), image.GetLength(1)];
		
		for (int i = 1; i < image.GetLength(0) - 1; i++) {
		for (int k = 1; k < image.GetLength(1) - 1; k++) {
			double fx  = 0.5 * (image[i + 1, k] - image[i - 1, k]);
			double fy  = 0.5 * (image[i, k + 1] - image[i, k - 1]);
			double fxx = image[i + 1, k] - 2 * image[i, k] + image[i - 1, k];
			double fyy = image[i, k + 1] - 2 * image[i, k] + image[i, k - 1];
			double fxy = 0.25 * (image[i + 1, k + 1] + image[i - 1, k - 1] - image[i - 1, k + 1] - image[i + 1, k - 1]);

			double denom = (1 + fx * fx + fy * fy);
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
	private List<SparseItem> Subsample(double[,] image, int xcount, int ycount)
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
	private Microsoft.Xna.Framework.Color[] MatrixToTextureData(double[,] matrix)
	{
		var data = new Microsoft.Xna.Framework.Color[matrix.GetLength(0) * matrix.GetLength(1)];

		double maxval = matrix.Max();
		double minval = matrix.Min();

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
