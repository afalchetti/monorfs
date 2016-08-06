// KinectMeasurer.cs
// Pose3D + PixelRangeMeasurement measurer with overrides for Kinect sensor
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

using AForge;
using Accord.Math;

using Microsoft.Xna.Framework;

namespace monorfs
{
/// <summary>
/// Measuring and related methods and configurations for
/// the Pose3D + PixelRangeMeasurement combination
/// in use with a Kinect sensor.
/// </summary>
public class KinectMeasurer : PRM3DMeasurer, IMeasurer<KinectMeasurer, Pose3D, PixelRangeMeasurement>
{
	/// <summary>
	/// Horizontal resolution of the real vehicle.
	/// </summary>
	public float ResX { get; private set; }

	/// <summary>
	/// Vertical resolution of the real vehicle.
	/// </summary>
	public float ResY { get; private set; }

	/// <summary>
	/// Image border where no keypoint can be found (because of incomplete patches).
	/// </summary>
	public int Border { get; private set; }

	/// <summary>
	/// Kinect real vehicle depth map accesor.
	/// </summary>
	public Func<float[][]> GetDepth { get; private set; }

	/// <summary>
	/// Construct a new KinectMeasurer object from default camera parameters.
	/// </summary>
	public KinectMeasurer()
		: this(575.8156,
		       new Rectangle(-640 / 2, -480 / 2, 640, 480),
		       new Range(0.1f, 2f),
	           640, 480, 24, () => new float[1][] {new float[1] {float.PositiveInfinity}} ) {}

	/// <summary>
	/// Construct a new PRM3DMeasurer object from its camera parameters.
	/// </summary>
	/// <param name="focal">Focal length.</param>
	/// <param name="film">Film area.</param>
	/// <param name="clip">Range clipping area.</param>
	public KinectMeasurer(double focal, Rectangle film, Range clip,
	                      float resx, float resy, int border, Func<float[][]> getdepth)
		: base(focal, film, clip)
	{
			this.ResX     = resx;
			this.ResY     = resy;
			this.Border   = border;
			this.GetDepth = getdepth;
	}

	/// <summary>
	/// Retrieve the measurer from a linear representation.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	/// <returns>Measurer.</returns>
	KinectMeasurer IMeasurer<KinectMeasurer, Pose3D, PixelRangeMeasurement>.FromLinear(double[] linear)
	{
		if (linear.Length != 10) {
			throw new ArgumentException("Linear representation must have exactly 10 parameters.");
		}

		PRM3DMeasurer b = new PRM3DMeasurer().FromLinear(linear.Submatrix(0, 6));
		return new KinectMeasurer(b.VisionFocal, b.FilmArea, b.RangeClip,
		                          (float) linear[7], (float) linear[8], (int) linear[9],
		                          () => new float[1][] {new float[1] {float.PositiveInfinity}});
	}

	/// <summary>
	/// Retrieve the measurer from a linear representation.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	/// <returns>Measurer.</returns>
	double[] IMeasurer<KinectMeasurer, Pose3D, PixelRangeMeasurement>.ToLinear()
	{
		return ToLinear().Concatenate(new double[3] { ResX, ResY, Border });
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle
	/// using measurement coordinates to express the landmark.
	/// </summary>
	/// <param name="landmark">Queried landmark in measurement coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public override bool VisibleM(PixelRangeMeasurement landmark)
	{
		int   x     = (int) (landmark.X + ResX / 2);
		int   y     = (int) (landmark.Y + ResY / 2);
		float range = (float) landmark.Range;

		if (!base.VisibleM(landmark)) {
			return false;
		}

		float[][] depth = GetDepth();

		// the point is inbounds (redundant with base implementation, but
		// floating point errors could make it necessary
		if (y < 0 || y >= depth.Length || x < 0 || x >= depth[0].Length) {
			return false;
		}

		// the point is in front of the depth map (i.e. not occluded)
		return range <= depth[x][y];
	}

	/// <summary>
	/// Find if a landmark is visible in measurement space and
	/// return a fuzzy value for points near the border of the visible region.
	/// </summary>
	/// <param name="landmark">Queried landmark in pixel-range coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public override double FuzzyVisibleM(PixelRangeMeasurement landmark)
	{
		int   x     = (int)  (landmark.X + ResX / 2);
		int   y     = (int)  (landmark.Y + ResY / 2);
		float range = (float) landmark.Range;

		double    minwdistance = base.FuzzyVisibleM(landmark);

		if (minwdistance == 0) {  // outside of image region, can't obtain depth
			return 0;
		}

		float[][] depth = GetDepth();

		minwdistance = Math.Min(minwdistance, (range - RangeClip.Min)  / VisibilityRamp[2]);
		minwdistance = Math.Min(minwdistance, (depth[x][y]  - range) / VisibilityRamp[2]);

		return Math.Max(0, Math.Min(1, minwdistance));
	}
}
}
