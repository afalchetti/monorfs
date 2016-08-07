// IMeasurer.cs
// Measuring and related methods and configurations
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

using Microsoft.Xna.Framework.Graphics;

namespace monorfs
{
/// <summary>
/// Measuring and related methods and configurations.
/// </summary>
public interface IMeasurer<MeasurerT, PoseT, MeasurementT>
	where PoseT        : IPose<PoseT>, new()
	where MeasurementT : IMeasurement<MeasurementT>, new()
{
	/// <summary>
	/// Obtain a linear representation for the measurer.
	/// </summary>
	/// <returns>Linear representation.</returns>
	double[] ToLinear();

	/// <summary>
	/// Retrieve the measurer from a linear representation.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	/// <returns>Measurer.</returns>
	MeasurerT FromLinear(double[] linear);

	/// <summary>
	/// Compute the volume of the visible area (in measurement space).
	/// </summary>
	double Volume();

	/// <summary>
	/// Obtain a measurement from the given pose.
	/// It does not use any randomness or misdetection.
	/// </summary>
	/// <param name="pose">Pose from which the measurement was made.</param>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Measurement.</returns>
	MeasurementT MeasurePerfect(PoseT pose, double[] landmark);

	/// <summary>
	/// Obtain the jacobian of the measurement model wrt. the landmark.
	/// </summary>
	/// <param name="pose">Pose from which the measurement was made.</param>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Measurement model linearization jacobian.</returns>
	double[][] MeasurementJacobianL(PoseT pose, double[] landmark);

	/// <summary>
	/// Obtain the jacobian of the measurement model wrt. the pose.
	/// </summary>
	/// <param name="pose">Pose from which the measurement was made.</param>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Measurement model linearization jacobian.</returns>
	double[][] MeasurementJacobianP(PoseT pose, double[] landmark);

	/// <summary>
	/// Given a measurement and a landmark, find the pose that best relates the two.
	/// The problem is underconstrained, i.e. there is a solution for every rotation.
	/// The output will be a compromise between rotation and translation distance to
	/// the initial estimate.
	/// </summary>
	/// <param name="pose0">Initial estimate.</param>
	/// <param name="measurement">Measurement.</param>
	/// <param name="landmark">Landmark.</param>
	/// <returns>Best fit for the landmark-measurement pair.</returns>
	PoseT FitToMeasurement(PoseT pose0, MeasurementT measurement, double[] landmark);

	/// <summary>
	/// Obtain a noise measurement, uniformly random in the visible area.
	/// </summary>
	/// <returns>Random measurement.</returns>
	MeasurementT RandomMeasure();

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle
	/// using measurement coordinates to express the landmark.
	/// </summary>
	/// <param name="landmark">Queried landmark in measurement coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	bool VisibleM(MeasurementT landmark);

	/// <summary>
	/// Find if a landmark is visible in measurement space and
	/// return a fuzzy value for points near the border of the visible region.
	/// </summary>
	/// <param name="landmark">Queried landmark in pixel-range coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	double FuzzyVisibleM(MeasurementT landmark);

	/// <summary>
	/// Transform a measurement vector in measurement space into a map-space vector.
	/// </summary>
	/// <param name="pose">Pose from which the measurement was made.</param>
	/// <param name="measurement">Measurement in m-space.</param>
	/// <returns>Measurement expressed in 3D space.</returns>
	double[] MeasureToMap(PoseT pose, MeasurementT measurement);

	/// <summary>
	/// Render the vehicle physical body on the graphics device.
	/// </summary>
	/// <param name="graphics">Graphic context.</param>
	/// <param name="pose">Pose.</param>
	/// <param name="camera">Camera 4d transform matrix.</param>
	void RenderBody(GraphicsDevice graphics, PoseT pose, double[][] camera);

	/// <summary>
	/// Render the Field-of-View cone on the graphics device.
	/// </summary>
	/// <param name="graphics">Graphic context.</param>
	/// <param name="pose">Pose.</param>
	/// <param name="camera">Camera 4d transform matrix.</param>
	void RenderFOV(GraphicsDevice graphics, PoseT pose, double[][] camera);

	/// <summary>
	/// Create a vehicle descriptor string.
	/// </summary>
	/// <param name="format">Stirng format for double values.</param>
	/// <returns>Simulated vehicle descriptor string.</returns>
	string ToString(string format);
}
}
