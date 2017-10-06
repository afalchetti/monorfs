// FakeVehicle.cs
// Vehicle motion and measurement model that does nothing
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

using AForge;

using Microsoft.Xna.Framework;

namespace monorfs
{
/// <summary>
/// Fake vehicle model (does nothing).
/// </summary>
public class FakeVehicle<MeasurerT, PoseT, MeasurementT> : Vehicle<MeasurerT, PoseT, MeasurementT>
	where PoseT        : IPose<PoseT>, new()
	where MeasurementT : IMeasurement<MeasurementT>, new()
	where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
{
	/// <summary>
	/// Construct a new dummy vehicle.
	/// </summary>
	public FakeVehicle()
		: base(new PoseT().IdentityP(), new MeasurerT()) {}

	/// <summary>
	/// Construct a dummy vehicle copying metadata from another vehicle.
	/// </summary>
	/// <param name="that">Reference vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied.
	/// Relatively heavy operation.</param>
	public FakeVehicle(Vehicle<MeasurerT, PoseT, MeasurementT> that, bool copytrajectory = false)
		: base(that, copytrajectory) {}
	
	/// <summary>
	/// Apply the motion model to the vehicle. It does nothing.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="reading">Odometry reading (dx, dy, dz, dpitch, dyaw, droll).</param>
	public override void Update(GameTime time, double[] reading) {}



	/// <summary>
	/// Obtain several measurements from the hidden state. It always returns an empty list.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <returns>Pixel-range measurements.</returns>
	public override List<MeasurementT> Measure(GameTime time)
	{
		return new List<MeasurementT>();
	}
}
}
