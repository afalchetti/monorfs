// DrawUtils.cs
// Common rendering routines
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

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

using TimedState      = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedTrajectory = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<System.Tuple<double, double[]>>>>;
using TimedMapModel   = System.Collections.Generic.List<System.Tuple<double, monorfs.Map>>;


namespace monorfs
{

/// <summary>
/// Common rendering routines.
/// </summary>
public static class DrawUtils
{
	/// <summary>
	/// Render a path specified in global coordinates.
	/// </summary>
	/// <param name="graphics">MonoGame graphic context.</param>
	/// <param name="trajectory">Path to render.</param>
	/// <param name="color">Line color.</param>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public static void DrawTrajectory<PoseT>(GraphicsDevice graphics, TimedState trajectory,
	                                  Color color, double[][] camera)
		where PoseT : IPose<PoseT>, new()
	{
		double[][] vertices = new double[trajectory.Count][];

		PoseT dummy = new PoseT();

		for (int i = 0; i < trajectory.Count; i++) {
			PoseT w  = dummy.FromState(trajectory[i].Item2);
			vertices[i] = camera.TransformH(w.Location);
		}

		graphics.DrawUser2DPolygon(vertices, 0.06f, color, false);
	}
}
}

