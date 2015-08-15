// GraphicsDeviceExtension.cs
// Utility  methods to deal with graphics
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

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace monorfs
{
/// <summary>
/// 2D rendering extension procedures.
/// </summary>
public static class GraphicsDeviceExtension
{
	/// <summary>
	/// Generate a thick line (rotated rectangle) between two points.
	/// The points must be in homogeneous coordinates and the last
	/// coordinate value must be one.
	/// </summary>
	/// <param name="x0">First point.</param>
	/// <param name="x1">Second point.</param>
	/// <param name="thickness">Line thickness.</param>
	/// <returns>Array of points (rectangle).</returns>
	public static Vector3[] ThickLine(Vector3 x0, Vector3 x1, float thickness)
	{
		Vector3 direction = x1 - x0;

		if (direction.LengthSquared() > 1e-12) {
			direction.Normalize();
		}
		else {
			direction = Vector3.Zero;
		}

		Vector3 normal    = new Vector3(direction.Y, -direction.X, 0);
		float   halfthick = thickness / 2;

		Vector3[] rect = new Vector3[4];

		// "- halftick" => do it on the outside (not in the middle)
		rect[0] = x0 + (-halfthick - halfthick) * normal;
		rect[1] = x0 + ( halfthick - halfthick) * normal;
		rect[2] = x1 + (-halfthick - halfthick) * normal;
		rect[3] = x1 + ( halfthick - halfthick) * normal;

		return rect;
	}

	/// <summary>
	/// Draws a 2D thick line polygon on the specified GraphicsDevice.
	/// </summary>
	/// <param name="graphics">Rendering device.</param>
	/// <param name="vertices">Thin polygon description.</param>
	/// <param name="thickness">Line thickness.</param>
	/// <param name="color">Stroke color.</param>
	/// <param name="close">If set to <c>true</c>,
	/// join the last vertex and the first.</param>
	public static void DrawUser2DPolygon(this GraphicsDevice graphics, double[][] vertices,
		float thickness, Color color, bool close = false)
	{
		if (vertices.Length < 2) {
			return;
		}

		int outlinelength  = close ? 4 * vertices.Length + 2 : 4 * (vertices.Length - 1);
		VertexPositionColor[] outline = new VertexPositionColor[outlinelength];

		Vector3[] line = new Vector3[4];
		Vector3 x1     = vertices[0].ToVector3();
		Vector3 x2     = vertices[0].ToVector3();
		int     h      = 0;

		for (int i = 0; i < vertices.Length - 1; i++) {
			x1   = x2;
			x2   = vertices[i + 1].ToVector3();
			line = ThickLine(x1, x2, thickness);

			outline[h++] = new VertexPositionColor(line[0], color);
			outline[h++] = new VertexPositionColor(line[1], color);
			outline[h++] = new VertexPositionColor(line[2], color);
			outline[h++] = new VertexPositionColor(line[3], color);
		}

		if (close) {
			x1   = x2;
			x2   = vertices[0].ToVector3();
			line = ThickLine(x1, x2, thickness);

			outline[h++] = new VertexPositionColor(line[0], color);
			outline[h++] = new VertexPositionColor(line[1], color);
			outline[h++] = new VertexPositionColor(line[2], color);
			outline[h++] = new VertexPositionColor(line[3], color);

			outline[h++] = outline[0];
			outline[h++] = outline[1];
		}

		graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, outline, 0, outline.Length - 2);
	}
}
}

