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
using System;

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
	/// Draw a 2D thick line polygon on the specified GraphicsDevice.
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

	/// <summary>
	/// Draw a rectangular grid of vertices as a mesh.
	/// </summary>
	/// <param name="graphics">Rendering device.</param>
	/// <param name="grid">Grid locations.</param>
	/// <param name="color">Mesh color.</param>
	/// <param name="wireframe">If true, instead of rendering a closed surface, render
	/// the edges of all the triangles involved.</param>
	public static void DrawGrid(this GraphicsDevice graphics, Vector3[][] grid,
	                            Color color, bool wireframe = false)
	{
		if (grid.Length == 0 || grid[0].Length == 0) {
			return;
		}

		var vertices = new VertexPositionColor[grid.Length * grid[0].Length];

		float zmin = float.MaxValue;
		float zmax = float.MinValue;
			
		for (int k = 0; k < grid[0].Length; k++) {
		for (int i = 0; i < grid   .Length; i++) {
			if (zmin > grid[i][k].Z) {
				zmin = grid[i][k].Z;
			}
			if (zmax < grid[i][k].Z) {
				zmax = grid[i][k].Z;
			}
		}
		}

		float zrange = zmax - zmin;

		// expand the range a little, so nothing is completely opaque or transparent
		zmin = zmin - 0.1f * zrange;
		zmax = zmax + 0.1f * zrange;

		float zalpha = (zmin == zmax) ? 1 : 1 / (zmax - zmin);

		int h = 0;
		for (int k = 0; k < grid[0].Length; k++) {
		for (int i = 0; i < grid   .Length; i++) {
			vertices[h++] =
				new VertexPositionColor(grid[i][k], Color.Multiply(color, zalpha * (grid[i][k].Z - zmin)));
		}
		}

		int   stride  = grid.Length;
		int   current = 0;
		int[] indices = new int[2 * grid.Length * (grid[0].Length - 1)];

		// set indices
		h = 0;
		for (int k = 0; k < grid[0].Length - 1; k++) {
			int revstride = (k % 2 == 0) ? stride - 1: stride + 1;

			for (int i = 0; i < grid.Length; i++) {
				indices[h++] = current;
				current     += stride;

				indices[h++] = current;
				current     -= revstride;
			}

			current += revstride;
		}

		graphics.DrawUserIndexedPrimitives((wireframe) ? PrimitiveType.LineStrip : PrimitiveType.TriangleStrip,
		                                   vertices, 0, vertices.Length,
			indices,  0, (wireframe) ? indices.Length - 1 : indices.Length - 2);
	}
}
}

