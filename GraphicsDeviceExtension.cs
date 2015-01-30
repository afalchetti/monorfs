// GraphicsDeviceEntension.cs
// Utility  methods to deal with graphics
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

using System;
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
	/// <param name="rect">Output array of points (rectangle).
	/// It must have a capacity of at least 4.</param>
	public static void ThickLine(Vector3 x0, Vector3 x1, float thickness, out Vector3[] rect)
	{
		Vector3 direction = x1 - x0;

		if (direction.LengthSquared() > 1e-12) {
			direction.Normalize();
		}
		else {
			direction = Vector3.Zero;
		}

		Vector3 normal;
		normal = new Vector3(direction.Y, -direction.X, 0);

		float halfthick = thickness / 2;

		rect = new Vector3[4];

		rect[0] = x0 - halfthick * normal;
		rect[1] = x0 + halfthick * normal;
		rect[2] = x1 - halfthick * normal;
		rect[3] = x1 + halfthick * normal;
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
	public static void DrawUser2DPolygon(this GraphicsDevice graphics, VertexPositionColor[] vertices,
		float thickness, Color color, bool close = false)
	{
		if (vertices.Length < 2) {
			return;
		}

		int outlinelength  = close ? 4 * vertices.Length + 2 : 4 * (vertices.Length - 1);
		VertexPositionColor[] outline = new VertexPositionColor[outlinelength];

		Vector3[] line = new Vector3[4];
		Vector3 x1     = vertices[0].Position;
		Vector3 x2     = vertices[1].Position;
		int     h      = 0;

		// Z must be one, homogeneous coordinates
		x2   = vertices[0].Position;
		x2.Z = 1;

		for (int i = 0; i < vertices.Length - 1; i++) {
			x1   = x2;
			x2   = vertices[i + 1].Position;
			x2.Z = 1;
			ThickLine(x1, x2, thickness, out line);

			outline[h++] = new VertexPositionColor(line[0], color);
			outline[h++] = new VertexPositionColor(line[1], color);
			outline[h++] = new VertexPositionColor(line[2], color);
			outline[h++] = new VertexPositionColor(line[3], color);
		}

		if (close) {
			x1   = x2;
			x2   = vertices[0].Position;
			x2.Z = 1;
			ThickLine(x1, x2, thickness, out line);

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

