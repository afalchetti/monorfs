// Program.cs
// Main entry point to the program
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using NDesk.Options;

namespace monorfs
{
/// <summary>
/// Main program class.
/// </summary>
public class Program
{
	/// <summary>
	/// Save a 2d image into a stream as a PNG file.
	/// </summary>
	/// <param name="image">Image to be saved.</param>
	/// <param name="file">Output filename.</param>
	public static void SaveAsPng(Texture2D image, string file)
	{
		GraphicsDevice  graphics = new GraphicsDevice(GraphicsAdapter.DefaultAdapter,
		                                              GraphicsProfile.HiDef,
		                                              new PresentationParameters());
		RenderTarget2D halfsize = new RenderTarget2D(graphics, image.Width / 2, image.Height / 2,
		                                              false, SurfaceFormat.Color, DepthFormat.None,
		                                              0, RenderTargetUsage.DiscardContents);
		SpriteBatch sb = new SpriteBatch(graphics);
		
		graphics.SetRenderTarget(halfsize);
		graphics.Clear(Color.Black);

		sb.Begin(SpriteSortMode.Immediate, BlendState.NonPremultiplied, SamplerState.LinearClamp,
				 DepthStencilState.None, RasterizerState.CullNone);
		sb.Draw(image, new Rectangle(0, 0, halfsize.Width, halfsize.Height), Color.White);
		sb.End();

		byte[] data = new byte[4 * halfsize.Width * halfsize.Height];
		halfsize.GetData(data);

		// the internal format is ABGR but this isn't supported by S.Drawing.Bitmap
		// so this makes it ARGB. Note that the bytes are reversed (little endianness)
		for (int i = 0; i < data.Length; i += 4) {
			byte temp   = data[i + 0];
			data[i + 0] = data[i + 2];
			data[i + 2] = temp;
			data[i + 3] = 255;
		}
		
		using (var stream = new MemoryStream(data)) {
		using (var bitmap = new System.Drawing.Bitmap(halfsize.Width, halfsize.Height, System.Drawing.Imaging.PixelFormat.Format32bppArgb)) {
			var bitmapdata = bitmap.LockBits(new System.Drawing.Rectangle(0, 0, bitmap.Width, bitmap.Height),
			                                 System.Drawing.Imaging.ImageLockMode.WriteOnly, bitmap.PixelFormat);

			Marshal.Copy(data, 0, bitmapdata.Scan0, data.Length);

			bitmap.UnlockBits(bitmapdata);

			bitmap.Save(file, System.Drawing.Imaging.ImageFormat.Png);
		}
		}
	}

	/// <summary>
	/// Show a description of the program and its parameters to the user.
	/// </summary>
	private static void ShowHelp(OptionSet options)
	{
		Console.WriteLine("Usage: monorfs ");
		Console.WriteLine("Showcase a Rao-Blackwellized PHD SLAM navigator.");
		Console.WriteLine("If no options are specified a simulation of the full slam algorithm is run with one particle.");
		Console.WriteLine();
		Console.WriteLine("Options:");
		options.WriteOptionDescriptions(Console.Out);

	}

	/// <summary>
	/// Primary entry point.
	/// </summary>
	/// <param name="args">Command line arguments.</param>
	//[STAThread]
	public static void Main(string[] args)
	{
		string scenefile     = "map.world";
		string commandfile   = "";
		int    particlecount = 1;
		bool   onlymapping   = false;
		bool   simulate      = false;
		bool   showhelp      = false;

		OptionSet options = new OptionSet() {
			{ "f|file=",      "Scene description file. Simulated, recorded or device id.",    f       => scenefile     = f },
			{ "c|command=",   "Auto-command file (simulates user input).",                    c       => commandfile   = c },
			{ "p|particles=", "Number of particles used for the RB-PHD",                      (int p) => particlecount = p },
			{ "m|onlymap",    "Only do mapping, assuming known localization.",                m       => onlymapping   = m != null },
			{ "s|simulate",   "Generate an artificial simulation instead of using a sensor.", s       => simulate      = s != null },
			{ "h|help",       "Show this message and exit",                                   h       => showhelp      = h != null }
		};

		try {
			options.Parse(args);
		}
		catch (OptionException e) {
			Console.Write("monorfs: ");
			Console.WriteLine(e.Message);
			Console.WriteLine("Try monorfs --help for more information.");
			Environment.Exit(1);
		}

		if (showhelp) {
			ShowHelp(options);
			return;
		}

		if (!KinectVehicle.Initialize()) {
			KinectVehicle.Shutdown();
			Environment.Exit(2);
		}

		using (Simulation sim = new Simulation(scenefile, commandfile, particlecount, onlymapping, simulate)) {
			sim.Run();
		
			File.WriteAllText("trajectories.out", sim.SerializedTrajectories);
			File.WriteAllText("maps.out",         sim.SerializedMaps);

			SaveAsPng(sim.SceneBuffer, "final.png");
		}

		KinectVehicle.Shutdown();
	}
}
}
