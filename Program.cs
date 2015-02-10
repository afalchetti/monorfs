﻿// Program.cs
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

namespace monorfs
{
/// <summary>
/// Main program class.
/// </summary>
public class Program
{
	/// <summary>
	/// Primary entry point.
	/// </summary>
	/// <param name="args">Command line arguments.</param>
	//[STAThread]
	static void Main(string[] args)
	{
		Simulation sim = new Simulation("map.world", "movements.in");

		sim.Run();
		
		File.WriteAllText("trajectories.out", sim.SerializedTrajectories);
		File.WriteAllText("maps.out",         sim.SerializedMaps);
	}
}
}
