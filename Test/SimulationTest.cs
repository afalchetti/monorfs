// SimulationTest.cs
// Simulation and related classes unit tests.
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using NUnit.Framework;
using AE = monorfs.ArrayExtensions;

// "D:\applications\NUnit 2.6.3\bin\nunit.exe" "$(TargetPath)"
namespace monorfs.Test
{
/// <summary>
/// Simulator and related classes unit tests.
/// </summary>
[TestFixture]
class SimulationTest
{
	private Simulation               simulation;
	private Vehicle                  explorer;
	private Navigator                navigator;
	private GameTime                 lastnavigationupdate;
	private CircularBuffer<double[]> commands;

	private readonly Action nop = () => {};

	[TestFixtureSetUp]
	public void globalsetup()
	{
		if (!KinectVehicle.Initialize()) {
			KinectVehicle.Shutdown();
			Environment.Exit(2);
		}
	}

	[SetUp]
	public void setup() {
		try {
			simulation = new Simulation("test.world", "test.in", 1, false, true, false);
			commands   = simulation.Commands;
			explorer   = simulation.Explorer;
			navigator  = simulation.Navigator;
		}
		catch (Exception) {
			if (simulation != null) {
				simulation.Dispose();
			}

			throw;
		}
	}

	[TearDown]
	public void teardown()
	{
		simulation.Dispose();
	}

	[TestFixtureTearDown]
	public void globalteardown()
	{
		KinectVehicle.Shutdown();
	}
	
	/// <summary>
	/// Simplified and instrumentalized mock update step to allow simulating
	/// the simulation update step (the important part of the class).
	/// </summary>
	public void Update(GameTime time, bool forceslam, bool forcemapping, Action updatehook, Action measurehook, Action slamhook)
	{
		double[] autocmd = commands.Next();
		double   ds      = autocmd[0];
		double   dyaw    = autocmd[1];
		double   dpitch  = autocmd[2];
		double   droll   = autocmd[3];
		double   dcam    = autocmd[4];

		explorer .Update(time, 0, 0, ds, dyaw, dpitch, droll);
		navigator.Update(time, 0, 0, ds, dyaw, dpitch, droll);
		updatehook();

		if (time.TotalGameTime - lastnavigationupdate.TotalGameTime >= simulation.MeasureElapsed) {
			List<double[]> measurements = explorer.Measure();
			measurehook();
			
			navigator.SlamUpdate(time, measurements, true, true, true);
			slamhook();

			lastnavigationupdate = new GameTime(time.TotalGameTime, time.ElapsedGameTime);
		}
	}

	public void UpdateLoop(int loopcount, Action updatehook, Action measurehook, Action slamhook, Action iterwork)
	{
		GameTime time        = new GameTime(new TimeSpan(0), new TimeSpan(0));
		lastnavigationupdate = new GameTime(new TimeSpan(0), new TimeSpan(0));

		for (int i = 0; i < loopcount; i++) {
			Update(time, false, false, updatehook, measurehook, slamhook);
			iterwork();
			time = new GameTime(time.TotalGameTime.Add(simulation.FrameElapsed), simulation.FrameElapsed);
		}
	}

	public void UpdateLoop(int loopcount, Action updatehook, Action measurehook, Action slamhook, Action iterwork,
	                       Func<int, bool> forceslam, Func<int, bool> forcemapping)
	{
		GameTime time = new GameTime(new TimeSpan(0), new TimeSpan(0));

		for (int i = 0; i < loopcount; i++) {
			Update(time, forceslam(i), forcemapping(i), updatehook, measurehook, slamhook);
			iterwork();
			time = new GameTime(time.TotalGameTime.Add(simulation.FrameElapsed), simulation.FrameElapsed);
		}
	}

	[Test]
	public void perfectparticle()
	{
		UpdateLoop(300,
		() =>
		{
			navigator.VehicleParticles[0].State = explorer.State;
		},
		nop, nop,
		() =>
		{
			string message = "best particle: " + navigator.BestParticle + "\n" +
			                 "real vehicle:  " + string.Join(", ", Array.ConvertAll(explorer.State,                                           s => s.ToString("F2"))) + "\n" +
			                 "best estimate: " + string.Join(", ", Array.ConvertAll(navigator.VehicleParticles[navigator.BestParticle].State, s => s.ToString("F2"))) + "\n" +
			                 "real weight:   " + navigator.VehicleWeights[0] + "\n" +
			                 "best weight:   " + navigator.VehicleWeights[navigator.BestParticle] + "\n";

			Assert.AreEqual(0, navigator.BestParticle, message);
			Assert.IsTrue(explorer.State.SequenceEqual(navigator.VehicleParticles[navigator.BestParticle].State)/*, message*/);
		});
	}

	[Test]
	public void resample()
	{
		SimulatedVehicle   refvehicle = new SimulatedVehicle(new double[3] {0, 0, 0}, 0, new double[] {0, 0, 0}, new List<double[]>());
		SimulatedVehicle[] particles  = new SimulatedVehicle[5];
		
		particles[0] = new SimulatedVehicle(refvehicle); particles[0].State = new double[7] {0, 0, 0, 0, 0, 0, 0};
		particles[1] = new SimulatedVehicle(refvehicle); particles[1].State = new double[7] {1, 1, 1, 1, 1, 1, 1};
		particles[2] = new SimulatedVehicle(refvehicle); particles[2].State = new double[7] {2, 2, 2, 2, 2, 2, 2};
		particles[3] = new SimulatedVehicle(refvehicle); particles[3].State = new double[7] {3, 3, 3, 3, 3, 3, 3};
		particles[4] = new SimulatedVehicle(refvehicle); particles[4].State = new double[7] {4, 4, 4, 4, 4, 4, 4};

		int iterations = 10000;
		int count0     = 0;
		int count3     = 0;

		for (int i = 0; i < iterations; i++) {
			navigator.CollapseParticles(5);
			navigator.VehicleParticles[0] = particles[0];
			navigator.VehicleParticles[1] = particles[1];
			navigator.VehicleParticles[2] = particles[2];
			navigator.VehicleParticles[3] = particles[3];
			navigator.VehicleParticles[4] = particles[4];
			navigator.VehicleWeights  [0] = 0.11;
			navigator.VehicleWeights  [1] = 0.28;
			navigator.VehicleWeights  [2] = 0.3;
			navigator.VehicleWeights  [3] = 0.01;
			navigator.VehicleWeights  [4] = 0.3;
			
			navigator.ResampleParticles();
			//Console.WriteLine(string.Join(", ", Array.ConvertAll(navigator.VehicleParticles, p => p.X.ToString("F0"))));

			// these three have probabilities higher than 0.2 so they must always be in the resampled particles
			Assert.IsTrue(navigator.VehicleParticles.Any(x => particles[1].State.SequenceEqual(x.State)));
			Assert.IsTrue(navigator.VehicleParticles.Any(x => particles[2].State.SequenceEqual(x.State)));
			Assert.IsTrue(navigator.VehicleParticles.Any(x => particles[4].State.SequenceEqual(x.State)));
			
			count0 += navigator.VehicleParticles.Any(x => particles[0].State.SequenceEqual(x.State)) ? 1 : 0;
			count3 += navigator.VehicleParticles.Any(x => particles[3].State.SequenceEqual(x.State)) ? 1 : 0;
		}
		
		Console.WriteLine("count0: " + count0 + " / " + iterations + " -> " + ((float) count0 / iterations).ToString("F2") + "; expected = 0.55");
		Console.WriteLine("count3: " + count3 + " / " + iterations + " -> " + ((float) count3 / iterations).ToString("F2") + "; expected = 0.05");
		
		// these can't be in every resample; sometime they need to be missed
		Assert.Less(count0, iterations);
		Assert.Less(count3, iterations);
	}
}
}
