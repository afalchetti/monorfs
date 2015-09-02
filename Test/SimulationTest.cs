// SimulationTest.cs
// Simulation and related classes unit tests
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
using System.Linq;

using Microsoft.Xna.Framework;
using NUnit.Framework;

using AE = monorfs.ArrayExtensions;

namespace monorfs.Test
{
/// <summary>
/// Simulator and related classes unit tests.
/// </summary>
[TestFixture]
class SimulationTest
{
	private Simulation     simulation;
	private Vehicle        explorer;
	private PHDNavigator   navigator;
	private GameTime       lastnavigationupdate;
	private List<double[]> commands;
	private int            commandindex;

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
			simulation = Simulation.FromFiles("test.world", "test.in", 1, VehicleType.Simulation, NavigationAlgorithm.PHD, false, false);
			commands   = simulation.Commands;
			explorer   = simulation.Explorer;
			navigator  = simulation.Navigator as PHDNavigator;
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
	public void Update(GameTime time, Action updatehook, Action measurehook, Action slamhook)
	{
		double[] autocmd = commands[commandindex];
		double   ds      = autocmd[0];
		double   dyaw    = autocmd[1];
		double   dpitch  = autocmd[2];
		double   droll   = autocmd[3];

		commandindex++;

		if (commandindex >= commands.Count) {
			commandindex = 0;
		}

		explorer .Update(time, 0, 0, ds, dyaw, dpitch, droll);
		navigator.Update(time, 0, 0, ds, dyaw, dpitch, droll);
		updatehook();

		if (time.TotalGameTime - lastnavigationupdate.TotalGameTime >= Simulation.MeasureElapsed) {
			List<double[]> measurements = explorer.Measure();
			measurehook();
			
			navigator.SlamUpdate(time, measurements);
			slamhook();

			lastnavigationupdate = new GameTime(time.TotalGameTime, time.ElapsedGameTime);
		}
	}

	public void UpdateLoop(int loopcount, Action updatehook, Action measurehook, Action slamhook, Action iterwork)
	{
		GameTime time        = new GameTime(new TimeSpan(0), new TimeSpan(0));
		lastnavigationupdate = new GameTime(new TimeSpan(0), new TimeSpan(0));

		for (int i = 0; i < loopcount; i++) {
			Update(time, updatehook, measurehook, slamhook);
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

			// these three have probabilities higher than 0.2 so they must always be in the resampled particles
			Assert.IsTrue(navigator.VehicleParticles.Any(x => particles[1].State.SequenceEqual(x.State)));
			Assert.IsTrue(navigator.VehicleParticles.Any(x => particles[2].State.SequenceEqual(x.State)));
			Assert.IsTrue(navigator.VehicleParticles.Any(x => particles[4].State.SequenceEqual(x.State)));
			
			count0 += navigator.VehicleParticles.Any(x => particles[0].State.SequenceEqual(x.State)) ? 1 : 0;
			count3 += navigator.VehicleParticles.Any(x => particles[3].State.SequenceEqual(x.State)) ? 1 : 0;
		}

		// these can't be in every resample; sometime they need to be missed
		Assert.Less(count0, iterations);
		Assert.Less(count3, iterations);
	}
}
}
