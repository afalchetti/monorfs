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

using AE         = monorfs.ArrayExtensions;

using TimedState       = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using Simulation       = monorfs.Simulation<monorfs.PRM3DMeasurer, monorfs.Pose3D, monorfs.PixelRangeMeasurement>;
using PHDNavigator     = monorfs.PHDNavigator<monorfs.PRM3DMeasurer, monorfs.Pose3D, monorfs.PixelRangeMeasurement>;
using Vehicle          = monorfs.Vehicle<monorfs.PRM3DMeasurer, monorfs.Pose3D, monorfs.PixelRangeMeasurement>;
using SimulatedVehicle = monorfs.SimulatedVehicle<monorfs.PRM3DMeasurer, monorfs.Pose3D, monorfs.PixelRangeMeasurement>;
using TrackVehicle     = monorfs.TrackVehicle<monorfs.PRM3DMeasurer, monorfs.Pose3D, monorfs.PixelRangeMeasurement>;

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
	private int            nparticles;
	private Vehicle        initpose;

	private readonly Action nop = () => {};

	[OneTimeSetUp]
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
			nparticles = 20;
			simulation = Simulation.FromFiles("map.world", "movroom.in", nparticles, VehicleType.Simulation, NavigationAlgorithm.PHD, false, false);
			commands   = simulation.Commands;
			explorer   = simulation.Explorer;
			navigator  = simulation.Navigator as PHDNavigator;
			initpose   = new SimulatedVehicle(simulation.Explorer);
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

	[OneTimeTearDown]
	public void globalteardown()
	{
		KinectVehicle.Shutdown();
	}
	
	/// <summary>
	/// Simplified and instrumentalized mock update step to allow simulating
	/// the simulation update step (the important part of the class).
	/// </summary>
	public void Update(GameTime time, Action updatehook,
	                   Action measurehook, Action slamhook)
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

		explorer .Update(time, new double[6] {0, 0, ds, dpitch, dyaw, droll});
		navigator.Update(time, new double[6] {0, 0, ds, dpitch, dyaw, droll});
		updatehook();

		if (time.TotalGameTime - lastnavigationupdate.TotalGameTime >= Simulation.MeasureElapsed) {
			List<PixelRangeMeasurement> measurements = explorer.Measure(time);
			measurehook();

			navigator.SlamUpdate(time, measurements);
			slamhook();

			lastnavigationupdate = new GameTime(time.TotalGameTime, time.ElapsedGameTime);
		}
	}

	public void UpdateLoop(int loopcount, Action updatehook, Action measurehook,
	                       Action slamhook, Action iterwork)
	{
		GameTime time        = new GameTime(new TimeSpan(0), new TimeSpan(0));
		lastnavigationupdate = new GameTime(new TimeSpan(-1, 0, 0), TimeSpan.Zero);

		for (int i = 0; i < loopcount; i++) {
			Update(time, updatehook, measurehook, slamhook);
			iterwork();
			time = new GameTime(time.TotalGameTime.Add(simulation.FrameElapsed), simulation.FrameElapsed);
		}
	}

	[Test]
	public void perfectparticle()
	{
		// the "perfect particle" (the real vehicle) should be very
		// likely (although not exactly perfect)
		int iterations = 20;
		int success    = 0;

		for (int h = 0; h < iterations; h++) {
			Map goodmapmodel = new Map();
			int nloops = 80;
			int iperfect = 0;
			int[] nbest = new int[nparticles];
			bool missed = false;

			explorer = new SimulatedVehicle(initpose);
			navigator = new PHDNavigator(explorer, nparticles, false);

			UpdateLoop(nloops,
			updatehook: () =>
			{
				navigator.VehicleParticles[iperfect].Pose      = new Pose3D(explorer.Pose);
				navigator.VehicleParticles[iperfect].WayPoints = new TimedState(explorer.WayPoints);
				navigator.MapModels[iperfect]                  = goodmapmodel;
			},
			measurehook: nop,
			slamhook: () =>
			{
				bool found = false;
				for (int i = 0; i < nparticles; i++) {
					if (navigator.VehicleParticles[i].Pose.State.SequenceEqual(explorer.Pose.State)) {
						goodmapmodel = navigator.MapModels[i];
						found        = true;
						break;
					}
				}
				
				// if not found it was removed in the resampling stage;
				// this should not happen too often.
				// An exact approach would be to hook in the middle of the SlamUpdate()
				// call, before resampling, to remember the correct model, but that is too invasive;
				// for testing, just give up after the particle is lost and analyze the previous history
				if (!found) {
					missed = true;
				}
				
				if (!missed) {
					nbest[navigator.BestParticle]++;
				}
			},
			iterwork: nop);

			// find out how many rounds was it able to make before the particle got lost
			int rounds = 0;
			for (int i = 0; i < nparticles; i++) {
				rounds += nbest[i];
			}

			Console.WriteLine("rounds = " + rounds);

			bool perfectwins = true;
			for (int i = 0; i < nparticles; i++) {
				if (nbest[iperfect] < nbest[i]) {
					perfectwins = false;
				}
			}

			success += perfectwins ? 1 : 0;
		}

		Console.WriteLine("success rate: " + (double) success / iterations);
		Assert.Greater((double) success / iterations, 0.5);
	}

	[Test]
	public void resample()
	{
		TrackVehicle   refvehicle = new TrackVehicle();
		TrackVehicle[] particles  = new TrackVehicle[5];
		
		particles[0] = new TrackVehicle(refvehicle); particles[0].Pose = new Pose3D(new double[7] {0, 0, 0, 0, 0, 0, 0});
		particles[1] = new TrackVehicle(refvehicle); particles[1].Pose = new Pose3D(new double[7] {1, 1, 1, 1, 1, 1, 1});
		particles[2] = new TrackVehicle(refvehicle); particles[2].Pose = new Pose3D(new double[7] {2, 2, 2, 2, 2, 2, 2});
		particles[3] = new TrackVehicle(refvehicle); particles[3].Pose = new Pose3D(new double[7] {3, 3, 3, 3, 3, 3, 3});
		particles[4] = new TrackVehicle(refvehicle); particles[4].Pose = new Pose3D(new double[7] {4, 4, 4, 4, 4, 4, 4});

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
			navigator.VehicleWeights  [2] = 0.31;
			navigator.VehicleWeights  [3] = 0.01;
			navigator.VehicleWeights  [4] = 0.29;
			
			navigator.ResampleParticles();

			Assert.IsTrue(particles[2].Pose.State.SequenceEqual(navigator.BestEstimate.Pose.State));

			// these three have probabilities higher than 0.2 so they must always be in the resampled particles
			Assert.IsTrue(navigator.VehicleParticles.Any(x => particles[1].Pose.State.SequenceEqual(x.Pose.State)));
			Assert.IsTrue(navigator.VehicleParticles.Any(x => particles[2].Pose.State.SequenceEqual(x.Pose.State)));
			Assert.IsTrue(navigator.VehicleParticles.Any(x => particles[4].Pose.State.SequenceEqual(x.Pose.State)));
			
			count0 += navigator.VehicleParticles.Any(x => particles[0].Pose.State.SequenceEqual(x.Pose.State)) ? 1 : 0;
			count3 += navigator.VehicleParticles.Any(x => particles[3].Pose.State.SequenceEqual(x.Pose.State)) ? 1 : 0;
		}

		// these can't be in every resample; sometime they need to be missed
		Assert.Less(count0, iterations);
		Assert.Less(count3, iterations);
	}
}
}
