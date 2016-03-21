// Map.cs
// Map model as a mixture of gaussians
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
using System.Collections;
using System.Collections.Generic;

using Accord.Math;
using Accord.MachineLearning.Structures;

namespace monorfs
{
/// <summary>
/// Map model as mixture of gaussians.
/// </summary>
public class Map : IMap
{
	/// <summary>
	/// Maximum euclidean distance for a gaussian to be deemed relevant when evaluating the density.
	/// Note that ideally this would be Mahalanobis distance, but that is not scalable; a
	/// conservative euclidean distance should work fine, e.g. the prior measurement 5-sigma distance.
	/// </summary>
	public static double DensityDistanceThreshold { get { return Config.DensityDistanceThreshold; } }

	/// <summary>
	/// Point landmarks and their covariances.
	/// </summary>
	private KDTree<Gaussian> landmarks;

	/// <summary>
	/// The number of landmarks in the map.
	/// </summary>
	public int Count { get { return landmarks.Count; } }

	/// <summary>
	/// Expected number of landmarks in the complete map using the density.
	/// </summary>
	public double ExpectedSize {
		get {
			double expected = 0;

			foreach (Gaussian component in this) {
				expected += component.Weight;
			}

			return expected;
		}
	}

	/// <summary>
	/// Construct an empty map.
	/// </summary>
	public Map()
	{
		landmarks = new KDTree<Gaussian>(3);
	}

	/// <summary>
	/// Copy construct a map from another.
	/// </summary>
	public Map(IMap that)
	{
		landmarks = new KDTree<Gaussian>(3);

		foreach (var landmark in that) {
			landmarks.Add(landmark.Mean, landmark);
		}
	}

	/// <summary>
	/// Remove all landmarks from the map.
	/// </summary>
	public void Clear()
	{
		landmarks.Clear();
	}

	/// <summary>
	/// Add the specified landmark into the map.
	/// </summary>
	/// <param name="landmark">Landmark to be added.</param>
	public void Add(Gaussian landmark)
	{
		landmarks.Add(landmark.Mean, landmark);
	}

	/// <summary>
	/// Find the landmarks that are near to a point in space
	/// using DensityDistanceThreshold as the radius.
	/// </summary>
	/// <param name="point">Search point.</param>
	public Map Near(double[] point)
	{
		return Near(point, DensityDistanceThreshold);
	}

	/// <summary>
	/// Find the landmarks that are near to a point in space.
	/// </summary>
	/// <param name="point">Search point.</param>
	/// <param name="radius">Maximum distance from point to be included in the submap.</param>
	public Map Near(double[] point, double radius)
	{
		Map near = new Map();
		var kdnear = landmarks.Nearest(point, radius);

		foreach (var component in kdnear) {
			near.Add(component.Node.Value);
		}

		return near;
	}

	/// <summary>
	/// Evaluate the map model on a specified location
	/// using DensityDistanceThreshold as the radius.
	/// </summary>
	/// <param name="point">Evaluation location.</param>
	/// <returns>Density on the specified location.</returns>
	public double Evaluate(double[] point)
	{
		return Evaluate(point, DensityDistanceThreshold);
	}

	/// <summary>
	/// Evaluate the map model on a specified location.
	/// </summary>
	/// <param name="point">Evaluation location.</param>
	/// <returns>Density on the specified location.</returns>
	/// <param name="radius">Maximum distance from point to be included in the evaluation.</param>
	public double Evaluate(double[] point, double radius)
	{
		double value = 0;

		/*foreach (var component in Nearest(point, radius)) {
			Gaussian landmark = component.Node.Value;
			value += landmark.Weight * landmark.Evaluate(point);
		}*/

		foreach (var landmark in this) {
			value += landmark.Weight * landmark.Evaluate(point);
		}

		return value;
	}

	/// <summary>
	/// Generate an index for this map.
	/// </summary>
	/// <remarks>
	/// The comparison shall be by reference so to use
	/// this index, the gaussian must come from the same map.
	/// This is important so different but equivalent components
	/// can be distinguished, which may happen, for example, when
	/// default-initializing several landmarks.
	/// Since gaussian are immutable, submaps (e.g. FindAll)
	/// work correctly with these indices.
	/// </remarks>
	public Dictionary<Gaussian, int> Indexify()
	{
		var index = new Dictionary<Gaussian, int>();
		int i     = 0;

		foreach (Gaussian landmark in this) {
			index.Add(landmark, i);
			i++;
		}

		return index;
	}

	/// <summary>
	/// Retrieve the submap of all the landmarks that match a condition.
	/// </summary>
	/// <param name="match">Match condition.</param>
	/// <returns>Filtered submap.</returns>
	public IMap FindAll(Predicate<Gaussian> match)
	{
		Map filtered = new Map();

		foreach (var node in this.landmarks) {
			if (match(node.Value)) {
				filtered.landmarks.Add(node.Value.Mean, node.Value);
			}
		}

		return filtered;
	}

	/// <summary>
	/// Process every landmark and give a list of the results.
	/// </summary>
	/// <param name="converter">Processing routine.</param>
	/// <typeparam name="TOutput">The output type of the processing routine.</typeparam>
	/// <returns>Process results, one entry per landmark.</returns>
	public List<TOutput> ConvertAll<TOutput>(Converter<Gaussian, TOutput> converter)
	{
		List<TOutput> processed = new List<TOutput>();

		foreach (var node in landmarks) {
			processed.Add(converter(node.Value));
		}

		return processed;
	}

	/// <summary>
	/// Express the landmarks in a sequential list.
	/// </summary>
	/// <returns>Landmark list.</returns>
	public List<Gaussian> ToList()
	{
		List<Gaussian> list = new List<Gaussian>(landmarks.Count);

		foreach (var node in landmarks) {
			list.Add(node.Value);
		}

		return list;
	}

	/// <summary>
	/// Get an iterator over every landmarks.
	/// </summary>
	/// <returns>Landmark iterator.</returns>
	public IEnumerator<Gaussian> GetEnumerator()
	{
		foreach (var node in landmarks) {
			yield return node.Value;
		}
	}

	/// <summary>
	/// Get a non-generic iterator.
	/// </summary>
	/// <returns>Non-generic iterator.</returns>
	IEnumerator IEnumerable.GetEnumerator()
	{
		return GetEnumerator();
	}
}
}
