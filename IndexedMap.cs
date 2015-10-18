// IndexedMap.cs
// Map model as a mixture of gaussians with association indices
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

namespace monorfs
{
/// <summary>
/// Map model as mixture of gaussians with associated indices.
/// </summary>
public class IndexedMap : IMap
{
	/// <summary>
	/// Point landmarks and their covariances.
	/// </summary>
	private List<Gaussian> landmarks;

	/// <summary>
	/// The number of landmarks in the map.
	/// </summary>
	public int Count { get { return landmarks.Count; } }

	/// <summary>
	/// Construct an empty map.
	/// </summary>
	public IndexedMap()
	{
		landmarks = new List<Gaussian>();
	}

	/// <summary>
	/// Construct a empty map of a given capacity.
	/// </summary>
	/// <param name="capacity">Number of landmarks that may added without reallocation.</param>
	public IndexedMap(int capacity)
	{
		landmarks = new List<Gaussian>(capacity);
	}

	/// <summary>
	/// Copy construct a map from another.
	/// </summary>
	public IndexedMap(IMap that)
	{
		landmarks = new List<Gaussian>();

		foreach (var landmark in that) {
			landmarks.Add(landmark);
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
		landmarks.Add(landmark);
	}

	/// <summary>
	/// Get an indexed landmark. This index should be valid in the
	/// same conditions than the index of an IList, i.e. as long
	/// as landmarks with lower indices are not removed from the map.
	/// </summary>
	/// <param name="i">Index to be accesed.</param>
	public Gaussian this[int i]
	{
		get { return landmarks[i]; }
		set {landmarks[i] = value; }
	}

	/// <summary>
	/// Remove the landmark at the specified index from the map.
	/// </summary>
	/// <param name="index">Index of the landmark to be removed.</param>
	public void RemoveAt(int index)
	{
		landmarks.RemoveAt(index);
	}

	/// <summary>
	/// Evaluate the map model on a specified location.
	/// </summary>
	/// <param name="point">Evaluation location.</param>
	/// <returns>Density on the specified location.</returns>
	public double Evaluate(double[] point)
	{
		double value = 0;

		foreach (Gaussian component in this) {
			value += component.Weight * component.Evaluate(point);
		}

		return value;
	}

	/// <summary>
	/// Retrieve the submap of all the landmarks that match a condition.
	/// </summary>
	/// <param name="match">Match condition.</param>
	/// <returns>Filtered submap.</returns>
	public IMap FindAll(Predicate<Gaussian> match)
	{
		IndexedMap filtered = new IndexedMap();
		filtered.landmarks = this.landmarks.FindAll(match);

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
		return landmarks.ConvertAll(converter);
	}

	/// <summary>
	/// Express the landmarks in a sequential list.
	/// </summary>
	/// <returns>Landmark list.</returns>
	public List<Gaussian> ToList()
	{
		return new List<Gaussian>(landmarks);
	}

	/// <summary>
	/// Get an iterator over every landmarks.
	/// </summary>
	/// <returns>Landmark iterator.</returns>
	public IEnumerator<Gaussian> GetEnumerator()
	{
		foreach (var landmark in landmarks) {
			yield return landmark;
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
