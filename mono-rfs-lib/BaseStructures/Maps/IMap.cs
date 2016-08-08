﻿// Map.cs
// Map model as a mixture of gaussians interface
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

namespace monorfs
{
/// <summary>
/// Map model as mixture of gaussians.
/// </summary>
public interface IMap : IEnumerable<Gaussian>
{
	/// <summary>
	/// The number of landmarks in the map.
	/// </summary>
	int Count { get; }

	/// <summary>
	/// Number of dimensions for each landmark.
	/// </summary>
	int Dimensions { get; }

	/// <summary>
	/// Remove all landmarks from the map.
	/// </summary>
	void Clear();

	/// <summary>
	/// Add the specified landmark into the map.
	/// </summary>
	/// <param name="landmark">Landmark to be added.</param>
	void Add(Gaussian landmark);

	/// <summary>
	/// Evaluate the map model on a specified location.
	/// </summary>
	/// <param name="point">Evaluation location.</param>
	/// <returns>Density on the specified location.</returns>
	double Evaluate(double[] point);

	/// <summary>
	/// Retrieve the submap of all the landmarks that match a condition.
	/// </summary>
	/// <param name="match">Match condition.</param>
	/// <returns>Filtered submap.</returns>
	IMap FindAll(Predicate<Gaussian> match);

	/// <summary>
	/// Process every landmark and give a list of the results.
	/// </summary>
	/// <param name="converter">Processing routine.</param>
	/// <typeparam name="TOutput">The output type of the processing routine.</typeparam>
	/// <returns>Process results, one entry per landmark.</returns>
	List<TOutput> ConvertAll<TOutput>(Converter<Gaussian, TOutput> converter);

	/// <summary>
	/// Express the landmarks in a sequential list.
	/// </summary>
	/// <returns>Landmark list.</returns>
	List<Gaussian> ToList();
}
}
