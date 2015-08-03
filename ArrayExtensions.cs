// ArrayExtensions.cs
// Utility functions for the Array class
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

namespace monorfs
{
/// <summary>
/// Useful extensions for the Array class.
/// </summary>
public static class ArrayExtensions
{
	/// <summary>
	/// Fill an array with a specified value.
	/// </summary>
	/// <typeparam name="T">Array type.</typeparam>
	/// <param name="array">Array.</param>
	/// <param name="value">Filling value.</param>
	public static void Fill<T>(T[] array, T value)
	{
		for (int i = 0; i < array.Length; i++) {
			array[i] = value;
		}
	}
	
	/// <summary>
	/// Fill an array evaluating a function on every index.
	/// </summary>
	/// <typeparam name="T">Array type.</typeparam>
	/// <param name="array">Array.</param>
	/// <param name="filler">Filling function.</param>
	public static void Fill<T>(T[] array, Func<int, T> filler)
	{
		for (int i = 0; i < array.Length; i++) {
			array[i] = filler(i);
		}
	}
	
	/// <summary>
	/// Apply a function to every entry in the array.
	/// </summary>
	/// <typeparam name="T">Array type.</typeparam>
	/// <param name="array">Array.</param>
	/// <param name="map">Mapping function.</param>
	public static void Apply<T>(T[] array, Func<T, T> map)
	{
		for (int i = 0; i < array.Length; i++) {
			array[i] = map(array[i]);
		}
	}

	/// <summary>
	/// Get the minimum value of the array.
	/// </summary>
	/// <typeparam name="T">Array type.</typeparam>
	/// <param name="array">Array.</param>
	/// <returns>Minimum value index. If the array is empty, it returns -1.</returns>
	public static int Min<T>(T[] array) where T : IComparable
	{
		if (array.Length == 0) {
			return -1;
		}

		T   min  = array[0];
		int imin = 0;

		for (int i = 1; i < array.Length; i++) {
			if (array[i].CompareTo(min) < 0) {
				min  = array[i];
				imin = i;
			}
		}

		return imin;
	}
}
}
