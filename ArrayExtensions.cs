// ArrayExtensions.cs
// Utility functions for the Array class
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
