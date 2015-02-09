// CircularBuffer.cs
// Circular indexed array.
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
/// Circular indexed array.
/// </summary>
class CircularBuffer<T>
{
	/// <summary>
	/// Internal values.
	/// </summary>
	private T[] data;

	/// <summary>
	/// Internal array index.
	/// </summary>
	private int i;

	/// <summary>
	/// Number of elements in the buffer.
	/// </summary>
	public int Size { get; private set; }

	/// <summary>
	/// Construct a CircularBuffer with a specified buffer size.
	/// Initially values are default values.
	/// </summary>
	/// <param name="size">Number of elements in the buffer.</param>
	public CircularBuffer(int size)
	{
		Size = size;
		data = new T[size];
		i    = 0;
	}

	/// <summary>
	/// Get the next item and advance the buffer head.
	/// </summary>
	/// <returns>The next item.</returns>
	public T Next()
	{
		T item = data[i++];
		
		if (i >= Size) {
			i = 0;
		}

		return item;
	}

	/// <summary>
	/// Add an item to the buffer and advance the buffer head.
	/// </summary>
	/// <param name="item">New item.</param>
	public void Add(T item)
	{
		data[i++] = item;
		
		if (i >= Size) {
			i = 0;
		}
	}
}
}
