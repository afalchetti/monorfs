// CircularBuffer.cs
// Circular indexed array
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
using System.IO;

namespace monorfs
{
/// <summary>
/// Circular indexed array.
/// </summary>
public class CircularBuffer<T>
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
