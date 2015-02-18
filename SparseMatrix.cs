// SparseMatrix.cs
// Sparse matrix definition
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

namespace monorfs
{
/// <summary>
/// Sparse Matrix. Useful representation when most of the entries are null.
/// </summary>
public class SparseMatrix : IEnumerable<SparseItem>
{
	/// <summary>
	/// Internal indexed data representation.
	/// </summary>
	/// <remarks>The matrix order is column-major.
	/// An invariant of this class is that no data row is empty.
	/// If an operation induces a row to become empty, it must be removed.
	/// Note that the matrix can have empty rows but they are never saved in this variable.</remarks>
	private Dictionary<int, Dictionary<int, double>> data;

	/// <summary>
	/// Number of total rows (including null ones).
	/// </summary>
	public int Width { get; private set; }

	/// <summary>
	/// Number of total columns (including null ones).
	/// </summary>
	public int Height { get; private set; }

	/// <summary>
	/// Construct a new SparseMatrix.
	/// </summary>
	public SparseMatrix()
	{
		data   = new Dictionary<int, Dictionary<int, double>>();
		Width  = 0;
		Height = 0;
	}

	/// <summary>
	/// Construct a new SparseMatrix with a given initial width and height.
	/// </summary>
	/// <param name="width">Initial width.</param>
	/// <param name="height">Initial height.</param>
	public SparseMatrix(int width, int height)
	{
		data   = new Dictionary<int, Dictionary<int, double>>();
		Width  = width;
		Height = height;
	}

	/// <summary>
	/// Construct a SparseMatrix from another (shallow, but double is immutate).
	/// </summary>
	/// <param name="that"></param>
	public SparseMatrix(SparseMatrix that)
	{
		this.data = new Dictionary<int, Dictionary<int, double>>();

		foreach (SparseItem item in that) {
			this[item.I, item.K] = item.Value;
		}

		this.Width = that.Width;
		this.Height = that.Height;
	}

	/// <summary>
	/// Get or set an entry on the matrix.
	/// </summary>
	/// <param name="i">Row index.</param>
	/// <param name="k">Colummn index.</param>
	/// <returns>Entry value.</returns>
	public double this[int i, int k]
	{
		get
		{
			Dictionary<int, double> row;
			double value;
			return (data.TryGetValue(i, out row) && row.TryGetValue(k, out value)) ? value : 0;
		}

		set
		{
			Dictionary<int, double> row;
			if (data.TryGetValue(i, out row)) {
				row[k] = value;
			}
			else {
				row     = new Dictionary<int,double>();
				row [k] = value;
				data[i] = row;
			}
			
			Height = Math.Max(Height, i + 1);
			Width  = Math.Max(Width,  k + 1);
		}
	}

	/// <summary>
	/// Number of non-null entries.
	/// </summary>
	public int Count {
		get
		{
			int count = 0;

			foreach (var row in data) {
				count += row.Value.Count;
			}

			return count;
		}
	}

	/// <summary>
	/// Returns some entry in the matrix.
	/// It could be any of them.
	/// </summary>
	public SparseItem Any
	{
		get
		{
			if  (data.Count == 0) {
				return null;
			}

			// note that rows can't be empty (invariant of this class)
			// if they were, the very moment they become empty, they should be removed
			var row  = data.First();
			var item = row.Value.First();

			return new SparseItem(row.Key, item.Key, item.Value);
		}
	}

	/// <summary>
	/// Get a sparse row description.
	/// </summary>
	/// <param name="i">Row index.</param>
	/// <returns>Sparse row description.</returns>
	public Dictionary<int, double> Row(int i)
	{
		Dictionary<int, double> row;
		return (data.TryGetValue(i, out row)) ? row : new Dictionary<int, double>();
	}

	/// <summary>
	/// Get a sparse column description.
	/// </summary>
	/// <param name="k">Column index.</param>
	/// <returns>Sparse column description.</returns>
	public Dictionary<int, double> Column(int k)
	{
		var col = new Dictionary<int, double>();
		
		foreach (var row in data) {
			double value;
			if (row.Value.TryGetValue(k, out value)) {
				col.Add(row.Key, value);
			}
		}

		return col;
	}

	/// <summary>
	/// Get all the columns connected to a given row.
	/// </summary>
	/// <param name="i">Row index.</param>
	/// <returns>Connected columns' indices.</returns>
	public int[] RowNeighbours(int i)
	{
		var  row         = Row(i);
		int[] neighbours = new int[row.Count];
		row.Keys.CopyTo(neighbours, 0);

		return neighbours;
	}

	/// <summary>
	/// Get all the rows connected to a given column.
	/// </summary>
	/// <param name="k">Column index.</param>
	/// <returns>Connected rows' indices.</returns>
	public int[] ColumnNeighbours(int k)
	{
		var  col         = Column(k);
		int[] neighbours = new int[col.Count];
		col.Keys.CopyTo(neighbours, 0);

		return neighbours;
	}

	/// <summary>
	/// A dictionary of non-null sparse rows.
	/// </summary>
	public Dictionary<int, Dictionary<int, double>> Rows { get { return data; } }

	/// <summary>
	/// A dictionary of non-null sparse columns.
	/// </summary>
	public Dictionary<int, Dictionary<int, double>> Columns { get { return Transpose().Rows; } }

	/// <summary>
	/// Get the transpose of the matrix.
	/// </summary>
	/// <returns>Matrix transpose.</returns>
	public SparseMatrix Transpose()
	{
		SparseMatrix transpose = new SparseMatrix(this.Height, this.Width);

		foreach (var item in this) {
			transpose[item.K, item.I] = item.Value;
		}

		return transpose;
	}

	/// <summary>
	/// Add every entry of a given row to the matrix.
	/// Any previous colliding entry will be overwritten.
	/// </summary>
	/// <param name="i">Row index where the entries will be added.</param>
	/// <param name="row">New values.</param>
	public void AddRow(int i, Dictionary<int, double> row)
	{
		Dictionary<int, double> introw;
		if (data.TryGetValue(i, out introw)) {
			foreach (var item in row) {
				introw[item.Key] = item.Value;
			}
		}
		else {
			data.Add(i, new Dictionary<int, double>(row));
		}
	}

	/// <summary>
	/// Add every entry of a given column to the matrix.
	/// Any previous colliding entry will be overwritten.
	/// </summary>
	/// <param name="i">Column index where the entries will be added.</param>
	/// <param name="column">New values.</param>
	public void AddColumn(int k, Dictionary<int, double> column)
	{
		foreach (var item in column) {
			Dictionary<int, double> intcol;

			if (data.TryGetValue(item.Key, out intcol)) {
				intcol[k] = item.Value;
			}
			else {
				var newrow = new Dictionary<int,double>();
				newrow[k]  = item.Value;
				data.Add(item.Key, newrow);
			}
		}
	}

	/// <summary>
	/// Remove an entry on the matrix (leave it unspecified == 0)
	/// </summary>
	/// <param name="i">Row index.</param>
	/// <param name="k">Column index.</param>
	/// <returns></returns>
	public bool RemoveAt(int i, int k)
	{
		Dictionary<int, double> row;
		if (data.TryGetValue(i, out row) && row.Remove(k)) {
			if (row.Count == 0) {
				data.Remove(i);
			}

			return true;
		}

		return false;
	}
	
	/// <summary>
	/// Remove an entire row on the matrix (leave it unspecified == 0)
	/// </summary>
	/// <param name="i">Row index.</param>
	public bool RemoveRow(int i)
	{
		return data.Remove(i);
	}
	
	/// <summary>
	/// Remove an entire column on the matrix (leave it unspecified == 0)
	/// </summary>
	/// <param name="k">Column index.</param>
	public bool RemoveColumn(int k)
	{
		bool      removed  = false;
		List<int> toremove = new List<int>();

		foreach (var row in data) {
			removed |= row.Value.Remove(k);

			if (row.Value.Count == 0) {
				toremove.Add(row.Key);
			}
		}

		foreach (var i in toremove) {
			data.Remove(i);
		}

		return removed;
	}
	
	/// <summary>
	/// Remove a number of rows on the matrix (leave it unspecified == 0)
	/// </summary>
	/// <param name="indices">Row indices.</param>
	public bool RemoveRows(int[] indices)
	{
		bool removed = false;

		foreach (int i in indices) {
			removed |= data.Remove(i);
		}

		return removed;
	}
	
	/// <summary>
	/// Remove a number of columns on the matrix (leave it unspecified == 0)
	/// </summary>
	/// <param name="indices">Column indices.</param>
	public bool RemoveColumns(int[] indices)
	{
		bool      removed  = false;
		List<int> toremove = new List<int>();

		foreach (var row in data)    {
			foreach (int k   in indices) {
				removed |= row.Value.Remove(k);
			}

			if (row.Value.Count == 0) {
				toremove.Add(row.Key);
			}
		}

		foreach (var i in toremove) {
			data.Remove(i);
		}

		return removed;
	}

	/// <summary>
	/// Apply a function iteratively on every item of a row (for each row)
	/// and obtain the accumulated results.
	/// </summary>
	/// <param name="reducer">Applied function. The first parameter is the cumulative reuslt,
	/// the second parameter is the new entry.</param>
	/// <param name="initvalue">Initial value for the accumulation.</param>
	/// <returns>Sparse list of the resultl of applying the recursive accumulator to each row on the matrix.
	/// Each result is compound of the row index and the accumulator.</returns>
	public Dictionary<int, double> FoldRows(Func<double, double, double> reducer, double initvalue)
	{
		Dictionary<int, double> fold = new Dictionary<int, double>();
		
		foreach (var row in data) {
			double folded = initvalue;
			foreach (var item in row.Value) {
				folded = reducer(folded, item.Value);
			}

			fold[row.Key] = folded;
		}

		return fold;
	}

	/// <summary>
	/// Apply a function iteratively on every item of a column (for each column)
	/// and obtain the accumulated results.
	/// </summary>
	/// <param name="reducer">Applied function. The first parameter is the cumulative reuslt,
	/// the second parameter is the new entry. It must return the new accumulation.</param>
	/// <param name="initvalue">Initial value for the accumulation.</param>
	/// <returns>Sparse list of the resultl of applying the recursive accumulator to each column on the matrix.
	/// Each result is compound of the column index and the accumulator.</returns>
	public Dictionary<int, double> FoldColumns(Func<double, double, double> reducer, double initvalue)
	{
		Dictionary<int, double> fold = new Dictionary<int, double>();
		
		foreach (var row in data) {
		foreach (var item in row.Value) {
			fold[item.Key] = (fold.ContainsKey(item.Key)) ? reducer(fold[item.Key], item.Value) : initvalue;
		}
		}

		return fold;
	}

	/// <summary>
	/// Apply a mapping to every entry in the matrix.
	/// </summary>
	/// <param name="map">Applied mapping.</param>
	public void Apply(Func<double, double> map)
	{
		foreach (var row in data) {
		foreach (var item in row.Value) {
			row.Value[item.Key] = map(item.Value);
		}
		}
	}

	/// <summary>
	/// Get an iterator over every entry of the matrix.
	/// </summary>
	/// <returns>Matrix iterator.</returns>
	public IEnumerator<SparseItem> GetEnumerator()
	{
		foreach (var row  in data) {
		foreach (var item in row.Value) {
			yield return new SparseItem(row.Key, item.Key, item.Value);
		}
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

/// <summary>
/// Sparse matrix entry data.
/// </summary>
public class SparseItem
{
	public int I { get; private set; }
	public int K { get; private set; }
	public double Value { get; private set; }

	public SparseItem(int i, int k, double value) {
		I     = i;
		K     = k;
		Value = value;
	}
}
}
