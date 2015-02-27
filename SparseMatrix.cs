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
using System.Text;

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
	/// Initial value for every cell. If an unassigned cell is requested, this value will be returned.
	/// </summary>
	public double DefaultValue { get; private set; }

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
		data         = new Dictionary<int, Dictionary<int, double>>();
		DefaultValue = 0;
		Width        = 0;
		Height       = 0;
	}

	/// <summary>
	/// Construct a new SparseMatrix with a given initial width and height.
	/// </summary>
	/// <param name="width">Initial width.</param>
	/// <param name="height">Initial height.</param>
	/// <param name="defaultvalue">Unassigned cell value.</param>
	public SparseMatrix(int width, int height, double defaultvalue = 0)
	{
		data         = new Dictionary<int, Dictionary<int, double>>();
		DefaultValue = defaultvalue;
		Width        = width;
		Height       = height;
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
		
		this.DefaultValue = that.DefaultValue;
		this.Width        = that.Width;
		this.Height       = that.Height;
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
			return (data.TryGetValue(i, out row) && row.TryGetValue(k, out value)) ? value : DefaultValue;
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
		SparseMatrix transpose = new SparseMatrix(Height, Width);

		foreach (var item in this) {
			transpose[item.K, item.I] = item.Value;
		}

		return transpose;
	}

	/// <summary>
	/// Query if the specified entry is defined or is in its default state.
	/// </summary>
	/// <param name="i">Row index.</param>
	/// <param name="k">Column index.</param>
	/// <returns>True if the entry is defined.</returns>
	public bool Defines(int i, int k)
	{
		Dictionary<int, double> row;
		return data.TryGetValue(i, out row) && row.ContainsKey(k);
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
		if (!data.TryGetValue(i, out introw)) {
			introw = new Dictionary<int,double>();
			data.Add(i, introw);
		}

		int lastkey = int.MinValue;
		foreach (var item in row) {
			introw[item.Key] = item.Value;
			lastkey = Math.Max(lastkey, item.Key);
		}

		Width  = Math.Max(Width,  lastkey + 1);
		Height = Math.Max(Height, i + 1);
	}

	/// <summary>
	/// Add every entry of a given column to the matrix.
	/// Any previous colliding entry will be overwritten.
	/// </summary>
	/// <param name="i">Column index where the entries will be added.</param>
	/// <param name="column">New values.</param>
	public void AddColumn(int k, Dictionary<int, double> column)
	{
		int lastkey = int.MinValue;
		foreach (var item in column) {
			Dictionary<int, double> intcol;
			lastkey = Math.Max(lastkey, item.Key);

			if (data.TryGetValue(item.Key, out intcol)) {
				intcol[k] = item.Value;
			}
			else {
				var newrow = new Dictionary<int,double>();
				newrow[k]  = item.Value;
				data.Add(item.Key, newrow);
			}
		}
		
		Width  = Math.Max(Width,  k + 1);
		Height = Math.Max(Height, lastkey + 1);
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
	/// <param name="reducer">Applied function. The first parameter is the cumulative result,
	/// the second parameter is the new entry.</param>
	/// <param name="initvalue">Initial value for the accumulation.</param>
	/// <returns>Sparse list of the result of applying the recursive accumulator to each row on the matrix.
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
	/// Apply a function iteratively on every item of the matrix
	/// and obtain the accumulated result.
	/// </summary>
	/// <param name="reducer">Applied function. The first parameter is the cumulative result,
	/// the second parameter is the new entry.</param>
	/// <param name="initvalue">Initial value for the accumulation.</param>
	/// <returns>Result of applying the recursive accumulator to each item on the matrix.
	/// Note that the order in which the items are reduced is undeterminate.</returns>
	public double FoldMatrix(Func<double, double, double> reducer, double initvalue)
	{
		double fold = initvalue;
		
		foreach (var item in this) {
			fold = reducer(fold, item.Value);
		}

		return fold;
	}

	/// <summary>
	/// Apply a function iteratively on every item of a column (for each column)
	/// and obtain the accumulated results.
	/// </summary>
	/// <param name="reducer">Applied function. The first parameter is the cumulative result,
	/// the second parameter is the new entry. It must return the new accumulation.</param>
	/// <param name="initvalue">Initial value for the accumulation.</param>
	/// <returns>Sparse list of the result of applying the recursive accumulator to each column on the matrix.
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
	/// Remove all empty rows and columns and then reindex the remaining data.
	/// </summary>
	/// <returns></returns>
	public SparseMatrix Compact()
	{
		int[] rows, columns;

		return Compact(out rows, out columns);
	}

	private object xlock = new object();

	/// <summary>
	/// Remove all empty rows and columns and then reindex the remaining data.
	/// </summary>
	/// <param name="rows">Old row indices.</param>
	/// <param name="cols">Old column indices.</param>
	/// <returns></returns>
	public SparseMatrix Compact(out int[] rows, out int[] cols)
	{
		Dictionary<int, int> rowset = new Dictionary<int, int>();
		Dictionary<int, int> colset = new Dictionary<int, int>();

		foreach (var item in this) {
			rowset[item.I] = -1;
			colset[item.K] = -1;
		}

		//Console.WriteLine(this.ToStringFull());
		//Console.WriteLine(rowset.Count);
		//Console.WriteLine(colset.Count);
		
		rows = new List<int>(rowset.Keys).ToArray();
		cols = new List<int>(colset.Keys).ToArray();

		Array.Sort(rows);
		Array.Sort(cols);
		
		for (int i = 0; i < rows.Length; i++) {
			rowset[rows[i]] = i;
		}

		for (int k = 0; k < cols.Length; k++) {
			colset[cols[k]] = k;
		}

		SparseMatrix compacted = new SparseMatrix(rows.Length, cols.Length);

		foreach (var item in this) {
			//Console.WriteLine(item + "  " + rows.Length + "  " + cols.Length);
			compacted[rowset[item.I], colset[item.K]] = item.Value;
		}

		return compacted;
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

	/// <summary>
	/// Efficient equality comparer with other SparseMatrix.
	/// </summary>
	/// <param name="that">Compared matrix.</param>
	/// <returns>True if both matrix are structurally identical.
	/// This implies having the same values and the same defined structure.
	/// If one matrix defines an entry as zero and the other doesn't
	/// they are mathematically equal, but they are structurally different,
	/// i.e. they can answer the question "mat.Defines(i, k)" differently.</returns>
	public bool Equals(SparseMatrix that)
	{
		if (Count != that.Count || DefaultValue != that.DefaultValue || Width != that.Width || Height != that.Height) {
			return false;
		}

		foreach (var item in this) {
			if (!that.Defines(item.I, item.K) || (that[item.I, item.K] != item.Value)) {
				Console.WriteLine(that.Defines(item.I, item.K) + " " + (that[item.I, item.K] != item.Value));
				return false;
			}
		}

		return true;
	}

	/// <summary>
	/// Compares this object with another.
	/// </summary>
	/// <param name="that">Compared object.</param>
	/// <returns>True if the objects are structurally equal.</returns>
	public override bool Equals(object that)
	{
		return that is SparseMatrix && this.Equals(that as SparseMatrix);
	}

	/// <summary>
	/// Get a unique code that is equal for any two equal SparseMatrices.
	/// </summary>
	/// <returns>Hash code.</returns>
	public override int GetHashCode()
	{
		int hash = 17;
		
		hash = unchecked(37 * hash + DefaultValue.GetHashCode());
		hash = unchecked(37 * hash + Width);
		hash = unchecked(37 * hash + Height);
		
		foreach (var item in this) {
			hash = unchecked(37 * hash + item.I);
			hash = unchecked(37 * hash + item.K);
			hash = unchecked(37 * hash + item.Value.GetHashCode());
		}

		return hash;
	}

	/// <summary>
	/// Get a string representing the matrix
	/// that is sparse in nature.
	/// </summary>
	/// <returns>Sparse string representation.</returns>
	public override string ToString()
	{
		return "[" + string.Join(", ", this) + "]";
	}

	/// <summary>
	/// Get a string representing the matrix
	/// that is dense in nature.
	/// </summary>
	/// <returns>Dense string representation.</returns>
	public string ToStringFull()
	{
		string[] rows = new string[Height];
		
		for (int i = 0; i < Height; i++) {

			StringBuilder rep = new StringBuilder();
			rep.Append("[");

			if (Width > 0) {
				rep.Append(this[i, 0]);
			}

			for (int k = 1; k < Width; k++) {
				rep.Append(", ");
				rep.Append(this[i, k]);
			}

			rep.Append("]");

			rows[i] = rep.ToString();
		}

		return "[" + string.Join(",\n ", rows) + "]";
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

	public override string ToString()
	{
		StringBuilder rep = new StringBuilder();

		rep.Append("(");
		rep.Append(I);
		rep.Append(", ");
		rep.Append(K);
		rep.Append(": ");
		rep.Append(Value);
		rep.Append(")");

		return rep.ToString();
	}
}
}
