﻿// GraphCombinatorics.cs
// Graph combinatorics algorithms
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
using AE = monorfs.ArrayExtensions;

namespace monorfs
{
/// <summary>
/// Graph combinatorics algorithms.
/// </summary>
class GraphCombinatorics
{
	/// <summary>
	/// Solve the linear assignment problem. Choose the best matching between workers and works
	/// to be as efficient as possible (maximize utility sum).
	/// </summary>
	/// <param name="matrix">Utility matrix, where each row represent a worker and each column, a work.
	/// It is represented sparsely, but every index should have at least one entry (the return is a non-sparse list).
	/// This is for efficiency reasons, as the problem at hand doesn't require sparsity in the output.</param>
	/// <returns>Dense assignment list.</returns>
	public static int[] LinearAssignment(SparseMatrix matrix)
	{
		return Hungarian(matrix);
	}

	/// <summary>
	/// Solve the linear assignment problem with the hungarian algorithm proposed by Kuhn.
	/// </summary>
	/// <param name="matrix">Utility matrix, where each row represent a worker and each column, a work.
	/// It is represented sparsely, but every index should have at least one entry (the return is a non-sparse list).
	/// This is for efficiency reasons, as the problem at hand doesn't require sparsity in the output.</param>
	/// <returns>Dense assignment list.</returns>
	private static int[] Hungarian(SparseMatrix matrix)
	{
		//Console.WriteLine(matrix.ToStringFull());
		//Console.WriteLine("");

		// initial feasible labeling
		var      rowmax = matrix.FoldRows((cum, mi) => Math.Max(cum, mi), 0);
		double[] labelx = new double[matrix.Height];
		double[] labely = new double[matrix.Width];

		int   [] matchx = new int   [labelx.Length];
		int   [] matchy = new int   [labelx.Length];

		bool  [] visitx = new bool  [labelx.Length];  // set represented by its indicator function
		bool  [] visity = new bool  [labely.Length];  // idem

		double[] slack  = new double[labely.Length];
		int   [] parent = new int   [labely.Length];
		int      root   = -1;

		int    iminslack = int.MaxValue;
		double delta     = double.PositiveInfinity;

		foreach (var item in rowmax) {
			labelx[item.Key] = item.Value;
		}
		
		AE.Fill(matchx, -1);
		AE.Fill(matchy, -1);

		//Action tellme = () =>
		//{
		//	Console.WriteLine("labelx: [" + string.Join(", ", Array.ConvertAll(labelx, x => x.ToString("F2"))) + "]");
		//	Console.WriteLine("labely: [" + string.Join(", ", Array.ConvertAll(labely, x => x.ToString("F2"))) + "]");
			
		//	Console.WriteLine("matchx: [" + string.Join(", ", Array.ConvertAll(matchx, x => x.ToString())) + "]");
		//	Console.WriteLine("matchy: [" + string.Join(", ", Array.ConvertAll(matchy, x => x.ToString())) + "]");

		//	Console.WriteLine("visitx: [" + string.Join(", ", Array.ConvertAll(visitx, x => x ? "1" : "0")) + "]");
		//	Console.WriteLine("visity: [" + string.Join(", ", Array.ConvertAll(visity, x => x ? "1" : "0")) + "]");
			
		//	Console.WriteLine("slack:  [" + string.Join(", ", Array.ConvertAll(slack,  x => x.ToString())) + "]");
		//	Console.WriteLine("parent: [" + string.Join(", ", Array.ConvertAll(parent, x => x.ToString())) + "]");
			
		//	Console.WriteLine("root:   " + root);
		//	Console.WriteLine("");
		//};

		// find an unmatched worker to start building a tree
		//int h = 0;
		while ((root = Array.IndexOf(matchx, -1)) != -1) {
			// annotate all its slacks
			AE.Fill(parent, root);
			AE.Fill(slack, i => matrix.Defines(root, i) ? labelx[root] + labely[i] - matrix[root, i] : double.PositiveInfinity);
			
			Array.Clear(visitx, 0, visitx.Length);
			Array.Clear(visity, 0, visity.Length);
			visitx[root] = true;

			//Console.WriteLine("Iteration " + h++ + "\n-----------------");
			//tellme();

			bool found = false;
			//int m = 0;
			while (!found) {
				// find the minimal slack where "y is in T"
				iminslack = int.MaxValue;
				delta     = double.PositiveInfinity;

				//Console.WriteLine("    Sub-iteration " + m++ + "\n    ---------------------");

				for (int i = 0; i < slack.Length; i++) {
					if (visity[i] == false && slack[i] < delta) {
						iminslack = i;
						delta    = slack[i];
					}
				}

				if (delta == double.PositiveInfinity) {
					return null;  // there is no solution
				}
				
				//Console.WriteLine("    iminslack: " + iminslack);
				//Console.WriteLine("    delta:     " + delta);

				// change the labels to tighten the slacks
				// {
				for (int i = 0; i < labelx.Length; i++) {
					if (visitx[i] == true) {
						labelx[i] -= delta;
					}
				}

				for (int i = 0; i < labely.Length; i++) {
					if (visity[i] == true) {
						labely[i] += delta;
					}
					else {
						slack[i] -= delta;
					}
				}
				// }

				visity[iminslack] = true;

				//Console.WriteLine("\n    Tightened______");
				//Console.WriteLine("    labelx: [" + string.Join(", ", Array.ConvertAll(labelx, x => x.ToString("F2"))) + "]");
				//Console.WriteLine("    labely: [" + string.Join(", ", Array.ConvertAll(labely, x => x.ToString("F2"))) + "]");
				//Console.WriteLine("    slack:  [" + string.Join(", ", Array.ConvertAll(slack,  x => x.ToString()))     + "]");
				//Console.WriteLine("    ");
				//Console.WriteLine("    visity: [" + string.Join(", ", Array.ConvertAll(visity, x => x ? "1" : "0")) + "]");
				//Console.WriteLine("    ");

				// for matched minimal slack, expand tree
				if (matchy[iminslack] != -1) {
					int match = matchy[iminslack];
					visitx[match]  = true;

					//Console.WriteLine("    matched leaf:");

					for (int i = 0; i < labely.Length; i++) {
						if (!visity[i]) {
							double mdelta = matrix.Defines(match, i) ? labelx[match] + labely[i] - matrix[match, i] : double.PositiveInfinity;
							if (mdelta < slack[i]) {
								slack [i] = mdelta;
								parent[i] = match;
							}
						}
					}
					
					//Console.WriteLine("    visitx: [" + string.Join(", ", Array.ConvertAll(visitx, x => x ? "1" : "0")) + "]");
					//Console.WriteLine("    slack:  [" + string.Join(", ", Array.ConvertAll(slack,  x => x.ToString())) + "]");
					//Console.WriteLine("    parent: [" + string.Join(", ", Array.ConvertAll(parent, x => x.ToString())) + "]");
				}
				// for unmatched minimal slack, the tree has an augmenting branch,
				// follow it and invert every edge
				else {
					found = true;

					//Console.WriteLine("    free leaf");
				}

				//Console.WriteLine("    ");
			}

			// invert augmenting branch
			// {
			//Console.WriteLine("following branch...");
			int px, py, ty;
			for (py = iminslack, px = parent[py]; px != root; py = ty, px = parent[py]) {
				ty = matchx[px];
				matchx[px] = py;
				matchy[py] = px;

				//Console.WriteLine(px + "->" + py);

			}

			matchx[px] = py;
			matchy[py] = px;
			//Console.WriteLine(px + "->" + py);
			// }
			
			//Console.WriteLine("");
		}

		return matchx;
	}

	/// <summary>
	/// Compute an assignment associated total profit.
	/// </summary>
	/// <param name="profit">Profit matrix.</param>
	/// <param name="matches">Match list.</param>
	/// <returns>Total profit.</returns>
	public static double AssignmentValue(SparseMatrix profit, int[] matches)
	{
		// null means no solution
		if (matches == null) {
			return double.NegativeInfinity;
		}

		double total = 0;

		for (int i = 0; i < matches.Length; i++) {
			total += profit[i, matches[i]];
		}

		return total;
	}

	/// <summary>
	/// Reduce a profit matrix by eliminating and forcing certain edges.
	/// </summary>
	/// <param name="full">Original complete profit.</param>
	/// <param name="reducer">Descriptor of the changes that should be done.</param>
	private static SparseMatrix reduceprofit(SparseMatrix full, MurtyNode reducer)
	{
		SparseMatrix reduced    = new SparseMatrix(full);
		int[]        removerows = new int[reducer.Forced.Length];
		int[]        removecols = new int[reducer.Forced.Length];

		int h = 0;
		foreach (MatrixKey force in reducer.Forced) {
			removerows[h] = force.I;
			removecols[h] = force.K;

			h++;
		}
		
		reduced.RemoveRows   (removerows);
		reduced.RemoveColumns(removecols);
		
		foreach (MatrixKey force in reducer.Forced) {
			reduced[force.I, force.K] = 1;  // "infinity" against removed edges
		}

		foreach (MatrixKey force in reducer.Eliminated) {
			reduced.RemoveAt(force.I, force.K);
		}

		return reduced;
	}

	/// <summary>
	/// Enumerate all possible pairings using a Murty's algorithm.
	/// </summary>
	/// <param name="profit">Profit matrix to be maximized.</param>
	/// <returns>Enumeration of most likely assignments pairing permutations.</returns>
	public static IEnumerable<int[]> MurtyPairing(SparseMatrix profit)
	{
		var       frontier = new PriorityQueue<MurtyNode>();
		MurtyNode first    = new MurtyNode();
		first.Assignment   = LinearAssignment(profit);

		//Console.WriteLine(profit.ToStringFull());

		frontier.Add(AssignmentValue(profit, first.Assignment), first);

		while (frontier.Count > 0) {
			//Console.WriteLine(frontier.Count);
			MurtyNode best = frontier.Pop();
			//Console.WriteLine(frontier.Count);
			//Console.WriteLine(best);

			yield return best.Assignment;

			//Console.WriteLine("children___");
			foreach (MurtyNode child in best.Children) {
				child.Assignment = LinearAssignment(reduceprofit(profit, child));
				//Console.WriteLine(child);
				//Console.WriteLine(reduceprofit(profit, child).ToStringFull());
				//Console.WriteLine(AssignmentValue(reduceprofit(profit, child), child.Assignment));

				if (child.Assignment != null) {
					frontier.Add(AssignmentValue(profit, child.Assignment), child);
				}
			}
		}
	}

	/// <summary>
	/// Enumerate all possible pairings using a lexicographical order.
	/// </summary>
	/// <param name="profit">Profit matrix to be maximized.</param>
	/// <param name="modelsize">Number of model components (complete model).</param>
	/// <returns>Enumeration of model-measurement pairing permutations.</returns>
	public static IEnumerable<int[]> LexicographicalPairing(SparseMatrix profit, int modelsize)
	{
		int[] permutation = new int[profit.Rows.Count];
		int   a, b;

		int h = 0;
		foreach (var row in profit.Rows) {
			permutation[h++] = row.Key;
		}

		Array.Sort(permutation);
		//Console.WriteLine(string.Join(", ", Array.ConvertAll(permutation, i => i.ToString())));

		int measurestart = permutation.Length;
		for (int i = 0; i < permutation.Length; i++) {
			if (permutation[i] >=  modelsize) {
				measurestart = i;
				break;
			}
		}
		
		// to avoid equivalent permutations, all the measurement nodes are reversed
		// so only one permutation of this segment is used
		Array.Reverse(permutation, measurestart, permutation.Length - measurestart);
		yield return permutation;

		while (!lastpermutation(permutation)) {
			// find last a such that p[a] < p[a + 1]
			for (a = permutation.Length - 2; a > 0; a--) {
				if (permutation[a] < permutation[a + 1]) {
					break;
				}
			}

			// find last b > a such tha p[a] < p[b]
			for (b = permutation.Length - 1; b > a; b--) {
				if (permutation[a] < permutation[b]) {
					break;
				}
			}

			// swap the edges
			int temp       = permutation[a];
			permutation[a] = permutation[b];
			permutation[b] = temp;

			// and reverse the tail
			Array.Reverse(permutation, a + 1, permutation.Length - (a + 1));

			// to avoid equivalent permutations, all the measurement nodes are reversed
			// so only one permutation of this segment is used
			Array.Reverse(permutation, measurestart, permutation.Length - measurestart);
			yield return permutation;
		}
	}

	/// <summary>
	/// Detect the last permutation possible in a lexicographical order.
	/// </summary>
	/// <param name="permutation">Queried permutation.</param>
	/// <returns>True if there's no next permutation in the lexicographical order.</returns>
	private static bool lastpermutation(int[] permutation)
	{
		for (int i = 1; i < permutation.Length; i++) {
			if (permutation[i - 1] < permutation[i]) {
				return false;
			}
		}

		return true;
	}

	/// <summary>
	/// Partition a graph (expressed as a weight matrix) into a list of
	/// disjoint connected subcomponents.
	/// </summary>
	/// <param name="original">Original graph weight matrix.</param>
	/// <returns>Connected subcomponents' weight matrices.</returns>
	public static List<SparseMatrix> ConnectedComponents(SparseMatrix original)
	{
		SparseMatrix       complete   = new SparseMatrix(original);
		List<SparseMatrix> components = new List<SparseMatrix>();

		while (complete.Count > 0) {
			List<int> rowstack = new List<int>();
			List<int> colstack = new List<int>();

			bool[] rowvisited = new bool[complete.Height];
			bool[] colvisited = new bool[complete.Width];

			SparseMatrix component = new SparseMatrix(complete.Width, complete.Height);
			SparseItem   element   = complete.Any;
			rowstack.Add(element.I);
			colstack.Add(element.K);

			while (rowstack.Count > 0 || colstack.Count > 0) {
				//Console.WriteLine("|------");
				//Console.WriteLine("rowstack: [" + string.Join(", ", rowstack.ConvertAll(i => i.ToString())) + "]");
				//Console.WriteLine("colstack: [" + string.Join(", ", rowstack.ConvertAll(i => i.ToString())) + "]");
				//Console.WriteLine("");

				while  (rowstack.Count > 0) {
					int nextrow = rowstack[rowstack.Count - 1];
					rowstack.RemoveAt(rowstack.Count - 1);

					//Console.WriteLine("nextrow: " + nextrow);

					if (rowvisited[nextrow] == true) {
						continue;
					}

					rowvisited[nextrow] = true;

					var row = complete.Row(nextrow);

					component.AddRow(nextrow, row);
					complete.RemoveRow(nextrow);
					
					//Console.WriteLine("row =\n" + "[" + string.Join(", ", row.Select(i => i.Key + ": " + i.Value)) + "]");
					//Console.WriteLine("component =\n" + component.ToStringFull());
					//Console.WriteLine("complete =\n" + complete.ToStringFull());

					foreach (var k in row.Keys) {
						if (colvisited[k] == false && colstack.IndexOf(k) == -1) {
							colstack.Add(k);
							//Console.WriteLine("colstack add: " + k);
						}
					}
				}

				while (colstack.Count > 0) {
					int nextcol = colstack[colstack.Count - 1];
					colstack.RemoveAt(colstack.Count - 1);

					//Console.WriteLine("nextcol: " + nextcol);

					if (colvisited[nextcol] == true) {
						continue;
					}

					colvisited[nextcol] = true;

					var col = complete.Column(nextcol);

					component.AddColumn(nextcol, col);
					complete.RemoveColumn(nextcol);
					
					//Console.WriteLine("col =\n" + "[" + string.Join(", ", col.Select(i => i.Key + ": " + i.Value)) + "]");
					//Console.WriteLine("component =\n" + component.ToStringFull());
					//Console.WriteLine("complete =\n" + complete.ToStringFull());

					foreach (var i in col.Keys) {
						if (rowvisited[i] == false && rowstack.IndexOf(i) == -1) {
							rowstack.Add(i);
							//Console.WriteLine("rowstack add: " + i);
						}
					}
				}
			}

			components.Add(component);
		}

		return components;
	}
}

/// <summary>
/// Node for the Murty's algorithm.
/// </summary>
public class MurtyNode
{
	/// <summary>
	/// Edges that must be on the solution.
	/// </summary>
	public MatrixKey[] Forced { get; private set; }

	/// <summary>
	/// Edges that must not be on the solution.
	/// </summary>
	public MatrixKey[] Eliminated { get; private set; }

	/// <summary>
	/// Associated reduced profit matrix assignment solution.
	/// </summary>
	public int[] Assignment { get; set; }

	/// <summary>
	/// Construct a MurtyNode with no enforced or forbidden edges.
	/// </summary>
	public MurtyNode() : this(new MatrixKey[0], new MatrixKey[0]) {}

	/// <summary>
	/// Construct a MurtyNode with specified forced and eliminated lists.
	/// </summary>
	/// <param name="forced">Forced edges.</param>
	/// <param name="eliminated">Forbidden edges.</param>
	public MurtyNode(MatrixKey[] forced, MatrixKey[] eliminated)
	{
		Forced     = forced;
		Eliminated = eliminated;
	}

	/// <summary>
	/// Get children nodes at run-time.
	/// Assignment must have been pre-obtained for these to be meaningful;
	/// if it is null, it is considered that no solution exists => no children.
	/// </summary>
	public MurtyNode[] Children
	{
		get
		{
			if (Assignment == null) {
				return new MurtyNode[0];
			}

			List<MatrixKey> iremaining = new List<MatrixKey>();
			
			for (int i = 0; i < Assignment.Length; i++) {
				if (Array.IndexOf(Forced, new MatrixKey(i, Assignment[i])) == -1) {
					iremaining.Add(new MatrixKey(i, Assignment[i]));
				}
			}

			List<MurtyNode> children = new List<MurtyNode>();

			for (int i = 0; i < iremaining.Count - 1; i++) {
				if (Array.IndexOf(Forced, iremaining[i]) == -1) {
					MatrixKey[] eliminated = new MatrixKey[Eliminated.Length + 1];
					MatrixKey[] forced     = new MatrixKey[Forced    .Length + i];
				
					Eliminated.CopyTo(eliminated, 0);
					Forced    .CopyTo(forced,     0);

					eliminated[Eliminated.Length] = iremaining[i];

					for (int k = 0; k < i; k++) {
						if (Array.IndexOf(Forced, iremaining[k]) == -1) {
							forced[Forced.Length + k] = iremaining[k];
						}
					}

					children.Add(new MurtyNode(forced, eliminated));
				}
			}

			return children.ToArray();
		}
	}
	
	/// <summary>
	/// Efficient equality comparer with other MurtyNode.
	/// </summary>
	/// <param name="that">Compared node.</param>
	/// <returns>True if both nodes are structurally identical.
	/// This implies having the same field values and the same defined order for each field.</returns>
	public bool Equals(MurtyNode that)
	{
		if (this.Eliminated.Length != that.Eliminated.Length || this.Forced.Length != that.Forced.Length) {
			return false;
		}

		for (int i = 0; i < this.Eliminated.Length; i++) {
			if (this.Eliminated[i] != that.Eliminated[i]) {
				return false;
			}
		}

		for (int i = 0; i < this.Forced.Length; i++) {
			if (this.Forced[i] != that.Forced[i]) {
				return false;
			}
		}
		
		if (this.Assignment == null) {
			return that.Assignment == null;
		}
		else {
			if (that.Assignment == null) {
				return false;
			}

			for (int i = 0; i < this.Assignment.Length; i++) {
				if (this.Assignment[i] != that.Assignment[i]) {
					return false;
				}
			}
		}

		return true;
	}
	
	/// <summary>
	/// Compares this object with another.
	/// </summary>
	/// <param name="that">Compared object.</param>
	/// <returns>True if the objects are equal.</returns>
	public override bool Equals(object that)
	{
		return that is MurtyNode && this.Equals(that as MurtyNode);
	}

	/// <summary>
	/// Get a unique code that is equal for any two equal MurtyNodes.
	/// </summary>
	/// <returns>Hash code.</returns>
	public override int GetHashCode()
	{
		int hash = 17;
		hash = unchecked(37 * hash + Eliminated.GetHashCode());
		hash = unchecked(37 * hash + Forced    .GetHashCode());

		if (Assignment != null) {
			hash = unchecked(37 * hash + Assignment.GetHashCode());
		}

		return hash;
	}

	/// <summary>
	/// Get a string representation of the node.
	/// </summary>
	/// <returns>String representation.</returns>
	public override string ToString()
	{
		return "{\n    eliminated: " + string.Join(", ", Array.ConvertAll(Eliminated, i => i.ToString())) + "\n" +
		          "    forced:     " + string.Join(", ", Array.ConvertAll(Forced,     i => i.ToString())) + "\n" +
		          "    assignment: " + ((Assignment != null) ? string.Join(", ", Array.ConvertAll(Assignment, i => i.ToString())) : "null") + "\n}";
	}
}

/// <summary>
/// List of elements sorted by a priority value.
/// </summary>
public class PriorityQueue<T> : IEnumerable<KeyValuePair<double, T>>
{
	/// <summary>
	/// Internal list of values and priorities.
	/// </summary>
	private List<KeyValuePair<double, T>> data;

	/// <summary>
	/// Construct an empty PriorityQueue.
	/// </summary>
	public PriorityQueue()
	{
		data = new List<KeyValuePair<double, T>>();
	}

	/// <summary>
	/// Construct an empty PriorityQueue with initial capacity.
	/// </summary>
	/// <param name="capacity">Initial capacity.</param>
	public PriorityQueue(int capacity)
	{
		data = new List<KeyValuePair<double, T>>(capacity);
	}

	/// <summary>
	/// Construct a PriorityQueue from a IEnumerable.
	/// </summary>
	/// <param name="enumerable">Copied IEnumerable.</param>
	public PriorityQueue(IEnumerable<KeyValuePair<double, T>> enumerable)
	{
		data = new List<KeyValuePair<double, T>>(enumerable);
	}

	/// <summary>
	/// Get the number of elements in the queue.
	/// </summary>
	public int Count { get { return data.Count; } }

	/// <summary>
	/// Add an item to the queue.
	/// </summary>
	/// <param name="priority">Item priority; bigger the number, higher the priority.</param>
	/// <param name="item">Item to add.</param>
	public void Add(double priority, T item)
	{
		data.Add(new KeyValuePair<double, T>(priority, item));
		data.Sort((a, b) => Math.Sign(a.Key - b.Key));
	}

	/// <summary>
	/// Fetch the item with the highest priority.
	/// </summary>
	/// <returns>Highest priority item.</returns>
	public T Top()
	{
		return data[data.Count - 1].Value;
	}

	/// <summary>
	/// Get the item with the highest priority and remove it from the queue.
	/// </summary>
	/// <returns>Highest priority item.</returns>
	public T Pop()
	{
		T item = data[data.Count - 1].Value;
		data.RemoveAt(data.Count - 1);

		return item;
	}

	/// <summary>
	/// Get an IEnumerator for the class in descending priority order.
	/// </summary>
	/// <returns>Enumerator.</returns>
	public IEnumerator<KeyValuePair<double, T>> GetEnumerator()
	{
		return data.GetEnumerator();
	}
	
	/// <summary>
	/// Get a nongeneric IEnumerator for the class in descending priority order.
	/// </summary>
	/// <returns>Enumerator.</returns>
	IEnumerator IEnumerable.GetEnumerator()
	{
		return GetEnumerator();
	}
}

/// <summary>
/// Matrix index pair.
/// </summary>
public class MatrixKey
{
	/// <summary>
	/// Column index.
	/// </summary>
	public int I { get; private set; }

	/// <summary>
	/// Row index.
	/// </summary>
	public int K { get; private set; }

	/// <summary>
	/// Construct a matrix key given column/row indices.
	/// </summary>
	/// <param name="i">Column index.</param>
	/// <param name="k">Row index.</param>
	public MatrixKey(int i, int k)
	{
		I = i;
		K = k;
	}

	/// <summary>
	/// Compares with another matrix key by value.
	/// Two keys are equal iff both the column and row indices are equal.
	/// </summary>
	/// <param name="that">Compared key.</param>
	/// <returns>True if both keys refer to the same matrix index; false otherwise.</returns>
	public bool Equals(MatrixKey that)
	{
		return this.I == that.I && this.K == that.K;
	}
	
	/// <summary>
	/// Compares with another object by value.
	/// </summary>
	/// <param name="that">Compared object.</param>
	/// <returns>True if both keys refer to the same matrix index; false otherwise.</returns>
	public override bool Equals(object that)
	{
		return that is MatrixKey && this.Equals(that as MatrixKey);
	}

	/// <summary>
	/// Get a quasi-unique code that represents the object.
	/// </summary>
	/// <returns>Hash code.</returns>
	public override int GetHashCode()
	{
		int hash = 17;

		hash = unchecked(37 * hash + I);
		hash = unchecked(37 * hash + K);

		return hash;
	}
	
	/// <summary>
	/// Get a string representation of the key.
	/// </summary>
	/// <returns>String representation.</returns>
	public override string ToString()
	{
		return "(" + I + ", " + K + ")";
	}
}
}
