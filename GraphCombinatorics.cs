// GraphCombinatorics.cs
// Graph combinatorics algorithms.
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
		// initial feasible labeling
		var      rowmax = matrix.FoldRows((cum, mi) => Math.Max(cum, mi), 0);
		double[] labelx = new double[matrix.Height];
		double[] labely = new double[matrix.Width];

		int   [] matchx = new int   [labelx.Length];
		int   [] matchy = new int   [labelx.Length];

		bool  [] S      = new bool  [labelx.Length];  // set represented by its indicator function
		bool  [] T      = new bool  [labely.Length];  // idem

		double[] slack  = new double[labely.Length];
		int   [] parent = new int   [labely.Length];
		int      root   = -1;

		foreach (var item in rowmax) {
			labelx[item.Key] = item.Value;
		}

		// find an unmatched worker to start building a tree
		while ((root = Array.IndexOf(matchx, -1)) != -1) {

			// annotate all its slacks
			AE.Fill(parent, root);
			AE.Fill(slack, i => labelx[root] + labely[i] - matrix[root, i]);

			bool done = false;
			while (!done) {
				// find the minimal slack where "y is in T"
				int iminslack = int.MaxValue;
				double delta  = 0;

				for (int i = 0; i < slack.Length; i++) {
					if (T[i] == false && slack[i] < iminslack) {
						iminslack = i;
						delta    = slack[i];
					}
				}

				// change the labels to tighten the slacks
				for (int i = 0; i < labelx.Length; i++) {
					labelx[i] -= delta;
				}

				for (int i = 0; i < labely.Length; i++) {
					if (T[i] == true) {
						labely[i] += delta;
					}
					else {
						slack[i] -= delta;
					}
				}

				T[iminslack] = true;

				// for matched minimal slack, expand tree
				if (matchy[iminslack] != -1) {
					int match = matchy[iminslack];
					S[match]  = true;

					for (int i = 0; i < labely.Length; i++) {
						if (!T[i]) {
							double mslack = labelx[match] + labely[i] - matrix[match, i];
							if (mslack < slack[i]) {
								slack [i] = mslack;
								parent[i] = match;
							}
						}
					}
				}
				// for unmatched minimal slack, the tree has an augmenting branch,
				// follow it and invert every edge
				else {
					for (int py = iminslack, px = parent[py], ty; matchx[px] != -1; py = ty, px = parent[py]) {
						ty = matchx[px];
						matchx[px] = py;
						matchy[py] = px;

					}

					done = true;
				}
			}
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
			reduced[force.I, force.K] = 2.0;  // "infinity" in probabilities
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

		frontier.Add(AssignmentValue(profit, first.Assignment), first);

		while (frontier.Count > 0) {
			MurtyNode best = frontier.Pop();

			yield return best.Assignment;

			foreach (MurtyNode child in best.Children) {
				best.Assignment = LinearAssignment(reduceprofit(profit, best));
				frontier.Add(AssignmentValue(profit, best.Assignment), best);
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

		for (int i = 0; i < permutation.Length; i++) {
			if (permutation[i] >= modelsize) {
				permutation[i] = -permutation[i];
			}
		}

		while (!lastpermutation(permutation)) {
			// find lowest a such that p[a - 1] > p[a]
			for (a = 1; a < permutation.Length; a++) {
				if (permutation[a - 1] > permutation[a]) {
					break;
				}
			}

			// find lowest b < a such tha p[b] > p[a]
			for (b = 0; b < a; b++) {
				if (permutation[b] > permutation[b]) {
					break;
				}
			}

			// swap the edges
			int temp       = permutation[a];
			permutation[a] = permutation[b];
			permutation[b] = temp;

			// and reverse the tail
			for (int k = 0; k < a - 1; k++) {
				temp                   = permutation[k];
				permutation[k]         = permutation[a - 1 - k];
				permutation[a - 1 - k] = temp;
			}

			/*
			 * TODO all of this should be outside the combinatorics class... too specific
			// convert to model/measurement pairing
			var pairing = new List<Tuple<Gaussian, double[]>>();

			// paired model + misdetected
			for (int i = 0; i < model.Count; i++) {
				pairing.Add(new Tuple<Gaussian, double[]>(model[i], (permutation[i] != -1) ? measurements[permutation[i]] : null));
			}

			// clutter
			// TODO this may be speedable (by assuming all clutter permutation indices are next to each other => break somewhere)
			for (int i = 0; i < measurements.Count; i++) {
				if (permutation[i + measurements.Count] != -1) {
					pairing.Add(new Tuple<Gaussian, double[]>(model[i], measurements[permutation[i + measurements.Count]]));
				}
			}
			 * */

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
			if (permutation[i - 1] > permutation[i]) {
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

			SparseMatrix component = new SparseMatrix();
			SparseItem   element   = complete.Any;
			rowstack.Add(element.I);
			colstack.Add(element.K);

			while (rowstack.Count > 0 || colstack.Count > 0) {
				while  (rowstack.Count > 0) {
					int nextrow = rowstack[rowstack.Count - 1];
					rowstack.RemoveAt(rowstack.Count - 1);

					if (rowvisited[nextrow] == true) {
						continue;
					}

					rowvisited[nextrow] = true;

					var row = complete.Row(nextrow);

					component.AddRow(nextrow, row);
					complete.RemoveRow(nextrow);

					foreach (var k in row.Keys) {
						if (colvisited[k] == false) {
							colstack.Add(k);
						}
					}
				}

				while (colstack.Count > 0) {
					int nextcol = colstack[colstack.Count - 1];
					colstack.RemoveAt(colstack.Count - 1);

					if (colvisited[nextcol] == true) {
						continue;
					}

					colvisited[nextcol] = true;

					var col = complete.Column(nextcol);

					component.AddColumn(nextcol, col);
					complete.RemoveRow(nextcol);

					foreach (var i in col.Keys) {
						if (rowvisited[i] == false) {
							rowstack.Add(i);
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
	/// Assignment must have been pre-obtained for these to be meaningful.
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

			for (int i = 0; i < iremaining.Count; i++) {
				MatrixKey[] eliminated = new MatrixKey[Eliminated.Length + 1];
				MatrixKey[] forced     = new MatrixKey[Forced    .Length + i];
				
				Eliminated.CopyTo(eliminated, 0);
				Forced    .CopyTo(forced,     0);

				eliminated[Eliminated.Length] = iremaining[i];

				for (int k = 0; k < i; k++) {
					forced[Forced.Length + k] = iremaining[k];
				}

				children.Add(new MurtyNode(eliminated, forced));
			}

			return children.ToArray();
		}
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

	public IEnumerator<KeyValuePair<double, T>> GetEnumerator()
	{
		return data.GetEnumerator();
	}

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
	public int I { get; private set; }
	public int K { get; private set; }

	public MatrixKey(int i, int k)
	{
		I = i;
		K = k;
	}
}
}
