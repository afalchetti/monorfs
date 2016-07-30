// GraphCombinatoricsTest.cs
// Unit tests for the graph combinatorics algorithms
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
using System.Linq;

using NUnit.Framework;

using AE = monorfs.ArrayExtensions;
using GC = monorfs.GraphCombinatorics;
using SparseMatrix = monorfs.SparseMatrix<double>;
using SparseItem   = monorfs.SparseItem<double>;

namespace monorfs.Test
{
/// <summary>
/// GraphCombinatorics unit tests.
/// </summary>
[TestFixture]
class GraphCombinatoricsTest
{
	SparseMatrix threecomp;

	[SetUp]
	public void Setup()
	{
		threecomp = new SparseMatrix(6, 6);

		threecomp[0, 0] = 1;
		threecomp[1, 0] = 1;
		threecomp[1, 1] = 1;
		threecomp[2, 2] = 1;
		threecomp[2, 3] = 1;
		threecomp[2, 4] = 1;
		threecomp[3, 3] = 1;
		threecomp[4, 4] = 1;
		threecomp[5, 5] = 1;
	}

	[Test]
	public void ConnectedComponentsEmpty()
	{
		SparseMatrix empty = new SparseMatrix(100, 100);
		
		Console.WriteLine(empty.ToStringFull());
		Console.WriteLine("");
		Console.WriteLine(string.Join("\n|------\n", (GC.ConnectedComponents(empty).ConvertAll(m => m.ToStringFull()))));
		Console.WriteLine("");
		Assert.AreEqual(0, GC.ConnectedComponents(empty).Count);
	}

	[Test]
	public void ConnectedComponentsOne()
	{
		SparseMatrix full = new SparseMatrix(10, 10);

		for (int i = 0; i < 10; i++) {
		for (int k = 0; k < 10; k++) {
			full[i, k] = 1;
		}
		}
		
		Console.WriteLine(full.ToStringFull());
		Console.WriteLine("");
		Console.WriteLine(string.Join("\n|------\n", (GC.ConnectedComponents(full).ConvertAll(m => m.ToStringFull()))));
		Console.WriteLine("");
		Assert.AreEqual(1, GC.ConnectedComponents(full).Count);
	}

	[Test]
	public void ConnectedComponentsCount()
	{
		SparseMatrix matrix = threecomp;

		Console.WriteLine("1st______");
		Console.WriteLine(matrix.ToStringFull());
		Console.WriteLine("");
		Console.WriteLine(string.Join("\n|------\n", (GC.ConnectedComponents(matrix).ConvertAll(m => m.ToStringFull()))));
		Console.WriteLine("");
		Assert.AreEqual(3, GC.ConnectedComponents(matrix).Count);
		
		matrix[1, 2] = 1;

		Console.WriteLine("2nd______");
		Console.WriteLine(matrix.ToStringFull());
		Console.WriteLine("");
		Console.WriteLine(string.Join("\n|------\n", (GC.ConnectedComponents(matrix).ConvertAll(m => m.ToStringFull()))));
		Console.WriteLine("");
		Assert.AreEqual(2, GC.ConnectedComponents(matrix).Count);
		
		matrix[5, 4] = 1;
		
		Console.WriteLine("3rd______");
		Console.WriteLine(matrix.ToStringFull());
		Console.WriteLine("");
		Console.WriteLine(string.Join("\n|------\n", (GC.ConnectedComponents(matrix).ConvertAll(m => m.ToStringFull()))));
		Console.WriteLine("");
		Assert.AreEqual(1, GC.ConnectedComponents(matrix).Count);

	}

	[Test]
	public void ConnectedComponentsPartition()
	{
		Console.WriteLine(threecomp.ToStringFull());
		Console.WriteLine("");
		Console.WriteLine(string.Join("\n|------\n", (GC.ConnectedComponents(threecomp).ConvertAll(m => m.ToStringFull()))));
		Console.WriteLine("");
		List<SparseMatrix> components = GC.ConnectedComponents(threecomp);

		Assert.AreEqual(threecomp.Count, components.Sum(m => m.Count));

		SparseMatrix union = new SparseMatrix();
		SparseMatrix inter = new SparseMatrix();

		foreach (SparseMatrix component in components) {
		foreach (var item in component) {
			union[item.I, item.K] = item.Value;
		}
		}

		Assert.IsTrue(threecomp.Equals(union));
		
		var enumerator = components.GetEnumerator();

		enumerator.MoveNext();
		SparseMatrix current, previous;
		SparseMatrix first = enumerator.Current;

		previous = first;

		while (enumerator.MoveNext()) {
			current = enumerator.Current;

			inter = new SparseMatrix(current.Width, current.Height);
			foreach (var item in current) {
				if (previous.Defines(item.I, item.K)) {
					inter[item.I, item.K] = item.Value;
				}
			}
			
			Assert.AreEqual(0, inter.Count);

			previous = current;
		}
	}

	[Test]
	public void AssignmentValue1()
	{
		int[] matches = new int[6];
		AE.Fill(matches, i => i);

		Assert.AreEqual(6, GC.AssignmentValue(threecomp, matches));
	}

	[Test]
	public void AssignmentValue2()
	{
		int[] matches = new int[6];
		
		matches[0] = 1;
		matches[1] = 0;
		matches[2] = 4;
		matches[3] = 0;
		matches[4] = 4;
		matches[5] = 5;

		threecomp[1,  0] = 100;

		Assert.AreEqual(103, GC.AssignmentValue(threecomp, matches));
	}

	[Test]
	public void LinearAssignmentUnique()
	{
		SparseMatrix profit = new SparseMatrix(4, 4);

		for (int i = 0; i < 10; i++) {
			profit[i, i] = (i + 1) / 2.0;
		}

		int[] best     = GC.LinearAssignment(profit);
		int[] expected = new int[10];
		AE.Fill(expected, i => i);

		Assert.IsTrue(expected.SequenceEqual(best));
	}

	[Test]
	public void LinearAssignment1()
	{
		SparseMatrix profit = new SparseMatrix(3, 3);
		
		profit[0, 0] = 6; profit[0, 1] = 8; profit[0, 2] = 5;
		profit[1, 0] = 7; profit[1, 1] = 3; profit[1, 2] = 4;
		profit[2, 0] = 9; profit[2, 1] = 8; profit[2, 2] = 7;

		int[] best     = GC.LinearAssignment(profit);
		int[] expected = new int[3] {1, 0, 2};

		Assert.IsTrue(expected.SequenceEqual(best));
	}

	[Test]
	public void LinearAssignment2()
	{
		SparseMatrix profit = new SparseMatrix(3, 3);
		
		                  profit[0, 1] = 2; profit[0, 2] = 5;
		profit[1, 0] = 3;                   profit[1, 2] = 6;
		profit[2, 0] = 1; profit[2, 1] = 2;

		int[] best     = GC.LinearAssignment(profit);
		int[] expected = new int[3] {2, 0, 1};

		Assert.IsTrue(expected.SequenceEqual(best));
	}

	[Test]
	public void LinearAssignment3()
	{
		threecomp[4, 2] = 3;
		threecomp.RemoveAt(2, 2);
		int[] best     = GC.LinearAssignment(threecomp);
		int[] expected = new int[6] {0, 1, 4, 3, 2, 5};

		Assert.IsTrue(expected.SequenceEqual(best));
	}

	[Test]
	public void LexicographicalPairingFull()
	{
		SparseMatrix profit = new SparseMatrix(3, 3);
		
		profit[0, 0] = 6; profit[0, 1] = 8; profit[0, 2] = 5;
		profit[1, 0] = 7; profit[1, 1] = 3; profit[1, 2] = 4;
		profit[2, 0] = 9; profit[2, 1] = 8; profit[2, 2] = 7;

		var         pairing = GC.LexicographicalPairing(profit, 3);
		List<int[]> expected = new List<int[]>();
		
		expected.Add(new int[3] {0, 1, 2});
		expected.Add(new int[3] {0, 2, 1});
		expected.Add(new int[3] {1, 0, 2});
		expected.Add(new int[3] {1, 2, 0});
		expected.Add(new int[3] {2, 0, 1});
		expected.Add(new int[3] {2, 1, 0});

		Assert.AreEqual(expected.Count, pairing.Count());

		int h = 0;
		foreach (Tuple<int[], double> assignment in pairing) {
			Assert.IsTrue(expected[h++].SequenceEqual(assignment.Item1));
		}
	}

	[Test]
	public void LexicographicalPairingNoDuplicates()
	{
		SparseMatrix profit = new SparseMatrix(3, 3);
		
		profit[0, 0] = 6; profit[0, 1] = 8; profit[0, 2] = 5;
		profit[1, 0] = 7; profit[1, 1] = 3; profit[1, 2] = 4;
		profit[2, 0] = 9; profit[2, 1] = 8; profit[2, 2] = 7;

		var         pairing  = GC.LexicographicalPairing(profit, 1);
		List<int[]> expected = new List<int[]>();
		
		expected.Add(new int[3] {0, 2, 1});
		expected.Add(new int[3] {1, 2, 0});
		expected.Add(new int[3] {2, 1, 0});
		
		Assert.AreEqual(expected.Count, pairing.Count());

		int h = 0;
		foreach (Tuple<int[], double> assignment in pairing) {
			Assert.IsTrue(expected[h++].SequenceEqual(assignment.Item1));
		}
	}

	[Test]
	public void MurtyNodeChildrenWithDuplicates()
	{
		MatrixKey[] forced     = {new MatrixKey(1, 1)};
		MatrixKey[] eliminated = {new MatrixKey(0, 2)};

		MurtyNode node  = new MurtyNode(forced, eliminated);
		node.Assignment = new int[5] {0, 1, 2, 3, 4};

		Console.WriteLine("parent___");
		Console.WriteLine(node);
		Console.WriteLine("");

		MurtyNode[] children = node.Children;
		MurtyNode[] expected = { new MurtyNode(new MatrixKey[] {new MatrixKey(1, 1)}, new MatrixKey[] {new MatrixKey(0, 2), new MatrixKey(0, 0)}),
		                         new MurtyNode(new MatrixKey[] {new MatrixKey(1, 1), new MatrixKey(0, 0)}, new MatrixKey[] {new MatrixKey(0, 2), new MatrixKey(2, 2)}),
		                         new MurtyNode(new MatrixKey[] {new MatrixKey(1, 1), new MatrixKey(0, 0), new MatrixKey(2, 2)}, new MatrixKey[] {new MatrixKey(0, 2), new MatrixKey(3, 3)})};

		Assert.AreEqual(expected.Length, children.Length);
		
		Console.WriteLine("children___");
		Console.WriteLine(string.Join(", ", Array.ConvertAll(children, i => i.ToString())));

		foreach (var child in expected) {
			Assert.Contains(child, children);
		}
	}

	[Test]
	public void MurtyNodeChildrenNone()
	{
		MatrixKey[] forced     = {new MatrixKey(0, 0), new MatrixKey(1, 1), new MatrixKey(2, 2), new MatrixKey(3, 3), new MatrixKey(4, 4)};
		MatrixKey[] eliminated = {new MatrixKey(1, 2)};

		MurtyNode node  = new MurtyNode(forced, eliminated);
		node.Assignment = new int[5] {0, 1, 2, 3, 4};

		Console.WriteLine("parent___");
		Console.WriteLine(node);
		Console.WriteLine("");

		MurtyNode[] children = node.Children;
		
		Console.WriteLine("children___");
		Console.WriteLine(string.Join(", ", Array.ConvertAll(children, i => i.ToString())));

		Assert.AreEqual(0, children.Length);
	}

	[Test]
	public void MurtyPairingFullSmall()
	{
		SparseMatrix profit = new SparseMatrix(3, 3);
		
		profit[0, 0] = 6; profit[0, 1] = 8; profit[0, 2] = 5;
		profit[1, 0] = 7; profit[1, 1] = 3; profit[1, 2] = 4;
		profit[2, 0] = 9; profit[2, 1] = 8; profit[2, 2] = 7;

		var         pairing  = GC.MurtyPairing(profit);
		List<int[]> expected = new List<int[]>();
		
		expected.Add(new int[3] {1, 0, 2});
		expected.Add(new int[3] {1, 2, 0});
		expected.Add(new int[3] {2, 0, 1});
		expected.Add(new int[3] {0, 2, 1});
		expected.Add(new int[3] {2, 1, 0});
		expected.Add(new int[3] {0, 1, 2});
		
		Assert.AreEqual(expected.Count, pairing.Count());

		int h = 0;
		foreach (Tuple<int[], double> assignment in pairing) {
			Assert.IsTrue(expected[h++].SequenceEqual(assignment.Item1));
		}
	}

	[Test]
	public void MurtyPairingUnique()
	{
		SparseMatrix profit = new SparseMatrix(5, 5);

		for (int i = 0; i < 5; i++) {
			profit[i, i] = 1;
		}
		
		var         pairing  = GC.MurtyPairing(profit);
		List<int[]> expected = new List<int[]>();

		expected.Add(new int[5] {0, 1, 2, 3, 4});

		Assert.AreEqual(expected.Count, pairing.Count());

		int h = 0;
		foreach (Tuple<int[], double> assignment in pairing) {
			Assert.IsTrue(expected[h++].SequenceEqual(assignment.Item1));
		}
	}
}
}
