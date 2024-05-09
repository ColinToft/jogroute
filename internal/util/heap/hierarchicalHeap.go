package heap

import (
	"errors"
)

// A HierarchicalHeap implementation using the container/heap package.
// Heap functions are based on the original Go implementation in the container/heap package.
// The original Go implementation is licensed under the BSD 3-Clause License, allowing use with modification, provided that the following is included:

// Copyright (c) 2009 The Go Authors. All rights reserved.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Priority queue interface implementation (uses slice)
// An Item is something we manage in a priority queue.
type Item struct {
	value    int // The value of the item; arbitrary.
	priority int // The priority of the item in the queue.
}

// Init establishes the heap invariants required by the other routines in this package.
// Init is idempotent with respect to the heap invariants
// and may be called whenever the heap invariants may have been invalidated.
// The complexity is O(n) where n = len(h).
func (h PriorityQueue) Init() {
	// heapify
	n := len(h)
	for i := n/2 - 1; i >= 0; i-- {
		h.down(i, n)
	}
}

// Push pushes the element x onto the heap.
// The complexity is O(log n) where n = len(h).
func (h *PriorityQueue) Push(x *Item) {
	*h = append(*h, x)
	h.up(len(*h) - 1)
}

// Pop removes and returns the minimum element (according to Less) from the heap.
// The complexity is O(log n) where n = len(h).
// Pop is equivalent to Remove(h, 0).
func (h *PriorityQueue) Pop() *Item {
	old := *h
	n := len(old) - 1
	old[0], old[n] = old[n], old[0]
	h.down(0, n)

	m := len(old)
	item := old[m-1]
	old[m-1] = nil // avoid memory leak
	*h = old[0 : m-1]

	return item
}

// Remove removes and returns the element at index i from the heap.
// The complexity is O(log n) where n = len(h).
func (h PriorityQueue) Remove(i int) any {
	n := len(h) - 1
	if n != i {
		h[i], h[n] = h[n], h[i]
		if !h.down(i, n) {
			h.up(i)
		}
	}
	return h.Pop()
}

// Fix re-establishes the heap ordering after the element at index i has changed its value.
// Changing the value of the element at index i and then calling Fix is equivalent to,
// but less expensive than, calling Remove(h, i) followed by a Push of the new value.
// The complexity is O(log n) where n = len(h).
func (h PriorityQueue) Fix(i int) {
	if !h.down(i, len(h)) {
		h.up(i)
	}
}

func (h PriorityQueue) up(j int) {
	for {
		i := (j - 1) / 2 // parent
		if i == j || h[j].priority >= h[i].priority {
			break
		}
		h[i], h[j] = h[j], h[i]
		j = i
	}
}

func (h PriorityQueue) down(i0, n int) bool {
	i := i0
	for {
		j1 := 2*i + 1
		if j1 >= n || j1 < 0 { // j1 < 0 after int overflow
			break
		}
		j := j1 // left child
		if j2 := j1 + 1; j2 < n && h[j2].priority < h[j1].priority {
			j = j2 // = 2*i + 2  // right child
		}
		if h[j].priority >= h[i].priority {
			break
		}
		h[i], h[j] = h[j], h[i]
		i = j
	}
	return i > i0
}

// A PriorityQueue implements heap.Interface and holds Items.
type PriorityQueue []*Item

// Hierarchical Heap is done as a list of heaps
type HierarchicalHeap struct {
	heaps           []*PriorityQueue
	current         int // The index of the current heap we are popping from
	min             int // The smallest priority value possible
	max             int // The largest priority value possible
	priorityPerHeap int // The range of priorities per heap

	len int // The number of items in the hierarchical heap
}

// NewHierarchicalHeap creates a new hierarchical heap
func NewHierarchicalHeap(heapCount, min, max int) (*HierarchicalHeap, error) {
	if min >= max {
		return nil, errors.New("min must be less than max")
	}

	priorityPerHeap := (max - min) / heapCount

	heaps := make([]*PriorityQueue, heapCount)
	for i := 0; i < heapCount; i++ {
		heaps[i] = &PriorityQueue{}
		heaps[i].Init()
	}

	return &HierarchicalHeap{heaps: heaps, current: len(heaps), min: min, max: max, priorityPerHeap: priorityPerHeap}, nil
}

// Enqueue adds an item to the hierarchical heap
func (hh *HierarchicalHeap) Enqueue(value, priority int) {
	heapIndex := (priority - hh.min) / hh.priorityPerHeap
	hh.heaps[heapIndex].Push(&Item{value: value, priority: priority})

	// Update the current heap if necessary
	if heapIndex < hh.current {
		hh.current = heapIndex
	}

	hh.len++

	// fmt.Printf("Pushing %d with priority %d to heap %d\n", value, priority, heapIndex)
}

// Dequeue removes an item from the hierarchical heap
func (hh *HierarchicalHeap) Dequeue() int {
	if hh.current >= len(hh.heaps) {
		return -1
	}

	// fmt.Printf("Popping from heap %d\n", hh.current)

	// Pop from the current heap
	item := hh.heaps[hh.current].Pop()

	hh.len--

	// Update the current heap if necessary
	if hh.len > 0 {
		for len(*hh.heaps[hh.current]) == 0 {
			hh.current++
		}
	} else {
		hh.current = len(hh.heaps)
	}

	return item.value
}
