import heapq
import itertools
import math


class PriorityQueue:
    """Adapted from the implementation notes on https://docs.python.org/3.8/library/heapq.html"""
    def __init__(self):
        self.heap = []
        self.entry_finder = {}
        self.REMOVED = '<removed-task>'  # placeholder for a removed task
        self.counter = itertools.count()

    def push(self, task, priority=0):
        'Add a new task or update the priority of an existing task'
        if task in self.entry_finder:
            self.remove(task)
        count = next(self.counter)
        entry = [priority, count, task]
        self.entry_finder[task] = entry
        heapq.heappush(self.heap, entry)

    def remove(self, task):
        'Mark an existing task as REMOVED.  Raise KeyError if not found.'
        entry = self.entry_finder.pop(task)
        entry[-1] = self.REMOVED

    def pop(self):
        'Remove and return the lowest priority task. Raise KeyError if empty.'
        while self.heap:
            priority, count, task = heapq.heappop(self.heap)
            if task is not self.REMOVED:
                del self.entry_finder[task]
                return task
        raise KeyError('pop from an empty priority queue')

    # Some useful additional methods not taken from Python docs
    def contains(self, state):
        return state in self.entry_finder

    def length(self):
        return len(self.heap)

    def get_priority(self, task):
        """Returns math.inf if the task is not in the queue"""
        if task in self.entry_finder:
            return self.entry_finder[task][0]
        else:
            return math.inf
