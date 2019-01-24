"""
@file    __init__.py
@author  Daniel Krajzewicz
@author  Laura Bieker
@author  Karol Stosiek
@author  Michael Behrisch
@author  Jakob Erdmann
@date    2008-03-27
@version $Id: __init__.py 17235 2014-11-03 10:53:02Z behrisch $

This file contains a content handler for parsing sumo network xml files.
It uses other classes from this module to represent the road network.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2014 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""

import os, sys
import optparse
import subprocess
import random
from Queue import PriorityQueue
import xml.sax
from xml.sax import saxutils, parse, make_parser, handler
from copy import copy
from itertools import *

SUMO_HOME = "/home/hayder/sumo-0.22.0"
try:
    sys.path.append(os.path.join(SUMO_HOME, "tools"))
    # import the library
    import sumolib
    from sumolib import checkBinary
    from sumolib.net import Net
    from sumolib.net import NetReader
    from sumolib.net import Lane
    from sumolib.net import Edge
    from sumolib.net import Node
    from sumolib.net import Connection
    from sumolib.net import Roundabout
       
except ImportError:
    sys.exit("please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

graph = sumolib.net.readNet('Dijkstra1.net.xml')

import traci
# the port used for communicating with your sumo instance
PORT = 8873

class priorityDictionary(dict):
    def __init__(self):
        '''Initialize priorityDictionary by creating binary heap
            of pairs (value,key).  Note that changing or removing a dict entry will
            not remove the old pair from the heap until it is found by smallest() or
            until the heap is rebuilt.'''
        self.__heap = []
        dict.__init__(self)

    def smallest(self):
        '''Find smallest item after removing deleted items from heap.'''
        if len(self) == 0:
            raise IndexError, "smallest of empty priorityDictionary"
        heap = self.__heap
        while heap[0][1] not in self or self[heap[0][1]] != heap[0][0]:
            lastItem = heap.pop()
            insertionPoint = 0
            while 1:
                smallChild = 2*insertionPoint+1
                if smallChild+1 < len(heap) and \
                        heap[smallChild][0] > heap[smallChild+1][0]:
                    smallChild += 1
                if smallChild >= len(heap) or lastItem <= heap[smallChild]:
                    heap[insertionPoint] = lastItem
                    break
                heap[insertionPoint] = heap[smallChild]
                insertionPoint = smallChild
        return heap[0][1]

    def __iter__(self):
        '''Create destructive sorted iterator of priorityDictionary.'''
        def iterfn():
            while len(self) > 0:
                x = self.smallest()
                yield x
                del self[x]
        return iterfn()

    def __setitem__(self,key,val):
        '''Change value stored in dictionary and add corresponding
            pair to heap.  Rebuilds the heap if the number of deleted items grows
            too large, to avoid memory leakage.'''
        dict.__setitem__(self,key,val)
        heap = self.__heap
        if len(heap) > 2 * len(self):
            self.__heap = [(v,k) for k,v in self.iteritems()]
            self.__heap.sort()  # builtin sort likely faster than O(n) heapify
        else:
            newPair = (val,key)
            insertionPoint = len(heap)
            heap.append(None)
            while insertionPoint > 0 and val < heap[(insertionPoint-1)//2][0]:
                heap[insertionPoint] = heap[(insertionPoint-1)//2]
                insertionPoint = (insertionPoint-1)//2
            heap[insertionPoint] = newPair

    def setdefault(self,key,val):
        '''Reimplement setdefault to call our customized __setitem__.'''
        if key not in self:
            self[key] = val
        return self[key]

    def update(self, other):
        for key in other.keys():
            self[key] = other[key]

def Dijkstra(graph, start, end=None):
        D = {}	# dictionary of final distances
	P = {}	# dictionary of predecessors
	Q = priorityDictionary()	# estimated distances of non-final vertices
	Q[start] = 0
        edge = graph.getEdges()     
	for vertex in Q:
		D[vertex] = Q[vertex]
		if vertex == end: break
		
		for edge in vertex.getOutgoing():
			vwLength = D[vertex] + edge.getLength() 
			if edge in D:
				if vwLength < D[edge]:
					raise ValueError, "Dijkstra: found better path to already-final vertex"
			elif edge not in Q or vwLength < Q[edge]:
				Q[edge] = vwLength
				P[edge] = vertex
	
	return (D,P)
        
			
def shortestPath(graph, start, end):
	"""
	Find a single shortest path from the given start vertex to the given end vertex.
	The input has the same conventions as Dijkstra().
	The output is a list of the vertices in order along the shortest path.
	"""
        start = graph.getEdge(start)
        end = graph.getEdge(end)
	D,P = Dijkstra(graph, start, end)
	Path = []
	while 1:
		Path.append(end)
		if end == start: break
		end = P[end]
	Path.reverse()
	return Path

def main():
    traci.init(PORT)
    route = shortestPath(graph, '1', '10')
    edges = [str (edge.getID()) for edge in route] 
    #create the new route for vehicle
    traci.route.add("0", edges)
    #assign the new route for vehicle with id vehicle1
    traci.vehicle.add("vehicle0","0",-2,0,10.0)   
    traci.close()
    sys.stdout.flush() 
    

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
       sumoBinary = checkBinary('sumo')
    else:
       sumoBinary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    sumoProcess = subprocess.Popen([sumoBinary, "-c", "dijkstra.sumo.cfg", "--tripinfo-output", "tripinfo.xml", "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)
    main()
    sumoProcess.wait()