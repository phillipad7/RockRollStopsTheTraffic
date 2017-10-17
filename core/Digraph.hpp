// Digraph.hpp
//
// ICS 46 Spring 2017
// Project #4: Rock and Roll Stops the Traffic
//
// This header file declares a class template called Digraph, which is
// intended to implement a generic directed graph.  The implementation
// uses the adjacency lists technique, so each vertex stores a linked
// list of its outgoing edges.
//
// Along with the Digraph class template is a class DigraphException
// and a couple of utility structs that aren't generally useful outside
// of this header file.
//
// In general, directed graphs are all the same, except in the sense
// that they store different kinds of information about each vertex and
// about each edge; these two types are the type parameters to the
// Digraph class template.

#ifndef DIGRAPH_HPP
#define DIGRAPH_HPP

#include <functional>
#include <list>
#include <map>
#include <utility>
#include <vector>
#include <algorithm>

#include <iostream>
#include <queue>

#define INFINI std::numeric_limits<double>::max() //DBL_MAX //INT32_MAX


// DigraphExceptions are thrown from some of the member functions in the
// Digraph class template, so that exception is declared here, so it
// will be available to any code that includes this header file.

class DigraphException
{
public:
    DigraphException(const std::string& reason): reason_{reason} { }

    std::string reason() const { return reason_; }

private:
    std::string reason_;
};



// A DigraphEdge lists a "from vertex" (the number of the vertex from which
// the edge points), a "to vertex" (the number of the vertex to which the
// edge points), and an EdgeInfo object.  Because different kinds of Digraphs
// store different kinds of edge information, DigraphEdge is a template
// struct.

template <typename EdgeInfo>
struct DigraphEdge
{
    int fromVertex;
    int toVertex;
    EdgeInfo einfo;
};



// A DigraphVertex includes two things: a VertexInfo object and a list of
// its outgoing edges.  Because different kinds of Digraphs store different
// kinds of vertex and edge information, DigraphVertex is a template struct.

template <typename VertexInfo, typename EdgeInfo>
struct DigraphVertex
{
    VertexInfo vinfo;
    std::list<DigraphEdge<EdgeInfo>> edges;     // list of DigraphEdge<EdgeInfo>
                                                // EdgeInfo{int fromVer, int toVer, EdgeInfo einfo}
};



// Digraph is a class template that represents a directed graph implemented
// using adjacency lists.  It takes two type parameters:
//
// * VertexInfo, which specifies the kind of object stored for each vertex
// * EdgeInfo, which specifies the kind of object stored for each edge
//
// You'll need to implement the member functions declared here; each has a
// comment detailing how it is intended to work.
//
// Each vertex in a Digraph is identified uniquely by a "vertex number".
// Vertex numbers are not necessarily sequential and they are not necessarily
// zero- or one-based.

template <typename VertexInfo, typename EdgeInfo>
class Digraph
{
public:
    // The default constructor initializes a new, empty Digraph so that
    // contains no vertices and no edges.
    Digraph();

    // The copy constructor initializes a new Digraph to be a deep copy
    // of another one (i.e., any change to the copy will not affect the
    // original).
    Digraph(const Digraph& d);

    // The move constructor initializes a new Digraph from an expiring one.
    Digraph(Digraph&& d);

    // The destructor deallocates any memory associated with the Digraph.
    ~Digraph();

    // The assignment operator assigns the contents of the given Digraph
    // into "this" Digraph, with "this" Digraph becoming a separate, deep
    // copy of the contents of the given one (i.e., any change made to
    // "this" Digraph afterward will not affect the other).
    Digraph& operator=(const Digraph& d);

    // The move assignment operator assigns the contents of an expiring
    // Digraph into "this" Digraph.
    Digraph& operator=(Digraph&& d);

    // vertices() returns a std::vector containing the vertex numbers of
    // every vertex in this Digraph.
    std::vector<int> vertices() const;

    // edges() returns a std::vector of std::pairs, in which each pair
    // contains the "from" and "to" vertex numbers of an edge in this
    // Digraph.  All edges are included in the std::vector.
    std::vector<std::pair<int, int>> edges() const;

    // This overload of edges() returns a std::vector of std::pairs, in
    // which each pair contains the "from" and "to" vertex numbers of an
    // edge in this Digraph.  Only edges outgoing from the given vertex
    // number are included in the std::vector.  If the given vertex does
    // not exist, a DigraphException is thrown instead.
    std::vector<std::pair<int, int>> edges(int vertex) const;

    // vertexInfo() returns the VertexInfo object belonging to the vertex
    // with the given vertex number. If that vertex does not exist, a
    // DigraphException is thrown instead.
    VertexInfo vertexInfo(int vertex) const;

    // edgeInfo() returns the EdgeInfo object belonging to the edge
    // with the given "from" and "to" vertex numbers.  If either of those
    // vertices does not exist *or* if the edge does not exist, a
    // DigraphException is thrown instead.
    EdgeInfo edgeInfo(int fromVertex, int toVertex) const;

    // addVertex() adds a vertex to the Digraph with the given vertex
    // number and VertexInfo object.  If there is already a vertex in
    // the graph with the given vertex number, a DigraphException is
    // thrown instead.
    void addVertex(int vertex, const VertexInfo& vinfo);

    // addEdge() adds an edge to the Digraph pointing from the given
    // "from" vertex number to the given "to" vertex number, and
    // associates with the given EdgeInfo object with it.  If one
    // of the vertices does not exist *or* if the same edge is already
    // present in the graph, a DigraphException is thrown instead.
    void addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo);

    // removeVertex() removes the vertex (and all of its incoming
    // and outgoing edges) with the given vertex number from the
    // Digraph.  If the vertex does not exist already, a DigraphException
    // is thrown instead.
    void removeVertex(int vertex);

    // removeEdge() removes the edge pointing from the given "from"
    // vertex number to the given "to" vertex number from the Digraph.
    // If either of these vertices does not exist *or* if the edge
    // is not already present in the graph, a DigraphException is
    // thrown instead.
    void removeEdge(int fromVertex, int toVertex);

    // vertexCount() returns the number of vertices in the graph.
    int vertexCount() const;

    // edgeCount() returns the total number of edges in the graph,
    // counting edges outgoing from all vertices.
    int edgeCount() const;

    // This overload of edgeCount() returns the number of edges in
    // the graph that are outgoing from the given vertex number.
    // If the given vertex does not exist, a DigraphException is
    // thrown instead.
    int edgeCount(int vertex) const;

    // isStronglyConnected() returns true if the Digraph is strongly
    // connected (i.e., every vertex is reachable from every other),
    // false otherwise.
    bool isStronglyConnected() const;

    // findShortestPaths() takes a start vertex number and a function
    // that takes an EdgeInfo object and determines an edge weight.
    // It uses Dijkstra's Shortest Path Algorithm to determine the
    // shortest paths from the start vertex to every other vertex
    // in the graph.  The result is returned as a std::map<int, int>
    // where the keys are vertex numbers and the value associated
    // with each key k is the precedessor of that vertex chosen by
    // the algorithm.  For any vertex without a predecessor (e.g.,
    // a vertex that was never reached, or the start vertex itself),
    // the value is simply a copy of the key.
    std::map<int, int> findShortestPaths(
        int startVertex,
        std::function<double(const EdgeInfo&)> edgeWeightFunc) const;


private:
    // Add whatever member variables you think you need here.  One
    // possibility is a std::map where the keys are vertex numbers
    // and the values are DigraphVertex<VertexInfo, EdgeInfo> objects.


    // You can also feel free to add any additional member functions
    // you'd like (public or private), so long as you don't remove or
    // change the signatures of the ones that already exist.

    std::map<int, DigraphVertex <VertexInfo, EdgeInfo>> gmap;  //EdgeInfo

};


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph()
{
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(const Digraph& d)
{
    gmap.clear();
    gmap.insert(d.gmap.begin(),d.gmap.end());
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(Digraph&& d)
{
    std::swap(gmap, d.gmap);
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::~Digraph()
{
    gmap.clear();
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(const Digraph& d)
{
    gmap.clear();
    gmap.insert(d.gmap.begin(), d.gmap.end());
    return *this;
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(Digraph&& d)
{
    std::swap(gmap, d.gmap);
    return *this;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<int> Digraph<VertexInfo, EdgeInfo>::vertices() const
{
    std::vector<int> vecVer;
    for(int i=0; i<gmap.size(); i++)
    {
        vecVer.push_back(i);
    }
    return vecVer;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges() const
{
    std::vector< std::pair<int, int> > vecPair;

    for(auto mappair: gmap)
    {
        for(auto digEdge: mappair.second.edges)
        {
            vecPair.push_back(std::pair<int, int> (digEdge.fromVertex, digEdge.toVertex) );
        }
    }
    return vecPair;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int> > Digraph<VertexInfo, EdgeInfo>::edges(int vertex) const
{
    
    std::vector< std::pair<int, int> > vecPair;

    for(auto didge: gmap.at(vertex).edges)
    {
        vecPair.push_back(std::pair<int,int>(didge.fromVertex, didge.toVertex));
    }
    return vecPair;
}


template <typename VertexInfo, typename EdgeInfo>
VertexInfo Digraph<VertexInfo, EdgeInfo>::vertexInfo(int vertex) const
{
    // std::map<int, DigraphVertex <VertexInfo, EdgeInfo> > gmap;   // a dictionary, {{frVer:DigraVer},{frVer:DigraVer},}
                                                                    // gmap[frVer].first = fromVertex     
                                                                    // gmap[frVer].second = DigraphVertex
    // struct DigraphVertex {VertexInfo vinfo; std::list<DigraphEdge<EdgeInfo>> edges;
    // struct DigraphEdge {int fromVertex;   int toVertex;    EdgeInfo einfo;};
    // EdgeInfo einfo{int fromVer, int toVer, EdgeInfo einfo}
                                                                    // gmap.at(fromVertex).edges = std::list<DigraphEdge<EdgeInfo> >
                                                                    // for i in gmap[fromVertex].edges: // looping DigraphEdge  in edges
                                                                    //  i = DigraphEdge<EdgeInfo>
                                                                    //  i.fromVertex = int      i.toVertex = int    i.einfo = EdgeInfo
    
    DigraphException exception("Vertex does not exist");

    auto fd = gmap.find(vertex);
    if (fd !=gmap.end())
    {
        return gmap.at(vertex).vinfo;
    }
    throw exception;
}


template <typename VertexInfo, typename EdgeInfo>
EdgeInfo Digraph<VertexInfo, EdgeInfo>::edgeInfo(int fromVertex, int toVertex) const
{
    
    DigraphException exception("Edge not found");

    for(auto edge: gmap.at(fromVertex).edges)
    {
        if(edge.toVertex == toVertex)
        {
            return edge.einfo;
        }
    }
    throw exception;
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addVertex(int vertex, const VertexInfo& vinfo)
{
    
    DigraphException exception("Vertex is already existed");

    std::pair<int, DigraphVertex <VertexInfo, EdgeInfo>> newVertex;  //EdgeInfo
    newVertex.first = vertex;
    newVertex.second.vinfo = vinfo;
    typename std::map<int, DigraphVertex <VertexInfo, EdgeInfo>>::iterator it;
    
    it = gmap.find(vertex);
    
    if(it == gmap.end())
    {
        gmap.insert(newVertex);
    }
    else
    {
        throw exception;
    }
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo)
{
    DigraphException exception("Edge is already existed");

    DigraphEdge<EdgeInfo> adEdge;
    adEdge.fromVertex = fromVertex;
    adEdge.toVertex = toVertex;
    adEdge.einfo = einfo;
    
    auto fdFromVertex = gmap.find(fromVertex);
    if(fdFromVertex != gmap.end())
    {
        for(auto oneedge: gmap.at(fromVertex).edges)
        {
            if(oneedge.toVertex == toVertex)
            {
                throw exception;
            }
        }
    }

    gmap.at(fromVertex).edges.push_back(adEdge);        // push an edge to the Vertex if the edge was not previously existed
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeVertex(int vertex)
{
    DigraphException exception("Vertex does not exist");

    typename std::map<int, DigraphVertex <VertexInfo, EdgeInfo>>::iterator it;
    
    it = gmap.find(vertex);
    
    if(it != gmap.end())                            // found vertex
    {
        gmap.erase(vertex);                         // erase key vertex from gmap. 
        for(auto mappair: gmap)
        {
            int size = mappair.second.edges.size(); // size of the list of DigraphEdge of each pair{fVer, DigVer}

            for(int i=0; i<size; i++)
            {
                DigraphEdge<EdgeInfo> dedge = mappair.second.edges.back();
                mappair.second.edges.pop_back();
                if(dedge.toVertex != vertex)
                {
                    mappair.second.edges.push_front(dedge);
                }
            }
        }
    }
    else
    {
        throw exception;
    }

}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeEdge(int fromVertex, int toVertex)
{
    // MARK::cannot use std::list.remove() because not yet overloading == sign
    
    DigraphException exception("Edge does not exist");

    typename std::map<int, DigraphVertex <VertexInfo, EdgeInfo>>::iterator it;
    
    it = gmap.find(fromVertex);
    if(it != gmap.end())                                // found fromVertex
    {     
        std::list<DigraphEdge<EdgeInfo>> nList;

        for(auto oneedge: gmap.at(fromVertex).edges)
        {
            if(oneedge.toVertex != toVertex)            // found toVertex
            {
                nList.push_back(oneedge);
            }
        }

        if(nList.size() != gmap.at(fromVertex).edges.size())
        {
            gmap.at(fromVertex).edges = nList;
        }
        else
        {
            throw exception;
        }
    }
    else
    {
        throw exception;
    }
}


// vertexCount() returns the number of vertices in the graph.
template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::vertexCount() const
{
    return gmap.size();
}


// edgeCount() returns the total number of edges in the graph,
// counting edges outgoing from all vertices.
template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount() const
{
    int count = 0;

    for(auto mappair: gmap)
    {
        count += mappair.second.edges.size();
    }
    return count;
}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount(int vertex) const
{
    return gmap.at(vertex).edges.size();
}


template <typename VertexInfo, typename EdgeInfo>
bool Digraph<VertexInfo, EdgeInfo>::isStronglyConnected() const
{
    std::map<int, bool> isVisited;

    for(auto mappair: gmap)
    {
        isVisited[mappair.first] = false;
    }

    for(auto mappair: gmap)
    {
        std::map<int,bool> isViCopy = isVisited;
        int count =0;
        for (auto edge: mappair.second.edges)
        {
            if(isViCopy[edge.toVertex] == 0)
            {
                count++;
                isViCopy[edge.toVertex] =1;
            }
        }
        if(count == vertexCount())
            return true;
    }
    return false;
}

// findShortestPaths() takes 
// a start vertex number and 
// a function that takes an EdgeInfo object and determines an edge weight.
//
// It uses Dijkstra's Shortest Path Algorithm to determine the
// shortest paths from the start vertex to every other vertex in the graph. 

// The result is returned as a std::map<int, int>
// where the keys are vertex numbers and the value associated
// with each key k is the precedessor of that vertex chosen by the algorithm.  
// For any vertex without a predecessor (e.g.,
// a vertex that was never reached, or the start vertex itself),
// the value is simply a copy of the key.


// startVertex = 1;
// {0:1, 2:1, 3:0,...}
template <typename VertexInfo, typename EdgeInfo>
std::map<int, int> Digraph<VertexInfo, EdgeInfo>::findShortestPaths(
    int startVertex,
    std::function<double(const EdgeInfo&)> edgeWeightFunc) const
{
    std::map<int,int> pass;                 // shortest pass to be returned
    std::map<int,double> shortestD;         

    std::map<int,bool> isKnown;
    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<std::pair<double,int>> >  pq;
    // std::priority_queue<std::pair<int,int> > pq;     // need to be sorted from smallest to largest

    for(auto mappair: gmap)
    {
        isKnown[mappair.first] = false;                 // set Kv to false
        shortestD[mappair.first] = INFINI;              // set dv to INFINITY
        pass[mappair.first] = mappair.first;
    }
    shortestD[startVertex] = 0.0;                       // shortestD[startVertex] = 0;

    pq.push(std::make_pair(0, startVertex));

    while(!pq.empty())
    {
        std::pair<double,int> vp = pq.top();            // (priority, index)
        pq.pop();

        if(!isKnown.at(vp.second))
        {
            isKnown.at(vp.second) = true;

            for(auto w: gmap.at(vp.second).edges)
            {               
                if(shortestD.at(w.toVertex) > shortestD.at(vp.second) + edgeWeightFunc(w.einfo))
                {
                    shortestD.at(w.toVertex) = shortestD.at(vp.second) + edgeWeightFunc(w.einfo);
                    
                    pass[w.toVertex] = vp.second;
                    pq.push(std::make_pair(shortestD.at(w.toVertex), w.toVertex));
                }
            }
        }
    }
    return pass;
}


#endif // DIGRAPH_HPP
