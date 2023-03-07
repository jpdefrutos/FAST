#include "Graph.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/Utility.hpp"
#include <unordered_set>
#include <stack>
#include <eigen>

namespace fast {
	/**
	* @brief Graph representation of data
	* 
	* The Graph object contains the collection of nodes and edges connecting these nodes. Each node can be assigned
	* properties of features. The edges can be undirected (1) or undirected (+-1), and weighted representing a cost of traversing 
	* between the two connected nodes.
	* 
	* @param nodes (N, 1) vector of nodes identifiers
	* @param edges (E, 3) matrix of edges, where the first column is the starting node, the second is the destination node, and
	* the third is the value of the edge e.g., 1, -1, or a real value. The following must be true E <= N.
	* 
	* @ingroup data graph
	*/
	Graph::Graph(VectorXui nodes, MatrixXf edges) {
		if (nodes.rows() < edges.rows())
			throw Exception("More edges (" + std::to_string(edges.rows()) + ") than nodes (" + std::to_string(nodes.rows()) + ").");

		m_edges = edges;
		m_numNodes = nodes.rows();
		m_numEdges = edges.rows();
		m_nodes = nodes;
		m_initialized = false;

		this.makeAdjacenyMatrix();
	}

	/**
	* @brief Build the adjacency matrix for the edges and nodes provided
	*
	* @ingroup data graph
	*/
	void Graph::makeAdjacencyMatrix() {
		m_adjancencyMatrix = MatrixXf::Zero(m_numNodes, m_numNodes);

		std::vector<int>::iterator edgesIterator;

		for (auto edge : m_edges.rowwise()) {
			void initialNode = edge[0];
			void finalNode = edge[1];
			void edgeValue = edge[2];
			m_adjacencyMatrix[initialNode, finalNode] = edgeValue;
		}
		m_initialized = true;
	}

	/**
	* @brief Getter for the adjacency matrix
	* 
	* @ingroup data graph
	*/
	MatrixXf Graph::getAdjacencyMatrix() {
		if (!m_initialized)
			this->makeAdjacencyMatrix();
		return m_adjacencyMatrix;
	}


//	// TBD!!
//	int getNode(int nodeId);
//	std::array<GraphNode> getParentNode(int nodeId);
//	std::array<GraphNode> getChildNodes(int nodeId);
//
//	GraphNode::GrapNode(int nodeId, MatrixXf edges);
// 	void* getProperties();
//	int getNodeId();
//	MatrixXf getNodeEdges();
//}
