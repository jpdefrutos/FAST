#pragma once

#include "FAST/Data/DataObject.hpp"

namespace fast {
/**
* \brief A graph data object
* 
* \ingroup data graph
*/
	class FAST_EXPORT Graph : public DataObject {
		FAST_DATA_OBJECT(Graph)
	public:
		FAST_CONSTRUCTOR(Graph,
			VectorXui, nodes,,
			MatrixXf, edges,,
		);
		void makeAdjacencyMatrix();
		
		int getNode(int nodeId);
		std::array<GraphNode> getParentNode(int nodeId);
		std::array<GraphNode> getChildNodes(int nodeId);
		MatrixXf getAdjacencyMatrix();

	private:
		std::array<GraphNode> m_nodes;
		MatrixXf m_edges;
		MatrixXf m_adjacencyMatrix;
		bool m_initialized;
		int m_numNodes;
		int m_numEdges;

		std::array<GraphNode> buildGraph(VectorXui nodes, MatrixXf edges);
	};

	class FAST_EXPORT GraphNode : public DataObject {
		FAST_DATA_OBJECT(GraphNode)
	public:
		FAST_CONSTRUCTOR(GrapNode,
			int, nodeId, ,
			MatrixXf, edges, ,
			void* nodeProperties, =void*,
			);

		void* getProperties();
		int getNodeId();
		MatrixXf getNodeEdges();

	private:
		int m_nodeId;
		MatrixXf m_edges;
		void* m_properties;
	};
}