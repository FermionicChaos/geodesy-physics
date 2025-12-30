#include <geodesy/phys/mesh.h>

#include <unordered_set>
#include <unordered_map>
#include <queue>

namespace geodesy::phys {

	mesh::vertex::vertex() {
		this->Position					= math::vec<float, 3>(0.0f, 0.0f, 0.0f);
		this->Normal					= math::vec<float, 3>(0.0f, 0.0f, 0.0f);
		this->Tangent					= math::vec<float, 3>(0.0f, 0.0f, 0.0f);
		this->Bitangent					= math::vec<float, 3>(0.0f, 0.0f, 0.0f);
		// this->BoneID					= math::vec<uint, 4>(UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX);
		// this->BoneWeight				= math::vec<float, 4>(0.0f, 0.0f, 0.0f, 0.0f);
		this->TextureCoordinate			= math::vec<float, 3>(0.0f, 0.0f, 0.0f);
		this->Color						= math::vec<float, 4>(0.0f, 0.0f, 0.0f, 0.0f);
	}

	mesh::mesh() {
		this->Name						= "";
		this->Mass						= 1.0f;
		this->CenterOfMass 				= { 0.0f, 0.0f, 0.0f };
		this->BoundingRadius			= { 0.0f, 0.0f, 0.0f };
	}

	mesh::vertex mesh::operator[](size_t aIndex) const {
		return this->Vertex[aIndex];
	}

	mesh::vertex& mesh::operator[](size_t aIndex) {
		return this->Vertex[aIndex];
	}

	// Calculates the center of mass of the mesh.
	math::vec<float, 3> mesh::center_of_mass() const {
		math::vec<float, 3> COM = math::vec<float, 3>(0.0f, 0.0f, 0.0f);
		for (const auto& Vertex : this->Vertex) {
			COM += Vertex.Position;
		}
		COM /= static_cast<float>(this->Vertex.size());
		return COM;
	}

	// Determines the bounding radius of the mesh.
	math::vec<float, 3> mesh::bounding_radius() const {
		float MaxRadius = 0.0f;
		math::vec<float, 3> MaxPosition = { 0.0f, 0.0f, 0.0f };
		math::vec<float, 3> COM = this->center_of_mass();
		for (const auto& Vertex : this->Vertex) {
			float Distance = math::length(Vertex.Position - COM);
			if (Distance > MaxRadius) {
				MaxRadius = Distance;
				MaxPosition = Vertex.Position - COM;
			}
		}
		return MaxPosition;
	}

	std::vector<std::pair<math::vec<float, 3>, std::shared_ptr<phys::mesh>>> mesh::split_disconnected_meshes() const {
		std::vector<std::pair<math::vec<float, 3>, std::shared_ptr<phys::mesh>>> SeparatedMeshes;
		
		if (this->Vertex.empty()) {
			return SeparatedMeshes;
		}

		// Build adjacency list - which vertices are connected to which
		std::unordered_map<uint, std::unordered_set<uint>> VertexConnections;
		
		// Determine which index buffer to use
		const bool Use32Bit = !this->Topology.Data32.empty();
		const size_t IndexCount = Use32Bit ? this->Topology.Data32.size() : this->Topology.Data16.size();
		
		// Helper to add bidirectional edge
		auto AddEdge = [&](uint v1, uint v2) {
			VertexConnections[v1].insert(v2);
			VertexConnections[v2].insert(v1);
		};

		// Helper to get index from appropriate buffer
		auto GetIndex = [&](size_t i) -> uint {
			return Use32Bit ? this->Topology.Data32[i] : this->Topology.Data16[i];
		};

		// Parse topology to build vertex connections
		switch (this->Topology.Primitive) {
			case primitive::TRIANGLE: {
				for (size_t i = 0; i + 2 < IndexCount; i += 3) {
					uint v0 = GetIndex(i);
					uint v1 = GetIndex(i + 1);
					uint v2 = GetIndex(i + 2);
					AddEdge(v0, v1);
					AddEdge(v1, v2);
					AddEdge(v2, v0);
				}
				break;
			}
			case primitive::TRIANGLE_STRIP: {
				for (size_t i = 0; i + 2 < IndexCount; i++) {
					uint v0 = GetIndex(i);
					uint v1 = GetIndex(i + 1);
					uint v2 = GetIndex(i + 2);
					AddEdge(v0, v1);
					AddEdge(v1, v2);
					AddEdge(v2, v0);
				}
				break;
			}
			case primitive::TRIANGLE_FAN: {
				uint v0 = GetIndex(0);
				for (size_t i = 1; i + 1 < IndexCount; i++) {
					uint v1 = GetIndex(i);
					uint v2 = GetIndex(i + 1);
					AddEdge(v0, v1);
					AddEdge(v1, v2);
					AddEdge(v2, v0);
				}
				break;
			}
			case primitive::LINE: {
				for (size_t i = 0; i + 1 < IndexCount; i += 2) {
					uint v0 = GetIndex(i);
					uint v1 = GetIndex(i + 1);
					AddEdge(v0, v1);
				}
				break;
			}
			case primitive::LINE_STRIP: {
				for (size_t i = 0; i + 1 < IndexCount; i++) {
					uint v0 = GetIndex(i);
					uint v1 = GetIndex(i + 1);
					AddEdge(v0, v1);
				}
				break;
			}
			case primitive::POINT: {
				// Points don't connect, each is its own component
				for (size_t i = 0; i < IndexCount; i++) {
					uint v = GetIndex(i);
					VertexConnections[v]; // Ensure entry exists
				}
				break;
			}
		}

		// Find connected components using BFS
		std::unordered_set<uint> Visited;
		std::vector<std::vector<uint>> Components;

		for (const auto& [VertexIndex, _] : VertexConnections) {
			if (Visited.count(VertexIndex)) continue;

			// BFS to find all vertices in this component
			std::vector<uint> Component;
			std::queue<uint> Queue;
			Queue.push(VertexIndex);
			Visited.insert(VertexIndex);

			while (!Queue.empty()) {
				uint Current = Queue.front();
				Queue.pop();
				Component.push_back(Current);

				for (uint Neighbor : VertexConnections[Current]) {
					if (!Visited.count(Neighbor)) {
						Visited.insert(Neighbor);
						Queue.push(Neighbor);
					}
				}
			}

			Components.push_back(Component);
		}

		// Get original mesh center of mass for offset calculation
		math::vec<float, 3> OriginalCOM = this->CenterOfMass;

		// Create a new mesh for each component
		for (size_t ComponentIdx = 0; ComponentIdx < Components.size(); ComponentIdx++) {
			auto NewMesh = std::make_shared<mesh>();
			const auto& Component = Components[ComponentIdx];

			// Map old vertex indices to new indices
			std::unordered_map<uint, uint> OldToNewIndex;
			for (size_t i = 0; i < Component.size(); i++) {
				uint OldIndex = Component[i];
				OldToNewIndex[OldIndex] = static_cast<uint>(i);
				NewMesh->Vertex.push_back(this->Vertex[OldIndex]);
			}

			// Copy and remap topology
			NewMesh->Topology.Primitive = this->Topology.Primitive;
			
			if (Use32Bit) {
				for (uint OldIndex : this->Topology.Data32) {
					if (OldToNewIndex.count(OldIndex)) {
						NewMesh->Topology.Data32.push_back(OldToNewIndex[OldIndex]);
					}
				}
			} else {
				for (ushort OldIndex : this->Topology.Data16) {
					if (OldToNewIndex.count(OldIndex)) {
						NewMesh->Topology.Data16.push_back(static_cast<ushort>(OldToNewIndex[OldIndex]));
					}
				}
			}

			// Calculate properties
			NewMesh->Name = this->Name + "_part" + std::to_string(ComponentIdx);
			NewMesh->CenterOfMass = NewMesh->center_of_mass();
			NewMesh->BoundingRadius = NewMesh->bounding_radius();
			NewMesh->Mass = this->Mass * (static_cast<float>(Component.size()) / static_cast<float>(this->Vertex.size()));

			// Calculate offset from original mesh origin to separated mesh center of mass
			math::vec<float, 3> Offset = NewMesh->CenterOfMass - OriginalCOM;

			SeparatedMeshes.push_back({Offset, NewMesh});
		}

		return SeparatedMeshes;
	}

}
