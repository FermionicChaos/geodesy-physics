#include <geodesy/phys/mesh.h>

#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <limits>
#include <cmath>
#include <list>

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
		this->BoundingRadius			= 0.0f;
	}

	mesh::lod_parameters::lod_parameters() {
		this->ReductionFactor			= 0.5f;
		this->TargetVertexCount			= -1;
		this->TargetTriangleCount		= -1;
		this->MaxError					= std::numeric_limits<float>::max();
		this->BoundaryWeight			= 1000.0f;
		this->NormalWeight				= 1.0f;
		this->TextureWeight				= 0.5f;
		this->ColorWeight				= 0.5f;
		this->PreserveTopology			= true;
		this->PreserveBoundaries		= true;
		this->PreserveSeams				= true;
		this->CreaseAngleThreshold		= 30.0f;
		this->Metric					= metric_type::MODIFIED_QEM;
		this->Placement					= placement_policy::OPTIMAL;
		this->MaxIterations				= -1;
		this->RecalculateNormals		= true;
		this->OptimizePositions			= false;
		this->ValidateTopology			= true;
		this->AllowFlippedTriangles		= false;
		this->ErrorBudgetMultiplier		= 2.0f;
	}

	mesh::lod_chain::lod_chain() {
		// Vectors are already initialized by default constructor
	}

	mesh::vertex mesh::operator[](size_t aIndex) const {
		return this->Vertex[aIndex];
	}

	mesh::vertex& mesh::operator[](size_t aIndex) {
		return this->Vertex[aIndex];
	}

	// Calculates the center of mass of the mesh.
	math::vec<float, 3> mesh::calculate_center_of_mass() const {
		math::vec<float, 3> COM = math::vec<float, 3>(0.0f, 0.0f, 0.0f);
		for (const auto& Vertex : this->Vertex) {
			COM += Vertex.Position;
		}
		COM /= static_cast<float>(this->Vertex.size());
		return COM;
	}

	// Determines the bounding radius of the mesh.
	float mesh::calculate_bounding_radius() const {
		float MaxRadius = 0.0f;
		math::vec<float, 3> COM = this->calculate_center_of_mass();
		for (const auto& Vertex : this->Vertex) {
			float Distance = math::length(Vertex.Position - COM);
			if (Distance > MaxRadius) {
				MaxRadius = Distance;
			}
		}
		return MaxRadius;
	}

	// Calculate the inertia tensor of the mesh assuming uniform density.
	// Uses volume integration over tetrahedra formed with origin.
	// Based on: "Computing the Moment of Inertia of a Solid Defined by a Triangle Mesh"
	// by Jonathan Blow, 2004
	math::mat<float, 3, 3> mesh::calculate_inertia_tensor() const {
		if (this->Vertex.empty() || (this->Topology.Data16.empty() && this->Topology.Data32.empty())) {
			// Return identity matrix scaled by mass if no geometry
			return math::mat<float, 3, 3>(
				this->Mass, 0.0f, 0.0f,
				0.0f, this->Mass, 0.0f,
				0.0f, 0.0f, this->Mass
			);
		}

		// Only works with triangle topology
		if (this->Topology.Primitive != primitive::TRIANGLE) {
			// Return identity for non-triangle meshes
			return math::mat<float, 3, 3>(
				this->Mass, 0.0f, 0.0f,
				0.0f, this->Mass, 0.0f,
				0.0f, 0.0f, this->Mass
			);
		}

		const bool Use32Bit = !this->Topology.Data32.empty();
		const size_t IndexCount = Use32Bit ? this->Topology.Data32.size() : this->Topology.Data16.size();
		
		if (IndexCount % 3 != 0) {
			// Invalid triangle list
			return math::mat<float, 3, 3>(
				this->Mass, 0.0f, 0.0f,
				0.0f, this->Mass, 0.0f,
				0.0f, 0.0f, this->Mass
			);
		}

		// Canonical inertia tensor integrals for a tetrahedron
		const float a = 1.0f / 60.0f;
		const float b = 1.0f / 120.0f;

		// Accumulate volume and inertia contributions from each tetrahedron
		float Volume = 0.0f;
		math::mat<float, 3, 3> InertiaTensor = {
			0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f
		};

		for (size_t i = 0; i < IndexCount; i += 3) {
			// Get triangle vertices
			uint i0 = Use32Bit ? this->Topology.Data32[i] : this->Topology.Data16[i];
			uint i1 = Use32Bit ? this->Topology.Data32[i + 1] : this->Topology.Data16[i + 1];
			uint i2 = Use32Bit ? this->Topology.Data32[i + 2] : this->Topology.Data16[i + 2];

			const math::vec<float, 3>& v0 = this->Vertex[i0].Position;
			const math::vec<float, 3>& v1 = this->Vertex[i1].Position;
			const math::vec<float, 3>& v2 = this->Vertex[i2].Position;

			// Signed volume of tetrahedron formed by origin and triangle
			float det = v0[0] * (v1[1] * v2[2] - v1[2] * v2[1])
					  - v0[1] * (v1[0] * v2[2] - v1[2] * v2[0])
					  + v0[2] * (v1[0] * v2[1] - v1[1] * v2[0]);
			
			float tetVolume = det / 6.0f;
			Volume += tetVolume;

			// Accumulate inertia tensor contribution from this tetrahedron
			// Using canonical tetrahedron integration formulas
			for (int j = 0; j < 3; ++j) {
				for (int k = 0; k < 3; ++k) {
					if (j == k) {
						// Diagonal: I_xx = integral(y^2 + z^2), I_yy = integral(x^2 + z^2), I_zz = integral(x^2 + y^2)
						float sum = 0.0f;
						for (int n = 0; n < 3; ++n) {
							if (n != j) {
								sum += a * (v0[n] * v0[n] + v1[n] * v1[n] + v2[n] * v2[n])
									 + b * (v0[n] * v1[n] + v0[n] * v2[n] + v1[n] * v2[n]);
							}
						}
						InertiaTensor(j, k) += tetVolume * sum;
					} else {
						// Off-diagonal: I_xy = -integral(x*y), etc.
						float sum = a * (v0[j] * v0[k] + v1[j] * v1[k] + v2[j] * v2[k])
								  + b * (v0[j] * v1[k] + v0[j] * v2[k] + v0[k] * v1[j]
									   + v0[k] * v2[j] + v1[j] * v2[k] + v1[k] * v2[j]);
						InertiaTensor(j, k) -= tetVolume * sum;
					}
				}
			}
		}

		// Handle zero or negative volume (open mesh or inverted normals)
		if (Volume <= 0.0f) {
			// Fallback to diagonal inertia based on bounding sphere
			float R = this->calculate_bounding_radius();
			float I = 0.4f * this->Mass * R * R; // Sphere approximation
			return math::mat<float, 3, 3>(
				I, 0.0f, 0.0f,
				0.0f, I, 0.0f,
				0.0f, 0.0f, I
			);
		}

		// Scale by mass (assumes uniform density = mass / volume)
		float density = this->Mass / Volume;
		InertiaTensor = InertiaTensor * density;

		return InertiaTensor;
	}

	std::vector<std::shared_ptr<phys::mesh>> mesh::split_disconnected_meshes() const {
		std::vector<std::shared_ptr<phys::mesh>> SeparatedMeshes;
		
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

		// If there's only one component, return the original mesh
		if (Components.size() <= 1) {
			auto OriginalMesh = std::make_shared<mesh>();
			*OriginalMesh = *this;
			SeparatedMeshes.push_back(OriginalMesh);
			return SeparatedMeshes;
		}

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
			NewMesh->CenterOfMass = NewMesh->calculate_center_of_mass();
			NewMesh->BoundingRadius = NewMesh->calculate_bounding_radius();
			NewMesh->Mass = this->Mass * (static_cast<float>(Component.size()) / static_cast<float>(this->Vertex.size()));

			SeparatedMeshes.push_back(NewMesh);
		}

		return SeparatedMeshes;
	}

	// ===== LOD Generation using Quadric Error Metrics (QEM) =====

	// Quadric error matrix for a vertex (4x4 symmetric matrix represented as 10 values)
	struct Quadric {
		double q[10]; // Upper triangular: q11,q12,q13,q14,q22,q23,q24,q33,q34,q44

		Quadric() {
			for (int i = 0; i < 10; i++) q[i] = 0.0;
		}

		// Construct quadric from plane equation ax + by + cz + d = 0
		Quadric(double a, double b, double c, double d) {
			q[0] = a * a;  q[1] = a * b;  q[2] = a * c;  q[3] = a * d;
			q[4] = b * b;  q[5] = b * c;  q[6] = b * d;
			q[7] = c * c;  q[8] = c * d;
			q[9] = d * d;
		}

		// Add two quadrics
		Quadric operator+(const Quadric& other) const {
			Quadric result;
			for (int i = 0; i < 10; i++) {
				result.q[i] = q[i] + other.q[i];
			}
			return result;
		}

		// Evaluate quadric error for point (x, y, z)
		double evaluate(double x, double y, double z) const {
			return q[0] * x * x + 2 * q[1] * x * y + 2 * q[2] * x * z + 2 * q[3] * x
				 + q[4] * y * y + 2 * q[5] * y * z + 2 * q[6] * y
				 + q[7] * z * z + 2 * q[8] * z
				 + q[9];
		}

		// Find optimal vertex position that minimizes quadric error
		bool findOptimalPosition(math::vec<float, 3>& pos) const {
			// Solve linear system: A * v = b where A is upper 3x3 of quadric matrix
			double det = q[0] * (q[4] * q[7] - q[5] * q[5])
					   - q[1] * (q[1] * q[7] - q[2] * q[5])
					   + q[2] * (q[1] * q[5] - q[2] * q[4]);

			if (std::abs(det) < 1e-10) return false; // Singular matrix

			double invDet = 1.0 / det;
			pos[0] = static_cast<float>(-invDet * (q[3] * (q[4] * q[7] - q[5] * q[5]) - q[1] * (q[6] * q[7] - q[5] * q[8]) + q[2] * (q[6] * q[5] - q[4] * q[8])));
			pos[1] = static_cast<float>(-invDet * (q[0] * (q[6] * q[7] - q[5] * q[8]) - q[3] * (q[1] * q[7] - q[2] * q[5]) + q[2] * (q[1] * q[8] - q[2] * q[6])));
			pos[2] = static_cast<float>(-invDet * (q[0] * (q[4] * q[8] - q[5] * q[6]) - q[1] * (q[1] * q[8] - q[2] * q[6]) + q[3] * (q[1] * q[5] - q[2] * q[4])));

			return true;
		}
	};

	// Edge structure for priority queue
	struct Edge {
		uint v1, v2;                        // Vertex indices
		double error;                       // Collapse error
		math::vec<float, 3> targetPos;      // Target position after collapse
		
		bool operator>(const Edge& other) const {
			return error > other.error;     // Min heap
		}
	};

	std::shared_ptr<phys::mesh> mesh::generate_lod(const lod_parameters& aParams) const {
		// Only support triangle meshes for now
		if (this->Topology.Primitive != primitive::TRIANGLE) {
			return nullptr;
		}

		// Ensure we have triangles
		bool Use32Bit = !this->Topology.Data32.empty();
		size_t IndexCount = Use32Bit ? this->Topology.Data32.size() : this->Topology.Data16.size();
		if (IndexCount < 3) return nullptr;

		// Create result mesh
		auto ResultMesh = std::make_shared<mesh>();
		*ResultMesh = *this;

		// Determine target vertex count
		size_t TargetVertexCount = this->Vertex.size();
		if (aParams.TargetVertexCount > 0) {
			TargetVertexCount = static_cast<size_t>(aParams.TargetVertexCount);
		} else if (aParams.TargetTriangleCount > 0) {
			TargetVertexCount = static_cast<size_t>(aParams.TargetTriangleCount * 3 / 2); // Rough estimate
		} else {
			TargetVertexCount = static_cast<size_t>(this->Vertex.size() * aParams.ReductionFactor);
		}
		TargetVertexCount = std::max<size_t>(3, TargetVertexCount); // At least a triangle

		// Build adjacency information
		std::unordered_map<uint, std::unordered_set<uint>> VertexToVertices; // Edges
		std::unordered_map<uint, std::unordered_set<uint>> VertexToTriangles; // Which triangles use this vertex
		std::vector<std::array<uint, 3>> Triangles; // List of triangles

		auto GetIndex = [&](size_t i) -> uint {
			return Use32Bit ? this->Topology.Data32[i] : this->Topology.Data16[i];
		};

		for (size_t i = 0; i + 2 < IndexCount; i += 3) {
			uint v0 = GetIndex(i);
			uint v1 = GetIndex(i + 1);
			uint v2 = GetIndex(i + 2);

			uint triIdx = static_cast<uint>(Triangles.size());
			Triangles.push_back({v0, v1, v2});

			VertexToTriangles[v0].insert(triIdx);
			VertexToTriangles[v1].insert(triIdx);
			VertexToTriangles[v2].insert(triIdx);

			VertexToVertices[v0].insert(v1); VertexToVertices[v0].insert(v2);
			VertexToVertices[v1].insert(v0); VertexToVertices[v1].insert(v2);
			VertexToVertices[v2].insert(v0); VertexToVertices[v2].insert(v1);
		}

		// Calculate quadric for each vertex
		std::vector<Quadric> VertexQuadrics(this->Vertex.size());
		std::vector<bool> VertexValid(this->Vertex.size(), true);
		std::vector<bool> TriangleValid(Triangles.size(), true);

		for (size_t i = 0; i < Triangles.size(); i++) {
			const auto& tri = Triangles[i];
			const auto& p0 = this->Vertex[tri[0]].Position;
			const auto& p1 = this->Vertex[tri[1]].Position;
			const auto& p2 = this->Vertex[tri[2]].Position;

			// Calculate plane equation
			math::vec<float, 3> v1 = p1 - p0;
			math::vec<float, 3> v2 = p2 - p0;
			math::vec<float, 3> normal = math::normalize(v1 ^ v2);
			
			double a = normal[0];
			double b = normal[1];
			double c = normal[2];
			double d = -(a * p0[0] + b * p0[1] + c * p0[2]);

			Quadric planeQuadric(a, b, c, d);
			VertexQuadrics[tri[0]] = VertexQuadrics[tri[0]] + planeQuadric;
			VertexQuadrics[tri[1]] = VertexQuadrics[tri[1]] + planeQuadric;
			VertexQuadrics[tri[2]] = VertexQuadrics[tri[2]] + planeQuadric;
		}

		// Identify boundary edges (appear in only one triangle)
		std::unordered_set<uint64_t> BoundaryEdges;
		std::unordered_map<uint64_t, int> EdgeCount;
		
		auto MakeEdgeKey = [](uint a, uint b) -> uint64_t {
			if (a > b) std::swap(a, b);
			return (static_cast<uint64_t>(a) << 32) | b;
		};

		for (const auto& tri : Triangles) {
			EdgeCount[MakeEdgeKey(tri[0], tri[1])]++;
			EdgeCount[MakeEdgeKey(tri[1], tri[2])]++;
			EdgeCount[MakeEdgeKey(tri[2], tri[0])]++;
		}

		for (const auto& [edge, count] : EdgeCount) {
			if (count == 1) BoundaryEdges.insert(edge);
		}

		// Calculate error and target position for all edges
		auto CalculateEdgeError = [&](uint v1, uint v2, Edge& edge) -> bool {
			if (!VertexValid[v1] || !VertexValid[v2]) return false;

			edge.v1 = v1;
			edge.v2 = v2;

			// Check if boundary edge and should be preserved
			if (aParams.PreserveBoundaries && BoundaryEdges.count(MakeEdgeKey(v1, v2))) {
				edge.error = std::numeric_limits<double>::max();
				return false;
			}

			Quadric combined = VertexQuadrics[v1] + VertexQuadrics[v2];

			// Determine target position
			math::vec<float, 3> pos;
			bool optimalFound = false;

			if (aParams.Placement == lod_parameters::placement_policy::OPTIMAL) {
				optimalFound = combined.findOptimalPosition(pos);
			}

			if (!optimalFound) {
				// Fallback strategies
				const auto& p1 = this->Vertex[v1].Position;
				const auto& p2 = this->Vertex[v2].Position;

				switch (aParams.Placement) {
					case lod_parameters::placement_policy::MIDPOINT:
						pos = (p1 + p2) * 0.5f;
						break;
					case lod_parameters::placement_policy::ENDPOINT_V1:
						pos = p1;
						break;
					case lod_parameters::placement_policy::ENDPOINT_V2:
						pos = p2;
						break;
					case lod_parameters::placement_policy::ENDPOINT_MIN: {
						double err1 = combined.evaluate(p1[0], p1[1], p1[2]);
						double err2 = combined.evaluate(p2[0], p2[1], p2[2]);
						pos = (err1 < err2) ? p1 : p2;
						break;
					}
					default:
						pos = (p1 + p2) * 0.5f;
				}
			}

			edge.targetPos = pos;
			edge.error = combined.evaluate(pos[0], pos[1], pos[2]);

			// Add edge length weight if hybrid metric
			if (aParams.Metric == lod_parameters::metric_type::HYBRID ||
				aParams.Metric == lod_parameters::metric_type::EDGE_LENGTH) {
				float edgeLength = math::length(this->Vertex[v2].Position - this->Vertex[v1].Position);
				edge.error += edgeLength * 0.1; // Weight factor
			}

			return edge.error <= aParams.MaxError;
		};

		// Build initial edge heap
		std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> EdgeHeap;
		
		for (const auto& [v, neighbors] : VertexToVertices) {
			for (uint neighbor : neighbors) {
				if (v < neighbor) { // Add each edge once
					Edge edge;
					if (CalculateEdgeError(v, neighbor, edge)) {
						EdgeHeap.push(edge);
					}
				}
			}
		}

		// Collapse edges until target reached
		size_t CurrentVertexCount = this->Vertex.size();
		int CollapseCount = 0;

		while (!EdgeHeap.empty() && CurrentVertexCount > TargetVertexCount) {
			if (aParams.MaxIterations > 0 && CollapseCount >= aParams.MaxIterations) break;

			Edge edge = EdgeHeap.top();
			EdgeHeap.pop();

			// Check if edge is still valid
			if (!VertexValid[edge.v1] || !VertexValid[edge.v2]) continue;
			if (!VertexToVertices[edge.v1].count(edge.v2)) continue;

			// Perform collapse: v2 -> v1, remove v2
			uint keepVertex = edge.v1;
			uint removeVertex = edge.v2;

			// Update vertex position
			ResultMesh->Vertex[keepVertex].Position = edge.targetPos;

			// Merge quadrics
			VertexQuadrics[keepVertex] = VertexQuadrics[keepVertex] + VertexQuadrics[removeVertex];

			// Update triangles and adjacency
			std::unordered_set<uint> AffectedVertices;
			std::vector<uint> TrianglesToRemove;

			for (uint triIdx : VertexToTriangles[removeVertex]) {
				if (!TriangleValid[triIdx]) continue;

				auto& tri = Triangles[triIdx];
				bool hasKeep = false;
				bool hasRemove = false;

				for (int i = 0; i < 3; i++) {
					if (tri[i] == keepVertex) hasKeep = true;
					if (tri[i] == removeVertex) hasRemove = true;
				}

				// If triangle has both vertices, it becomes degenerate
				if (hasKeep && hasRemove) {
					TriangleValid[triIdx] = false;
					TrianglesToRemove.push_back(triIdx);
				} else if (hasRemove) {
					// Replace removeVertex with keepVertex
					for (int i = 0; i < 3; i++) {
						if (tri[i] == removeVertex) {
							tri[i] = keepVertex;
						}
						AffectedVertices.insert(tri[i]);
					}
					VertexToTriangles[keepVertex].insert(triIdx);
				}
			}

			// Update vertex adjacency
			for (uint neighbor : VertexToVertices[removeVertex]) {
				if (neighbor == keepVertex) continue;
				VertexToVertices[neighbor].erase(removeVertex);
				VertexToVertices[neighbor].insert(keepVertex);
				VertexToVertices[keepVertex].insert(neighbor);
				AffectedVertices.insert(neighbor);
			}

			VertexToVertices.erase(removeVertex);
			VertexToVertices[keepVertex].erase(removeVertex);
			VertexValid[removeVertex] = false;
			CurrentVertexCount--;
			CollapseCount++;

			// Recompute errors for affected edges
			for (uint v : AffectedVertices) {
				if (!VertexValid[v]) continue;
				for (uint neighbor : VertexToVertices[v]) {
					if (v < neighbor) {
						Edge newEdge;
						if (CalculateEdgeError(v, neighbor, newEdge)) {
							EdgeHeap.push(newEdge);
						}
					}
				}
			}
		}

		// Build output mesh
		std::unordered_map<uint, uint> OldToNew;
		std::vector<mesh::vertex> NewVertices;

		for (size_t i = 0; i < VertexValid.size(); i++) {
			if (VertexValid[i]) {
				OldToNew[i] = static_cast<uint>(NewVertices.size());
				NewVertices.push_back(ResultMesh->Vertex[i]);
			}
		}

		ResultMesh->Vertex = NewVertices;

		// Build new topology
		ResultMesh->Topology.Data16.clear();
		ResultMesh->Topology.Data32.clear();

		bool UseNew32Bit = NewVertices.size() > 65535;

		for (size_t i = 0; i < Triangles.size(); i++) {
			if (!TriangleValid[i]) continue;

			const auto& tri = Triangles[i];
			if (!VertexValid[tri[0]] || !VertexValid[tri[1]] || !VertexValid[tri[2]]) continue;

			// Check for degenerate triangles
			if (tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0]) continue;

			uint i0 = OldToNew[tri[0]];
			uint i1 = OldToNew[tri[1]];
			uint i2 = OldToNew[tri[2]];

			if (UseNew32Bit) {
				ResultMesh->Topology.Data32.push_back(i0);
				ResultMesh->Topology.Data32.push_back(i1);
				ResultMesh->Topology.Data32.push_back(i2);
			} else {
				ResultMesh->Topology.Data16.push_back(static_cast<ushort>(i0));
				ResultMesh->Topology.Data16.push_back(static_cast<ushort>(i1));
				ResultMesh->Topology.Data16.push_back(static_cast<ushort>(i2));
			}
		}

		// Recalculate normals if requested
		if (aParams.RecalculateNormals) {
			// Reset normals
			for (auto& v : ResultMesh->Vertex) {
				v.Normal = math::vec<float, 3>(0.0f, 0.0f, 0.0f);
			}

			// Accumulate face normals
			size_t newIndexCount = UseNew32Bit ? ResultMesh->Topology.Data32.size() : ResultMesh->Topology.Data16.size();
			for (size_t i = 0; i + 2 < newIndexCount; i += 3) {
				uint i0 = UseNew32Bit ? ResultMesh->Topology.Data32[i] : ResultMesh->Topology.Data16[i];
				uint i1 = UseNew32Bit ? ResultMesh->Topology.Data32[i + 1] : ResultMesh->Topology.Data16[i + 1];
				uint i2 = UseNew32Bit ? ResultMesh->Topology.Data32[i + 2] : ResultMesh->Topology.Data16[i + 2];

				const auto& p0 = ResultMesh->Vertex[i0].Position;
				const auto& p1 = ResultMesh->Vertex[i1].Position;
				const auto& p2 = ResultMesh->Vertex[i2].Position;

				math::vec<float, 3> normal = math::normalize((p1 - p0) ^ (p2 - p0));
				ResultMesh->Vertex[i0].Normal += normal;
				ResultMesh->Vertex[i1].Normal += normal;
				ResultMesh->Vertex[i2].Normal += normal;
			}

			// Normalize
			for (auto& v : ResultMesh->Vertex) {
				if (math::length(v.Normal) > 0.001f) {
					v.Normal = math::normalize(v.Normal);
				}
			}
		}

		// Update mesh properties
		ResultMesh->Name = this->Name + "_lod";
		ResultMesh->CenterOfMass = ResultMesh->calculate_center_of_mass();
		ResultMesh->BoundingRadius = ResultMesh->calculate_bounding_radius();

		return ResultMesh;
	}

	mesh::lod_chain mesh::generate_lod_chain(int aNumLevels, float aReductionPerLevel, const lod_parameters& aBaseParams) const {
		lod_chain chain;

		if (aNumLevels <= 0) return chain;

		float currentReduction = 1.0f;
		float currentMaxError = aBaseParams.MaxError;

		for (int level = 0; level < aNumLevels; level++) {
			lod_parameters params = aBaseParams;
			params.ReductionFactor = currentReduction;
			params.MaxError = currentMaxError;

			auto lodMesh = this->generate_lod(params);
			if (!lodMesh || lodMesh->Vertex.empty()) break;

			chain.Levels.push_back(lodMesh);
			chain.ReductionFactors.push_back(currentReduction);
			chain.MaxErrors.push_back(static_cast<float>(currentMaxError));

			// Calculate approximate transition distance based on screen space error
			float transitionDistance = 100.0f * std::pow(2.0f, static_cast<float>(level));
			chain.TransitionDistances.push_back(transitionDistance);

			// Update for next level
			currentReduction *= aReductionPerLevel;
			currentMaxError *= aBaseParams.ErrorBudgetMultiplier;
		}

		return chain;
	}

	// ===== Convex Hull Generation using QuickHull Algorithm =====

	std::shared_ptr<phys::mesh> mesh::generate_convex_hull() const {
		if (this->Vertex.empty()) {
			return nullptr;
		}

		// Helper structures for QuickHull
		struct HullFace {
			uint v0, v1, v2;  // Vertex indices
			math::vec<float, 3> normal;
			math::vec<float, 3> centroid;
			std::vector<uint> outsideSet;  // Points outside this face
			
			HullFace(uint a, uint b, uint c) : v0(a), v1(b), v2(c) {}
		};

		auto CalculateFaceData = [&](HullFace& face, const std::vector<math::vec<float, 3>>& points) {
			const auto& p0 = points[face.v0];
			const auto& p1 = points[face.v1];
			const auto& p2 = points[face.v2];
			
			face.centroid = (p0 + p1 + p2) / 3.0f;
			face.normal = math::normalize((p1 - p0) ^ (p2 - p0));
		};

		auto PointAboveFace = [&](const math::vec<float, 3>& point, const HullFace& face, 
								   const std::vector<math::vec<float, 3>>& points) -> float {
			return ((point - points[face.v0]) * face.normal);
		};

		// Extract vertex positions
		std::vector<math::vec<float, 3>> points;
		points.reserve(this->Vertex.size());
		for (const auto& v : this->Vertex) {
			points.push_back(v.Position);
		}

		// Step 1: Find initial tetrahedron
		// Find extreme points along each axis
		uint minX = 0, maxX = 0, minY = 0, maxY = 0, minZ = 0, maxZ = 0;
		for (size_t i = 1; i < points.size(); i++) {
			if (points[i][0] < points[minX][0]) minX = i;
			if (points[i][0] > points[maxX][0]) maxX = i;
			if (points[i][1] < points[minY][1]) minY = i;
			if (points[i][1] > points[maxY][1]) maxY = i;
			if (points[i][2] < points[minZ][2]) minZ = i;
			if (points[i][2] > points[maxZ][2]) maxZ = i;
		}

		// Find the pair with maximum distance
		float maxDist = 0.0f;
		uint p0 = minX, p1 = maxX;
		
		auto CheckPair = [&](uint a, uint b) {
			float dist = math::length(points[b] - points[a]);
			if (dist > maxDist) {
				maxDist = dist;
				p0 = a;
				p1 = b;
			}
		};
		
		CheckPair(minX, maxX);
		CheckPair(minY, maxY);
		CheckPair(minZ, maxZ);

		// Find third point: furthest from line p0-p1
		uint p2 = 0;
		float maxDist2 = 0.0f;
		math::vec<float, 3> line = math::normalize(points[p1] - points[p0]);
		
		for (size_t i = 0; i < points.size(); i++) {
			if (i == p0 || i == p1) continue;
			math::vec<float, 3> v = points[i] - points[p0];
			float dist = math::length(v - line * (v * line));
			if (dist > maxDist2) {
				maxDist2 = dist;
				p2 = i;
			}
		}

		// Find fourth point: furthest from plane p0-p1-p2
		math::vec<float, 3> planeNormal = math::normalize((points[p1] - points[p0]) ^ (points[p2] - points[p0]));
		uint p3 = 0;
		float maxDist3 = 0.0f;
		
		for (size_t i = 0; i < points.size(); i++) {
			if (i == p0 || i == p1 || i == p2) continue;
			float dist = std::abs((points[i] - points[p0]) * planeNormal);
			if (dist > maxDist3) {
				maxDist3 = dist;
				p3 = i;
			}
		}

		// Create initial tetrahedron
		std::list<HullFace> hull;
		hull.push_back(HullFace(p0, p1, p2));
		hull.push_back(HullFace(p0, p2, p3));
		hull.push_back(HullFace(p0, p3, p1));
		hull.push_back(HullFace(p1, p3, p2));

		// Orient faces outward and calculate normals
		for (auto& face : hull) {
			CalculateFaceData(face, points);
			
			// Check if normal points inward by testing against centroid
			math::vec<float, 3> center = (points[p0] + points[p1] + points[p2] + points[p3]) * 0.25f;
			if ((face.centroid - center) * face.normal < 0.0f) {
				// Flip face
				std::swap(face.v1, face.v2);
				face.normal = -face.normal;
			}
		}

		// Step 2: Assign points to faces
		std::vector<bool> assigned(points.size(), false);
		assigned[p0] = assigned[p1] = assigned[p2] = assigned[p3] = true;

		for (auto& face : hull) {
			for (size_t i = 0; i < points.size(); i++) {
				if (assigned[i]) continue;
				float dist = PointAboveFace(points[i], face, points);
				if (dist > 1e-6f) {  // Point is outside this face
					face.outsideSet.push_back(i);
					assigned[i] = true;
				}
			}
		}

		// Step 3: QuickHull iteration
		const float epsilon = 1e-6f;
		
		while (true) {
			// Find face with outside points
			auto faceIt = hull.end();
			for (auto it = hull.begin(); it != hull.end(); ++it) {
				if (!it->outsideSet.empty()) {
					faceIt = it;
					break;
				}
			}
			
			if (faceIt == hull.end()) break;  // No more outside points

			// Find furthest point from this face
			uint furthestPoint = faceIt->outsideSet[0];
			float maxDistance = PointAboveFace(points[furthestPoint], *faceIt, points);
			
			for (uint idx : faceIt->outsideSet) {
				float dist = PointAboveFace(points[idx], *faceIt, points);
				if (dist > maxDistance) {
					maxDistance = dist;
					furthestPoint = idx;
				}
			}

			// Find all faces visible from this point
			std::list<HullFace> visibleFaces;
			std::list<HullFace> remainingFaces;
			
			for (auto& face : hull) {
				if (PointAboveFace(points[furthestPoint], face, points) > epsilon) {
					visibleFaces.push_back(face);
				} else {
					remainingFaces.push_back(face);
				}
			}

			// Find horizon edges (edges between visible and non-visible faces)
			struct Edge {
				uint v0, v1;
				bool operator==(const Edge& other) const {
					return (v0 == other.v0 && v1 == other.v1) || (v0 == other.v1 && v1 == other.v0);
				}
			};
			
			std::vector<Edge> horizonEdges;
			
			for (const auto& visFace : visibleFaces) {
				Edge edges[3] = {
					{visFace.v0, visFace.v1},
					{visFace.v1, visFace.v2},
					{visFace.v2, visFace.v0}
				};
				
				for (const auto& edge : edges) {
					bool isHorizon = true;
					// Check if this edge is shared with another visible face
					for (const auto& otherFace : visibleFaces) {
						if (&visFace == &otherFace) continue;
						
						bool hasV0 = (otherFace.v0 == edge.v0 || otherFace.v1 == edge.v0 || otherFace.v2 == edge.v0);
						bool hasV1 = (otherFace.v0 == edge.v1 || otherFace.v1 == edge.v1 || otherFace.v2 == edge.v1);
						
						if (hasV0 && hasV1) {
							isHorizon = false;
							break;
						}
					}
					
					if (isHorizon) {
						horizonEdges.push_back(edge);
					}
				}
			}

			// Create new faces from horizon edges to furthest point
			std::vector<HullFace> newFaces;
			std::vector<uint> orphanedPoints;
			
			// Collect orphaned points from visible faces
			for (const auto& face : visibleFaces) {
				for (uint idx : face.outsideSet) {
					if (idx != furthestPoint) {
						orphanedPoints.push_back(idx);
					}
				}
			}

			for (const auto& edge : horizonEdges) {
				HullFace newFace(edge.v0, edge.v1, furthestPoint);
				CalculateFaceData(newFace, points);
				
				// Ensure outward orientation
				math::vec<float, 3> toPoint = points[furthestPoint] - newFace.centroid;
				if ((toPoint * newFace.normal) < 0.0f) {
					std::swap(newFace.v1, newFace.v2);
					newFace.normal = -newFace.normal;
				}
				
				// Assign orphaned points to new faces
				for (uint idx : orphanedPoints) {
					if (PointAboveFace(points[idx], newFace, points) > epsilon) {
						newFace.outsideSet.push_back(idx);
					}
				}
				
				newFaces.push_back(newFace);
			}

			// Update hull
			hull = remainingFaces;
			for (auto& newFace : newFaces) {
				hull.push_back(newFace);
			}
		}

		// Step 4: Build output mesh
		auto resultMesh = std::make_shared<mesh>();
		resultMesh->Name = this->Name + "_convexhull";
		resultMesh->Topology.Primitive = primitive::TRIANGLE;

		// Map original vertex indices to new indices
		std::unordered_map<uint, uint> vertexMap;
		
		for (const auto& face : hull) {
			if (vertexMap.find(face.v0) == vertexMap.end()) {
				vertexMap[face.v0] = static_cast<uint>(resultMesh->Vertex.size());
				resultMesh->Vertex.push_back(this->Vertex[face.v0]);
			}
			if (vertexMap.find(face.v1) == vertexMap.end()) {
				vertexMap[face.v1] = static_cast<uint>(resultMesh->Vertex.size());
				resultMesh->Vertex.push_back(this->Vertex[face.v1]);
			}
			if (vertexMap.find(face.v2) == vertexMap.end()) {
				vertexMap[face.v2] = static_cast<uint>(resultMesh->Vertex.size());
				resultMesh->Vertex.push_back(this->Vertex[face.v2]);
			}
		}

		// Build topology
		bool use32Bit = resultMesh->Vertex.size() > 65535;
		
		for (const auto& face : hull) {
			uint i0 = vertexMap[face.v0];
			uint i1 = vertexMap[face.v1];
			uint i2 = vertexMap[face.v2];
			
			if (use32Bit) {
				resultMesh->Topology.Data32.push_back(i0);
				resultMesh->Topology.Data32.push_back(i1);
				resultMesh->Topology.Data32.push_back(i2);
			} else {
				resultMesh->Topology.Data16.push_back(static_cast<ushort>(i0));
				resultMesh->Topology.Data16.push_back(static_cast<ushort>(i1));
				resultMesh->Topology.Data16.push_back(static_cast<ushort>(i2));
			}
		}

		// Recalculate normals
		for (auto& v : resultMesh->Vertex) {
			v.Normal = math::vec<float, 3>(0.0f, 0.0f, 0.0f);
		}

		size_t indexCount = use32Bit ? resultMesh->Topology.Data32.size() : resultMesh->Topology.Data16.size();
		for (size_t i = 0; i + 2 < indexCount; i += 3) {
			uint i0 = use32Bit ? resultMesh->Topology.Data32[i] : resultMesh->Topology.Data16[i];
			uint i1 = use32Bit ? resultMesh->Topology.Data32[i + 1] : resultMesh->Topology.Data16[i + 1];
			uint i2 = use32Bit ? resultMesh->Topology.Data32[i + 2] : resultMesh->Topology.Data16[i + 2];

			const auto& p0 = resultMesh->Vertex[i0].Position;
			const auto& p1 = resultMesh->Vertex[i1].Position;
			const auto& p2 = resultMesh->Vertex[i2].Position;

			math::vec<float, 3> normal = math::normalize((p1 - p0) ^ (p2 - p0));
			resultMesh->Vertex[i0].Normal += normal;
			resultMesh->Vertex[i1].Normal += normal;
			resultMesh->Vertex[i2].Normal += normal;
		}

		for (auto& v : resultMesh->Vertex) {
			if (math::length(v.Normal) > 0.001f) {
				v.Normal = math::normalize(v.Normal);
			}
		}

		// Calculate mesh properties
		resultMesh->Mass = this->Mass;
		resultMesh->CenterOfMass = resultMesh->calculate_center_of_mass();
		resultMesh->BoundingRadius = resultMesh->calculate_bounding_radius();

		return resultMesh;
	}

}
