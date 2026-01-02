#pragma once
#ifndef GEODESY_PHYS_MESH_H
#define GEODESY_PHYS_MESH_H

#include <memory>

#include <geodesy/math.h>

// Jolt Physics API
#include <Jolt/Jolt.h>

namespace geodesy::phys {

    class mesh {
	public:

		// Informs how vertex groups are to be interpreted.
		enum primitive {
			POINT,
			LINE,
			LINE_STRIP,
			TRIANGLE,
			TRIANGLE_STRIP,
			TRIANGLE_FAN,
		};

		struct vertex {
			struct weight {
				math::vec<uint, 4>				BoneID;
				math::vec<float, 4>				BoneWeight;
			};
			math::vec<float, 3> 			Position;
			math::vec<float, 3> 			Normal;
			math::vec<float, 3> 			Tangent;
			math::vec<float, 3> 			Bitangent;
			math::vec<float, 3> 			TextureCoordinate;
			math::vec<float, 4> 			Color;
			vertex();
		};

		struct topology {
			primitive						Primitive;
			std::vector<ushort>				Data16;
			std::vector<uint>				Data32;
		};

		struct bone {
			struct weight {
				uint							ID;
				float							Weight;
			};
			std::string						Name;
			std::vector<weight>				Vertex;
			math::mat<float, 4, 4>			Offset;
		};

		// LOD Generation Parameters using Quadric Error Metrics (QEM)
		struct lod_parameters {
			// Primary simplification control
			float ReductionFactor;              // Target vertex count = original * ReductionFactor (0.0 to 1.0)
			
			// Alternative target specification (use one or the other)
			int TargetVertexCount;                // If > 0, overrides ReductionFactor
			int TargetTriangleCount;              // If > 0, overrides ReductionFactor
			
			// Quality control
			float MaxError;  // Maximum allowed quadric error per edge collapse
			float BoundaryWeight;            // Penalty for collapsing boundary edges (higher = preserve more)
			float NormalWeight;                 // Weight for normal deviation in error metric
			float TextureWeight;                // Weight for texture coordinate deviation
			float ColorWeight;                  // Weight for color deviation
			
			// Feature preservation
			bool PreserveTopology;              // Prevent edge collapses that change topology
			bool PreserveBoundaries;            // Prevent boundary edge collapses
			bool PreserveSeams;                 // Prevent UV seam collapses
			float CreaseAngleThreshold;        // Preserve edges with normal angle > threshold (degrees)
			
			// Edge collapse ordering
			enum class metric_type {
				QUADRIC_ERROR,                         // Standard QEM (distance to planes)
				MODIFIED_QEM,                          // QEM with normal/texture weights
				EDGE_LENGTH,                           // Collapse shortest edges first
				HYBRID                                 // Combine quadric error and edge length
			};
			metric_type Metric;
			
			// Vertex placement strategy after collapse
			enum class placement_policy {
				OPTIMAL,                               // Find optimal position minimizing quadric error
				MIDPOINT,                              // Place at edge midpoint
				ENDPOINT_MIN,                          // Place at endpoint with lower error
				ENDPOINT_V1,                           // Always place at first vertex
				ENDPOINT_V2                            // Always place at second vertex
			};
			placement_policy Placement;
			
			// Performance/quality trade-offs
			int MaxIterations;                    // Max edge collapses (-1 = no limit)
			bool RecalculateNormals;            // Recalculate normals after simplification
			bool OptimizePositions;            // Post-process: optimize vertex positions
			
			// Validation
			bool ValidateTopology;              // Check for invalid geometry after each collapse
			bool AllowFlippedTriangles;        // Allow triangle normal flips during collapse
			
			// LOD Chain specific
			float ErrorBudgetMultiplier;        // For LOD chains: multiply MaxError per level
			
			lod_parameters();
		};

		// Generate a chain of LODs with automatic parameter progression
		struct lod_chain {
			std::vector<std::shared_ptr<phys::mesh>> Levels;      // LOD levels from highest to lowest detail
			std::vector<float> ReductionFactors;                   // Reduction factor for each level
			std::vector<float> MaxErrors;                          // Maximum error for each level
			std::vector<float> TransitionDistances;                // Suggested camera distances for transitions
			
			lod_chain();
		};
		
		// Host Memory Objects
		std::string 					Name;
		float 							Mass;
		math::vec<float, 3> 			CenterOfMass;
		math::vec<float, 3> 			BoundingRadius;
		std::vector<vertex> 			Vertex;
		topology 						Topology;

		mesh();

		vertex operator[](size_t aIndex) const;
		vertex& operator[](size_t aIndex);

		// Calculates the center of mass of the mesh based on vertex positions.
		math::vec<float, 3> center_of_mass() const;

		// Calculates the bounding radius of the mesh based on vertex positions.
		math::vec<float, 3> bounding_radius() const;

		// Separates disconnected parts of the mesh into individual meshes based on topology connectivity.
		// Returns a vector of mesh objects, one for each connected component.
		std::vector<std::shared_ptr<phys::mesh>> split_disconnected_meshes() const;

		// Generate a single LOD level with specified parameters
		std::shared_ptr<phys::mesh> generate_lod(const lod_parameters& aParams) const;
		
		lod_chain generate_lod_chain(
			int aNumLevels,                            // Number of LOD levels to generate
			float aReductionPerLevel,                  // Reduction factor per level (multiplicative)
			const lod_parameters& aBaseParams          // Base parameters (modified per level)
		) const;

		// Generate convex hull of mesh using QuickHull algorithm.
		// Returns a new mesh containing only the convex hull triangulation.
		// Useful for creating simplified collision meshes from complex geometry.
		std::shared_ptr<phys::mesh> generate_convex_hull() const;

	};

}

#endif // !GEODESY_PHYS_MESH_H
