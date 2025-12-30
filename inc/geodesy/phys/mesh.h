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
			// enum input_rate {
			// 	VERTEX			= VK_VERTEX_INPUT_RATE_VERTEX,
			// 	INSTANCE		= VK_VERTEX_INPUT_RATE_INSTANCE
			// };
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

		// ! Untested
		// Separates disconnected parts of the mesh into individual meshes. Outputs a vector of pairs containing
		// position vector of the split mesh with respect to the original mesh's origin along with the new mesh.
		std::vector<std::pair<math::vec<float, 3>, std::shared_ptr<phys::mesh>>> split_disconnected_meshes() const;

		// Calculate lower LOD of mesh.
		//std::shared_ptr<phys::mesh> generate_lod(float aReductionFactor) const;

	};

}

#endif // !GEODESY_PHYS_MESH_H