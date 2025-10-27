#pragma once
#ifndef GEODESY_PHYS_NODE_H
#define GEODESY_PHYS_NODE_H

#include <memory>

// Include include config.
// #include "../../config.h"
// Include include math.
#include <geodesy/math.h>
// Include physics mesh. (Needed for collisions)
#include "mesh.h"
// Include animation.
#include "animation.h"
// Include force.
#include "force.h"

struct aiScene;
struct aiNode;

namespace geodesy::phys {

	/*
	The goal of phys::node is that it is the base node object for a node hierarchy with transformations.
	It's intended use is to first be used as a container class for unpacked 3d models from assimp. Second,
	a base primitive object in which rigid body physics can be applied to. A node can be a base physics node,
	it can be a graphics node carrying mesh instances, it can be a bone which animates vertex data in a vertex
	shader, and finally a root node can be an object which the engine processes and applies physics to.
	*/

	class node {
	public:

		// Node Types to identify runtime node.
		enum type : int {
			PHYSICS,
			GRAPHICS,
			OBJECT,
		};

		enum motion {
			STATIC,			// Node doesn't move in world space
			DYNAMIC,		// Node moves, but based on physical forces applied.
			ANIMATED,		// Node moves based on predetermined animation path data.
		};
		
		// Node traversal/hierarchy data.
		node*                   				Root;       		// Root node in hierarchy
		node*                   				Parent;     		// Parent node in hierarchy
		std::vector<node*> 						Child;      		// Child nodes in hierarchy
		
		// Node Data
		std::string             				Identifier; 		// Node identifier
		int 									Type;       		// Node type
		float									Mass;				// Kilogram			[kg]
		math::mat<float, 3, 3>					InertiaTensor;		// Inertia Tensor	[kg*m^2]
		math::vec<float, 3>						Position;			// Meter			[m]
		math::quaternion<float>					Orientation;		// Quaternion		[Dimensionless]
		math::vec<float, 3> 					Scale;				// Scaling Factor	[Dimensionless]
		math::mat<float, 4, 4> 					DefaultTransform; 	// Node transformation matrix
		math::mat<float, 4, 4> 					CurrentTransform;   // Final Node Transform each frame after physics and animation
		math::mat<float, 4, 4> 					GlobalTransform;    // Node Transform to World Space.
		math::vec<float, 3>						LinearMomentum;		// Linear Momentum	[kg*m/s]
		math::vec<float, 3>						AngularMomentum;	// Angular Momentum [kg*m/s]
		std::shared_ptr<phys::mesh>				CollisionMesh;		// Mesh Data
		
		node();
		~node();

		// Returns the count of nodes in the hierarchy starting from this node.
		size_t node_count() const;

		// For this node, it will calculate the model transform for a node at a particular time.
		math::mat<float, 4, 4> transform() const;

		// Returns the node with the given name in the hierarchy. Will return
		// nullptr if the node is not found in the hierarchy.
		node* find(std::string aName);

		// Creates a linearized list of nodes in the hierarchy.
		std::vector<node*> linearize();

		void set_root(node* aRootNode);

		// Overridable node data copy function.
		virtual void copy_data(const node* aNode);
		virtual void copy(const node* aNode);
		virtual void swap(node* aNode);
		virtual void host_update(
			double 									aDeltaTime = 0.0f, 
			double 									aTime = 0.0f, 
			const std::vector<phys::force>& 		aAppliedForces = {},
			const std::vector<phys::animation>& 	aPlaybackAnimation = {},
			const std::vector<float>& 				aAnimationWeight = {}
		);
		virtual void device_update(
			double 									aDeltaTime = 0.0f, 
			double 									aTime = 0.0f, 
			const std::vector<phys::force>& 		aAppliedForces = {}
		);
		
	};
	
}

#endif // !GEODESY_PHYS_NODE_H