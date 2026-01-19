#pragma once
#ifndef GEODESY_PHYS_NODE_H
#define GEODESY_PHYS_NODE_H

#include <memory>
#include <variant>

// Include include config.
// #include "../../config.h"
// Include include math.
#include <geodesy/math.h>
// Include physics mesh. (Needed for collisions)
#include "mesh.h"
// Include animation.
#include "animation.h"

struct aiScene;
struct aiNode;

namespace geodesy::phys {

	// ===== Jolt Physics Collision Filtering System ===== //
	
	/// BroadPhaseLayerInterface implementation
	/// Maps ObjectLayer (motion type) to BroadPhaseLayer for spatial partitioning
	/// Direct 1:1 mapping: EMotionType values (Static=0, Kinematic=1, Dynamic=2) map to BroadPhaseLayer
	class BroadPhaseLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
	public:
		BroadPhaseLayerInterfaceImpl() = default;
		
		virtual uint GetNumBroadPhaseLayers() const override;
		virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override;
		
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
		virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override;
#endif
	};
	
	/// ObjectVsBroadPhaseLayerFilter implementation
	/// Determines if an ObjectLayer can collide with a BroadPhaseLayer during broad phase
	class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
	public:
		virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override;
	};
	
	/// ObjectLayerPairFilter implementation
	/// Fine-grained collision rules based on motion types
	/// Collision rules:
	/// - Static vs Static: No collision (optimization)
	/// - Static vs Kinematic: Collide (kinematic bounces off walls)
	/// - Static vs Dynamic: Collide (dynamic bounces off walls)
	/// - Kinematic vs Kinematic: Collide (they stop each other)
	/// - Kinematic vs Dynamic: Collide (kinematic pushes dynamic)
	/// - Dynamic vs Dynamic: Collide (rigid body dynamics)
	class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
	public:
		virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override;
	};

	// ===== Physics Node Class ===== //

	/*
	The goal of phys::node is that it is the base node object for a node hierarchy with transformations.
	It's intended use is to first be used as a container class for unpacked 3d models from assimp. Second,
	a base primitive object in which rigid body physics can be applied to. A node can be a base physics node,
	it can be a graphics node carrying mesh instances, it can be a bone which animates vertex data in a vertex
	shader, and finally a root node can be an object which the engine processes and applies physics to.
	*/

	class node {
	public:
		
		// Node traversal/hierarchy data
		node* Root;                                             // Root node in hierarchy
		node* Parent;                                           // Parent node in hierarchy
		std::vector<node*> Child;                               // Child nodes in hierarchy
		std::string Identifier;                                 // Node identifier
		JPH::EMotionType Motion;                                // Determines how this node moves in world space (Static/Kinematic/Dynamic)
		bool Collision;                                         // Is collision detection enabled for this node
		math::vec<float, 3> Position;                           // Position [m]
		math::quaternion<float> Orientation;                    // Orientation quaternion [Dimensionless]
		math::vec<float, 3> Scale;                              // Scaling factor [Dimensionless]
		float Mass;                                             // Mass [kg]
		math::mat<float, 3, 3> InertiaTensor;                   // Inertia tensor [kg*m^2]
		math::vec<float, 3> LinearMomentum;                     // Linear momentum [kg*m/s]
		math::vec<float, 3> AngularMomentum;                    // Angular momentum [kg*m/s]
		std::shared_ptr<phys::mesh> PhysicsMesh;                // Collision mesh data (one mesh per node = one Jolt body)
		JPH::BodyID JoltBodyID;                                 // Jolt physics body ID (links node to physics simulation)
		std::vector<math::vec<float, 3>> PendingForces;         // Forces to apply [N]
		std::vector<math::vec<float, 3>> PendingImpulses;       // Impulses to apply [N*s]
		std::vector<math::vec<float, 3>> PendingTorques;        // Torques to apply [N*m]
		math::mat<float, 4, 4> TransformToParentDefault;        // Initially loaded transform data
		math::mat<float, 4, 4> TransformToParentCurrent;        // Cached transform data based on current state
		math::mat<float, 4, 4> TransformToWorld;                // Node transform to world space
		math::mat<float, 4, 4> InverseTransformToWorld;         // Cached inverse transform (world space to node local space)
		math::vec<float, 3> WorldScaleCache;                    // Cached world scale from physics simulation (scratch space for Jolt sync)
		
		node();
		node(JPH::PhysicsSystem* aPhysicsSystem, std::shared_ptr<phys::mesh> aPhysicsMesh = nullptr);
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

		/// Recursively recalculate transforms for this node and all children
		/// Syncs Jolt physics state back to node transforms (world and local)
		void recalculate_parent_transforms_and_local_data();

		// Overridable node data copy function.
		virtual void copy_data(const node* aNode);
		virtual void copy(const node* aNode);
		virtual void swap(node* aNode);
		virtual void host_update(
			double 									aDeltaTime = 0.0f, 
			double 									aTime = 0.0f, 
			const std::vector<phys::animation>& 	aPlaybackAnimation = {},
			const std::vector<float>& 				aAnimationWeight = {}
		);
		virtual void device_update(
			double 									aDeltaTime = 0.0f, 
			double 									aTime = 0.0f
		);
		
	};
	
}

#endif // !GEODESY_PHYS_NODE_H
