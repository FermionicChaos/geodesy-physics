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

		enum motion : int {
			STATIC,					// Node doesn't move in world space
			KINEMATIC,				// Node either moves based on animation, scripted movement, or player control.
			DYNAMIC,				// Node moves, but based on physical forces applied.
		};

		// Constraint types that can connect this node to its parent
		enum constraint_type : int {
			NONE,					// No constraint - completely free body (detached from parent hierarchy)
			FIXED,					// Rigidly welded to parent (no relative motion)
			POINT,					// Ball-and-socket joint (3 rotation DOF, 0 translation DOF)
			HINGE,					// Door hinge (1 rotation DOF around axis, 0 translation DOF)
			SLIDER,					// Piston/rail (1 translation DOF along axis, 0 rotation DOF)
			CONE,					// Cone constraint (swing limited, twist free)
			SWING_TWIST,			// Ragdoll joint (swing and twist limits)
			DISTANCE,				// Maintains fixed distance between two points
			SIX_DOF,				// Six degree of freedom constraint (configurable limits on all axes)
		};

		// Describes how this node is constrained to its parent
		struct constraint_descriptor {
			constraint_type Type = constraint_type::NONE;
			
			// Attachment points (relative to each body's center of mass)
			math::vec<float, 3> AttachmentPoint1 = { 0.0f, 0.0f, 0.0f };  // On parent
			math::vec<float, 3> AttachmentPoint2 = { 0.0f, 0.0f, 0.0f };  // On this node
			
			// Axes for directional constraints (Hinge, Slider, Cone)
			math::vec<float, 3> PrimaryAxis1 = { 0.0f, 1.0f, 0.0f };      // Parent's primary axis (hinge axis, slider direction)
			math::vec<float, 3> PrimaryAxis2 = { 0.0f, 1.0f, 0.0f };      // Child's primary axis
			math::vec<float, 3> NormalAxis1 = { 1.0f, 0.0f, 0.0f };       // Parent's normal axis (for angle reference)
			math::vec<float, 3> NormalAxis2 = { 1.0f, 0.0f, 0.0f };       // Child's normal axis
			
			// Limits for constrained motion
			float LimitMin = -static_cast<float>(math::constant::pi);		// Minimum angle (rad) or distance (m)
			float LimitMax = static_cast<float>(math::constant::pi);		// Maximum angle (rad) or distance (m)
			bool  LimitsEnabled = false;									// Whether to enforce limits
			
			// Spring settings for soft limits
			bool  UseSoftLimits = false;				// Use spring-based soft limits instead of hard limits
			float SpringFrequency = 2.0f;				// Spring frequency (Hz) when limits are exceeded
			float SpringDamping = 0.1f;					// Spring damping ratio (0 = no damping, 1 = critical damping)
			
			// Motor/actuation settings
			bool  MotorEnabled = false;					// Enable motor to drive the constraint
			float MotorTargetVelocity = 0.0f;			// Target velocity for velocity motor (rad/s or m/s)
			float MotorTargetPosition = 0.0f;			// Target position for position motor (rad or m)
			float MotorMaxForce = 0.0f;					// Maximum force/torque the motor can apply (N or N·m)
			
			// Friction
			float MaxFrictionForce = 0.0f;				// Maximum friction force/torque when not motorized (N or N·m)
			
			// Six DOF specific settings (which axes are free/limited)
			struct six_dof_settings {
				bool TranslationFree[3] = { false, false, false };	// X, Y, Z translation freedom
				bool RotationFree[3] = { false, false, false };		// X, Y, Z rotation freedom
				float TranslationMin[3] = { 0.0f, 0.0f, 0.0f };		// Min translation limits
				float TranslationMax[3] = { 0.0f, 0.0f, 0.0f };		// Max translation limits
				float RotationMin[3] = { 0.0f, 0.0f, 0.0f };		// Min rotation limits (rad)
				float RotationMax[3] = { 0.0f, 0.0f, 0.0f };		// Max rotation limits (rad)
			};
			six_dof_settings SixDOF;
			
			// Distance constraint specific
			float MinDistance = 0.0f;					// Minimum distance to maintain
			float MaxDistance = 1.0f;					// Maximum distance to maintain
			
			// Cone constraint specific
			float MaxConeAngle = static_cast<float>(math::constant::pi) / 4.0f;	// Maximum cone half-angle (rad)
			float MaxTwistAngle = static_cast<float>(math::constant::pi);			// Maximum twist angle (rad)
			
			constraint_descriptor() = default;
		};
		
		// Node traversal/hierarchy data.
		node*                   						Root;       		// Root node in hierarchy
		node*                   						Parent;     		// Parent node in hierarchy
		std::vector<node*> 								Child;      		// Child nodes in hierarchy

		// Node Data
		std::string             						Identifier; 		// Node identifier
		int 											Type;       		// Node type
		int 											Motion; 			// Determines how this node moves in world space.
		constraint_descriptor 							ParentConstraint;	// Describes how this node is constrained to its parent
		bool 											CollisionEnabled;	// Is collision detection enabled for this node.

		// Physics Data
		math::vec<float, 3>								Position;			// Meter			[m]
		math::quaternion<float>							Orientation;		// Quaternion		[Dimensionless]
		math::vec<float, 3> 							Scale;				// Scaling Factor	[Dimensionless]
		float											Mass;				// Kilogram			[kg]
		math::mat<float, 3, 3>							InertiaTensor;		// Inertia Tensor	[kg*m^2]
		math::vec<float, 3>								LinearMomentum;		// Linear Momentum	[kg*m/s]
		math::vec<float, 3>								AngularMomentum;	// Angular Momentum [kg*m/s]
		std::vector<std::shared_ptr<phys::mesh>>		PhysicsMeshes;		// Mesh Data
		
		// Cached Transform Data
		math::mat<float, 4, 4> 							DefaultTransform; 	// Node transformation matrix
		math::mat<float, 4, 4> 							CurrentTransform;   // Final Node Transform each frame after physics and animation
		math::mat<float, 4, 4> 							GlobalTransform;    // Node Transform to World Space.
		
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