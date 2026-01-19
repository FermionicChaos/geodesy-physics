#include <geodesy/phys/node.h>

// Model Loading
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace geodesy::phys {

	// ===== Jolt Filter Implementations ===== //

	uint BroadPhaseLayerInterfaceImpl::GetNumBroadPhaseLayers() const {
		return 3; // Static, Kinematic, Dynamic
	}

	JPH::BroadPhaseLayer BroadPhaseLayerInterfaceImpl::GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const {
		// Direct 1:1 mapping: ObjectLayer (EMotionType value) = BroadPhaseLayer
		// 0=Static, 1=Kinematic, 2=Dynamic
		return JPH::BroadPhaseLayer(inLayer);
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	const char* BroadPhaseLayerInterfaceImpl::GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const {
		switch ((JPH::BroadPhaseLayer::Type)inLayer) {
			case 0: return "STATIC";
			case 1: return "KINEMATIC";
			case 2: return "DYNAMIC";
			default: return "INVALID";
		}
	}
#endif

	bool ObjectVsBroadPhaseLayerFilterImpl::ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const {
		// Direct comparison using EMotionType values: 0=Static, 1=Kinematic, 2=Dynamic
		
		// Static objects only collide with kinematic and dynamic (not other static)
		if (inLayer1 == (JPH::ObjectLayer)JPH::EMotionType::Static) {
			return inLayer2 != JPH::BroadPhaseLayer((uint8_t)JPH::EMotionType::Static);
		}
		
		// Kinematic and dynamic collide with everything
		return true;
	}

	bool ObjectLayerPairFilterImpl::ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const {
		// Direct comparison using EMotionType values: 0=Static, 1=Kinematic, 2=Dynamic
		
		// Rule: Static vs Static - never collide (optimization)
		if (inObject1 == (JPH::ObjectLayer)JPH::EMotionType::Static && 
		    inObject2 == (JPH::ObjectLayer)JPH::EMotionType::Static) {
			return false;
		}
		
		// All other combinations collide:
		// - Static vs Kinematic: Collide
		// - Static vs Dynamic: Collide
		// - Kinematic vs Kinematic: Collide (they stop each other)
		// - Kinematic vs Dynamic: Collide (kinematic pushes dynamic)
		// - Dynamic vs Dynamic: Collide (rigid body dynamics)
		return true;
	}

	// ===== Node Implementation ===== //

	// Default constructor, zero out all data.
	node::node() {
		this->Identifier 				= "";
		this->Root 				= this;
		this->Parent 			= nullptr;
		this->Mass 				= 1.0f; // Default mass to 1 kg.
		this->InertiaTensor 	= {
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 1.0f
		};
		this->Position 			= { 0.0f, 0.0f, 0.0f }; // Default position to origin.
		this->Orientation 		= { 1.0f, 0.0f, 0.0f, 0.0f }; // Default orientation to identity quaternion.
		this->Scale 			= { 1.0f, 1.0f, 1.0f }; // Default scale to 1 in all dimensions.
		this->LinearMomentum 	= { 0.0f, 0.0f, 0.0f }; // Default linear momentum to zero.
		this->AngularMomentum 	= { 0.0f, 0.0f, 0.0f }; // Default angular momentum to zero.
		this->TransformToParentDefault 	= {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};
		this->TransformToParentCurrent = this->TransformToParentDefault;
		this->TransformToWorld = {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};
		this->InverseTransformToWorld = this->TransformToWorld; // Initialize as identity
		
		// Initialize Jolt Physics members
		this->JoltBodyID = JPH::BodyID();
		this->Motion = JPH::EMotionType::Static; // Default motion type
		this->Collision = true; // Default collision enabled
		this->PhysicsMesh = nullptr; // No physics mesh by default
	}

	node::node(JPH::PhysicsSystem* aPhysicsSystem) : node() {
		// Create a Jolt Body from phys::node and phys::mesh.
		if (aPhysicsSystem == nullptr) {
			return; // No physics system provided, skip body creation
		}
		
		// Create collision shape based on PhysicsMesh or use default box shape
		JPH::ShapeSettings* ShapeSettings = nullptr;
		
		if (this->PhysicsMesh != nullptr) {
			// Create shape based on mesh type
			switch (this->PhysicsMesh->Type) {
				case mesh::shape_type::BOX: {
					ShapeSettings = new JPH::BoxShapeSettings(
						JPH::Vec3(
							this->PhysicsMesh->Parameters.Box.HalfExtentX,
							this->PhysicsMesh->Parameters.Box.HalfExtentY,
							this->PhysicsMesh->Parameters.Box.HalfExtentZ
						)
					);
					break;
				}
				case mesh::shape_type::SPHERE: {
					ShapeSettings = new JPH::SphereShapeSettings(this->PhysicsMesh->Parameters.Sphere.Radius);
					break;
				}
				case mesh::shape_type::CAPSULE: {
					ShapeSettings = new JPH::CapsuleShapeSettings(
						this->PhysicsMesh->Parameters.Capsule.HalfHeight,
						this->PhysicsMesh->Parameters.Capsule.Radius
					);
					break;
				}
				case mesh::shape_type::VERTEX_DATA: {
					// Create convex hull from vertex data
					JPH::Array<JPH::Vec3> Vertices;
					Vertices.reserve(this->PhysicsMesh->Vertex.size());
					for (const auto& Vert : this->PhysicsMesh->Vertex) {
						Vertices.push_back(JPH::Vec3(Vert.Position[0], Vert.Position[1], Vert.Position[2]));
					}
					ShapeSettings = new JPH::ConvexHullShapeSettings(Vertices);
					break;
				}
				default: {
					// Default to box shape with unit dimensions
					ShapeSettings = new JPH::BoxShapeSettings(JPH::Vec3(0.5f, 0.5f, 0.5f));
					break;
				}
			}
		} else {
			// No mesh provided, use default box shape
			ShapeSettings = new JPH::BoxShapeSettings(JPH::Vec3(0.5f, 0.5f, 0.5f));
		}
		
		// Create shape from settings
		JPH::ShapeSettings::ShapeResult ShapeResult = ShapeSettings->Create();
		JPH::ShapeRefC Shape = ShapeResult.Get();
		
		// Convert node position to Jolt format
		JPH::RVec3 JoltPosition(this->Position[0], this->Position[1], this->Position[2]);
		
		// Convert node orientation to Jolt format
		// geodesy quaternion: [0]=w, [1]=x, [2]=y, [3]=z
		// Jolt uses x,y,z,w constructor order
		JPH::Quat JoltOrientation(this->Orientation[1], this->Orientation[2], this->Orientation[3], this->Orientation[0]);
		
		// Create body creation settings
		// ObjectLayer is directly mapped from EMotionType (0=Static, 1=Kinematic, 2=Dynamic)
		JPH::ObjectLayer Layer = static_cast<JPH::ObjectLayer>(this->Motion);
		
		JPH::BodyCreationSettings BodySettings(
			Shape,
			JoltPosition,
			JoltOrientation,
			this->Motion,
			Layer
		);
		
		// Set mass properties for dynamic bodies
		if (this->Motion == JPH::EMotionType::Dynamic) {
			JPH::MassProperties MassProps;
			MassProps.mMass = this->Mass;
			// Convert inertia tensor to Jolt format
			MassProps.mInertia = JPH::Mat44(
				JPH::Vec4(this->InertiaTensor(0, 0), this->InertiaTensor(0, 1), this->InertiaTensor(0, 2), 0.0f),
				JPH::Vec4(this->InertiaTensor(1, 0), this->InertiaTensor(1, 1), this->InertiaTensor(1, 2), 0.0f),
				JPH::Vec4(this->InertiaTensor(2, 0), this->InertiaTensor(2, 1), this->InertiaTensor(2, 2), 0.0f),
				JPH::Vec4(0.0f, 0.0f, 0.0f, 1.0f)
			);
			BodySettings.mMassPropertiesOverride = MassProps;
			BodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
		}
		
		// Create and add body to physics system
		JPH::BodyInterface& BodyInterface = aPhysicsSystem->GetBodyInterface();
		JPH::Body* Body = BodyInterface.CreateBody(BodySettings);
		
		if (Body != nullptr) {
			// Store body ID
			this->JoltBodyID = Body->GetID();
			
			// Add body to physics system
			BodyInterface.AddBody(this->JoltBodyID, JPH::EActivation::Activate);
		}
	}

	node::~node() {
		// Clear all child nodes memory.
		for (auto& C : this->Child) {
			delete C; // Delete each child node.
		}
	}

	size_t node::node_count() const {
		size_t TotalCount = 1;
		for (auto Chd : Child) {
			TotalCount += Chd->node_count();
		}
		return TotalCount;
	}

	// The main transform function that calculates the model transform for this node.
	math::mat<float, 4, 4> node::transform() const {
		// Recursively calculates the world transform for this node using current state of the node hierarchy.
		if (this->Root != this) {
			return this->Parent->transform() * this->TransformToParentCurrent;
		}
		else {
			return this->TransformToParentCurrent; // If this is the root node, return the current transform.
		}
	}

	node* node::find(std::string aName) {
		// Find the node with the given name in the hierarchy.
		if (this->Identifier == aName) {
			return this;
		}
		for (const auto& child : this->Child) {
			node* FoundNode = child->find(aName);
			if (FoundNode != nullptr) {
				return FoundNode;
			}
		}
		return nullptr; // Return nullptr if not found.
	}

	std::vector<node*> node::linearize() {
		std::vector<node*> Nodes;
		Nodes.push_back(this);
		for (auto Chd : this->Child) {
			std::vector<node*> CNodes = Chd->linearize();
			Nodes.insert(Nodes.end(), CNodes.begin(), CNodes.end());
		}
		return Nodes;
	}

	void node::set_root(node* aRootNode) {
		this->Root = aRootNode; // Set the root node for this node.
		for (auto& Chd : this->Child) {
			Chd->set_root(aRootNode);
		}
	}

	void node::copy_data(const node* aNode) {
		// This function simply copies all data not related to the hierarchy.
		// This is used to copy data from one node to another.
		this->Identifier = aNode->Identifier;
		this->Mass = aNode->Mass;
		this->InertiaTensor = aNode->InertiaTensor;
		this->Position = aNode->Position;
		this->Orientation = aNode->Orientation;
		this->Scale = aNode->Scale;
		this->LinearMomentum = aNode->LinearMomentum;
		this->AngularMomentum = aNode->AngularMomentum;
		this->TransformToParentDefault = aNode->TransformToParentDefault;
		this->TransformToParentCurrent = aNode->TransformToParentCurrent; // Copy the current transform.
		this->TransformToWorld = aNode->TransformToWorld; // Copy the global transform.
		this->PhysicsMesh = aNode->PhysicsMesh; // Copy the physics mesh if it exists.
	}

	void node::copy(const node* aNode) {}

	void node::swap(node* aNode) {
		// Swap the data of this node with the given node.
		if (aNode == nullptr) {
			return; // Nothing to swap with.
		}
		// Copy node data
		this->copy_data(aNode);
		this->Child.swap(aNode->Child);
		for (auto& Chd : this->Child) {
			// Set parent for immediate children
			Chd->Parent = this;
			// Set root for each child node tree
			Chd->set_root(this->Root);
		}
	}

	void node::host_update(
		double 									aDeltaTime, 
		double 									aTime, 
		const std::vector<force>& 				aAppliedForces,
		const std::vector<phys::animation>& 	aPlaybackAnimation,
		const std::vector<float>& 				aAnimationWeight
	) {
		//tex:
		// It is the responsibility of the model class to insure that the sum of the contribution
		// factors (weights) is equal to 1.
		// $$ 1 = w^{b} + \sum_{\forall A \in Anim} w_{i} $$
		// $$ T = T^{base} \cdot w^{base} + \sum_{\forall i \in A} T_{i}^{A} \cdot w_{i}^{A} $$ 

		// No Animation Data, just use bind pose.
		if (!(aPlaybackAnimation.size() > 0 ? aPlaybackAnimation.size() + 1 == aAnimationWeight.size() : false)) return;

		// Bind Pose Transform
		this->TransformToParentCurrent = (this->TransformToParentDefault * aAnimationWeight[0]);

		// TODO: Figure out how to load animations per node. Also incredibly slow right now. Optimize Later.
		// Overrides/Averages Animation Transformations with Bind Pose Transform based on weights.
		for (size_t i = 0; i < aPlaybackAnimation.size(); i++) {
			// Check if Animation Data exists for this node, if not, use bind pose.
			// Pull animation for readability.
			auto& NodeAnimation = aPlaybackAnimation[i][this->Identifier];
			float Weight = aAnimationWeight[i + 1];
			if (NodeAnimation.exists()) {
				// Calculate time in ticks
				float TickerTime = aTime * aPlaybackAnimation[i].TicksPerSecond;
				// Ensure TickerTime is within the bounds of the animation.
				float BoundedTickerTime = std::fmod(TickerTime, aPlaybackAnimation[i].Stop - aPlaybackAnimation[i].Start) + aPlaybackAnimation[i].Start;
				if (this->Root == this) {
					this->TransformToParentCurrent += this->TransformToParentDefault * NodeAnimation[BoundedTickerTime] * Weight;
				}
				else {
					this->TransformToParentCurrent += NodeAnimation[BoundedTickerTime] * Weight;
				}
			}
			else {
				// Animation Data Does Not Exist, use bind pose animation.
				this->TransformToParentCurrent += this->TransformToParentDefault * Weight;
			}
		}
	}

	void node::device_update(
		double 									aDeltaTime, 
		double 									aTime, 
		const std::vector<phys::force>& 		aAppliedForces
	) {
		// Does nothing, by definition, agnostic of GPU module.
	}

	void node::recalculate_parent_transforms_and_local_data() {
		// Calculate TransformToParentCurrent from parent's world transform and this node's world transform
		// Formula: C->TransformToParent = P->InverseTransformToWorld * C->TransformToWorld
		// Note: This function is only called on child nodes, never on root nodes (handled in world.cpp)
		this->TransformToParentCurrent = this->Parent->InverseTransformToWorld * this->TransformToWorld;
		
		// Extract local position from column 4 of TransformToParentCurrent
		this->Position = math::vec<float, 3>(
			this->TransformToParentCurrent(0, 3),
			this->TransformToParentCurrent(1, 3),
			this->TransformToParentCurrent(2, 3)
		);
		
		// Extract upper 3x3 rotation matrix from TransformToParentCurrent
		math::mat<float, 3, 3> LocalRotationMatrix;
		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 3; col++) {
				LocalRotationMatrix(row, col) = this->TransformToParentCurrent(row, col);
			}
		}
		
		// Extract scale and normalize rotation matrix (decompose rotation from scale)
		for (int col = 0; col < 3; col++) {
			math::vec<float, 3> axis = {
				LocalRotationMatrix(0, col),
				LocalRotationMatrix(1, col),
				LocalRotationMatrix(2, col)
			};
			// Extract scale from column length
			this->Scale[col] = math::length(axis);
			// Normalize axis to get pure rotation
			axis /= this->Scale[col];
			LocalRotationMatrix(0, col) = axis[0];
			LocalRotationMatrix(1, col) = axis[1];
			LocalRotationMatrix(2, col) = axis[2];
		}
		
		// Convert rotation matrix to quaternion
		this->Orientation = math::quat(LocalRotationMatrix);
		
		// Recursively process all children
		for (auto& ChildNode : this->Child) {
			ChildNode->recalculate_parent_transforms_and_local_data();
		}
	}
	
}