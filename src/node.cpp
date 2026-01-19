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
		this->Root = this;
		this->Parent = nullptr;
		// Child vector is default initialized (empty)
		this->Identifier = "";
		this->Motion = JPH::EMotionType::Static;
		this->Collision = false;
		this->Position = { 0.0f, 0.0f, 0.0f };
		this->Orientation = { 1.0f, 0.0f, 0.0f, 0.0f }; // Identity quaternion (w, x, y, z)
		this->Scale = { 1.0f, 1.0f, 1.0f };
		this->Mass = 1.0f;
		this->InertiaTensor = {
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 1.0f
		};
		this->LinearMomentum = { 0.0f, 0.0f, 0.0f };
		this->AngularMomentum = { 0.0f, 0.0f, 0.0f };
		this->PhysicsMesh = nullptr;
		this->JoltBodyID = JPH::BodyID();
		// PendingForces, PendingImpulses, PendingTorques vectors are default initialized (empty)
		this->TransformToParentDefault = {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};
		this->TransformToParentCurrent = {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};
		this->TransformToWorld = {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};
		this->InverseTransformToWorld = {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};
		this->WorldScaleCache = { 1.0f, 1.0f, 1.0f };
	}

	node::node(JPH::PhysicsSystem* aPhysicsSystem, std::shared_ptr<phys::mesh> aPhysicsMesh) : node() {
		// ! I don't like this code, change later.
		this->PhysicsMesh = aPhysicsMesh;
		
		// Create a Jolt Body from phys::node and phys::mesh.
		if (aPhysicsSystem == nullptr || this->PhysicsMesh == nullptr) {
			return; // No physics system or mesh provided, skip body creation (no collision capability)
		}
		
		// Get shape from PhysicsMesh
		JPH::ShapeRefC Shape = nullptr;
		
		if (this->PhysicsMesh->Shape != nullptr) {
			// Use the shape already created in the physics mesh
			Shape = this->PhysicsMesh->Shape;
		} else {
			return; // No shape available, skip body creation (no collision capability)
		}
		
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
		
		// Configure collision behavior based on Collision flag
		// If Collision == false, make it a sensor (no physical collision response, but still moves)
		if (!this->Collision) {
			BodySettings.mIsSensor = true; // Sensor bodies don't physically collide but can still move
		}
		
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
		double 									aTime
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