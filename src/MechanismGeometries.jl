__precompile__(true)

module MechanismGeometries

using GeometryTypes
using StaticArrays
using RigidBodyDynamics
using Rotations: rotation_between, RotMatrix, AngleAxis
using CoordinateTransformations: AffineMap, Transformation, 
                                 IdentityTransformation, 
                                 LinearMap, Translation

export create_skeleton, VisualElement

const GeometryLike = Union{AbstractGeometry, AbstractMesh}

struct VisualElement{G, T <: Transformation}
    frame::CartesianFrame3D
    object::G
    transform::T
end

to_affine_map(tform::Transform3D) = AffineMap(rotation(tform), translation(tform))

include("construction.jl")

end # module
