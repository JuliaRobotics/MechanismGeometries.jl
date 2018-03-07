__precompile__(true)

module MechanismGeometries

using GeometryTypes
using StaticArrays
using RigidBodyDynamics
using ColorTypes: RGBA
using Rotations: rotation_between, RotMatrix, AngleAxis
using CoordinateTransformations: AffineMap, Transformation, 
                                 IdentityTransformation, 
                                 LinearMap, Translation

export create_skeleton, VisualElement, parse_urdf_visuals

const GeometryLike = Union{AbstractGeometry, AbstractMesh}
const DEFAULT_COLOR = RGBA{Float32}(0.7, 0.7, 0.7, 1.0)

mutable struct VisualElement{G <: GeometryLike, T <: Transformation}
    frame::CartesianFrame3D
    geometry::G
    color::RGBA{Float32}
    transform::T
end

include("skeleton.jl")
include("urdf.jl")
using .URDF

end # module
