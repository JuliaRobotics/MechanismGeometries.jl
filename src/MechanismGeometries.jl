VERSION < v"0.7.0-beta2.199" && __precompile__()

module MechanismGeometries

using LinearAlgebra
using GeometryTypes
using StaticArrays
using RigidBodyDynamics
using ColorTypes: RGBA
using Rotations: rotation_between, RotMatrix, AngleAxis
using CoordinateTransformations: AffineMap, Transformation,
                                 IdentityTransformation,
                                 LinearMap, Translation

export AbstractGeometrySource,
       VisualElement,
       visual_elements,
       Skeleton,
       HyperPlane,
       URDFVisuals,
       MeshFile

# Re-export from ColorTypes
export RGBA

abstract type AbstractGeometrySource end

function visual_elements(mechanism, source::AbstractGeometrySource) end

struct MeshFile
    filename::String
end

const GeometryLike = Union{AbstractGeometry, AbstractMesh, MeshFile}
const DEFAULT_COLOR = RGBA{Float32}(0.7, 0.7, 0.7, 1.0)

struct HyperPlane{N, T} <: GeometryPrimitive{N, T}
    normal::Vec{N, T}
end

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
